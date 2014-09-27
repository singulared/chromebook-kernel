/*
 * Copyright (C) 2013 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Based on code obtained from Analogix Inc with copyright:
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include "drmP.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"
#include "drm_dp_helper.h"
#include "anx7808regs.h"
#include "drm_atomic.h"
#include "drm_edid.h"

/*
 * _wait_for - magic (register) wait macro
 *
 * Does the right thing for modeset paths when run under kdgb or similar atomic
 * contexts. Note that it's important that we check the condition again after
 * having timed out, since the timeout could be due to preemption or similar and
 * we've never had a chance to check the condition before the timeout.
 */
#define _wait_for(COND, TO_MS, INTVL_MS) ({ \
	unsigned long timeout__ = jiffies + msecs_to_jiffies(TO_MS) + 1;\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (time_after(jiffies, timeout__)) {			\
			if (!(COND))					\
				ret__ = -ETIMEDOUT;			\
			break;						\
		}							\
		if (drm_can_sleep())  {					\
			usleep_range(INTVL_MS * 1000,			\
				(INTVL_MS + 5) * 1000);			\
		} else {						\
			cpu_relax();					\
		}							\
	}								\
	ret__;								\
})

#define wait_for(COND, TO_MS, INTVL_MS) _wait_for(COND, TO_MS, INTVL_MS)

#define HDMI_DETECT_TIMEOUT_MS 5000
#define HDMI_DETECT_INTERVAL_MS 100
#define LINK_TRAIN_TIMEOUT_MS 2000
#define LINK_TRAIN_INTERVAL_MS 100
#define AUX_WAIT_MS 100
#define AUX_BUFFER_SIZE 0x10
#define CABLE_DET_DEBOUNCE_MS 500
#define HDCP_EN_WAIT_MS 1500
/*
 * Time in ms to wait for R0 checking in HDCP first step authentication after
 * AKSV is written to receiver. Must be > 100ms per HDCP protocol.
 */
#define SP_TX_WAIT_R0_TIME_VALUE_MS 176
/*
 * Time to wait for downstream repeater KSVFIFO ready in HDCP first step
 * authentication. Must be > 4.2s per HDCP protocol.
 * 0x9C => 4.2s
 */
#define SP_TX_WAIT_KSVR_TIME_VALUE 0xC8
#define HDCP_MAX_TRIES 10

enum dp_link_bw {
	BW_OVER = 0xFF,
	BW_54G = 0x14,
	BW_27G = 0x0A,
	BW_162G = 0x06,
	BW_NULL = 0x00
};

enum downstream_type {
	DOWNSTREAM_UNKNOWN,
	DOWNSTREAM_DP,
	DOWNSTREAM_VGA,
	DOWNSTREAM_HDMI,
	DOWNSTREAM_DISCONNECTED,
};

struct anx7808_data {
	struct drm_connector connector;
	struct drm_device *dev;
	struct drm_encoder *encoder;
	int pd_gpio;
	int reset_gpio;
	int intp_gpio;
	int intp_irq;
	int cable_det_gpio;
	int cable_det_irq;
	atomic_t cable_det_oneshot;
	struct regulator *vdd_mydp;
	struct i2c_client *tx_p0;
	struct i2c_client *tx_p1;
	struct i2c_client *tx_p2;
	struct i2c_client *rx_p0;
	struct i2c_client *rx_p1;
	struct delayed_work cable_det_work;
	struct delayed_work hdcp_en_work;
	enum dp_link_bw dp_manual_bw;
	int dp_manual_bw_changed;
	enum downstream_type ds_type;
	bool powered;
	bool recv_audio;
	bool recv_cts;
	bool hdcp_desired;
	int hdcp_tries;
	struct mutex big_lock;
	struct i2c_algorithm i2c_algorithm;
	struct i2c_adapter ddc_adapter;
	struct dentry *debugfs;
};

static struct i2c_client *anx7808_addr_to_client(struct anx7808_data *anx7808,
						 uint16_t addr)
{
	switch (addr >> 8) {
	case TX_P0:
		return anx7808->tx_p0;
	case TX_P1:
		return anx7808->tx_p1;
	case TX_P2:
		return anx7808->tx_p2;
	case RX_P0:
		return anx7808->rx_p0;
	case RX_P1:
		return anx7808->rx_p1;
	default:
		break;
	}
	DRM_ERROR("Invalid i2c address %04x.\n", addr);
	return NULL;
}

static int anx7808_read_reg(struct anx7808_data *anx7808, uint16_t addr,
			    uint8_t *value)
{
	int ret = 0;
	struct i2c_client *client = anx7808_addr_to_client(anx7808, addr);

	WARN_ON(!mutex_is_locked(&anx7808->big_lock));
	if (!anx7808->powered)
		return -ENODEV;

	if (client == NULL) {
		*value = 0;
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(client, addr & 0xFF);
	if (ret < 0) {
		DRM_ERROR("Failed to read i2c addr=%04x.\n", addr);
		*value = 0;
		return ret;
	}
	*value = ret;
	return 0;
}

static int anx7808_write_reg(struct anx7808_data *anx7808, uint16_t addr,
			     uint8_t value)
{
	int ret = 0;
	struct i2c_client *client = anx7808_addr_to_client(anx7808, addr);

	WARN_ON(!mutex_is_locked(&anx7808->big_lock));
	if (!anx7808->powered)
		return -ENODEV;

	if (client == NULL)
		return -EINVAL;

	ret = i2c_smbus_write_byte_data(client, addr & 0xFF, value);
	if (ret < 0) {
		DRM_ERROR("Failed to write i2c addr=%04x.\n", addr);
		return -EIO;
	}
	return 0;
}

static int anx7808_set_bits(struct anx7808_data *anx7808, uint16_t addr,
			    uint8_t bits)
{
	uint8_t c;
	int ret = 0;
	ret = anx7808_read_reg(anx7808, addr, &c);
	if (ret)
		return ret;
	return anx7808_write_reg(anx7808, addr, c | bits);
}

static int anx7808_clear_bits(struct anx7808_data *anx7808, uint16_t addr,
			      uint8_t bits)
{
	uint8_t c;
	int ret = 0;
	ret = anx7808_read_reg(anx7808, addr, &c);
	if (ret)
		return ret;
	return anx7808_write_reg(anx7808, addr, c & (~bits));
}

static int anx7808_copy_regs(struct anx7808_data *anx7808, uint16_t from,
			      uint16_t to, int length)
{
	int i;
	uint8_t c;
	int err = 0;

	for (i = 0; i < length; i++) {
		err |= anx7808_read_reg(anx7808, from + i, &c);
		err |= anx7808_write_reg(anx7808, to + i, c);
	}

	if (err)
		return -EIO;
	return 0;
}

static void anx7808_set_hpd(struct anx7808_data *anx7808, bool enable)
{
	if (!enable) {
		anx7808_clear_bits(anx7808, SP_TX_VID_CTRL3_REG, HPD_OUT);
		anx7808_set_bits(anx7808, HDMI_RX_TMDS_CTRL_REG6, TERM_PD);
	} else {
		anx7808_clear_bits(anx7808, HDMI_RX_TMDS_CTRL_REG6, TERM_PD);
		anx7808_set_bits(anx7808, SP_TX_VID_CTRL3_REG, HPD_OUT);
	}
}

static void anx7808_rx_initialization(struct anx7808_data *anx7808)
{
	DRM_DEBUG_KMS("Initializing ANX7808 receiver.\n");

	anx7808_write_reg(anx7808, HDMI_RX_HDMI_MUTE_CTRL_REG,
			  AUD_MUTE | VID_MUTE);
	anx7808_set_bits(anx7808, HDMI_RX_CHIP_CTRL_REG,
			 MAN_HDMI5V_DET | PLLLOCK_CKDT_EN | DIGITAL_CKDT_EN);
	anx7808_set_bits(anx7808, HDMI_RX_AEC_CTRL_REG, AVC_OE);
	anx7808_set_bits(anx7808, HDMI_RX_SRST_REG, HDCP_MAN_RST);
	usleep_range(1000, 2000);
	anx7808_clear_bits(anx7808, HDMI_RX_SRST_REG, HDCP_MAN_RST);
	anx7808_set_bits(anx7808, HDMI_RX_SRST_REG, SW_MAN_RST);
	usleep_range(1000, 2000);
	anx7808_clear_bits(anx7808, HDMI_RX_SRST_REG, SW_MAN_RST);
	anx7808_set_bits(anx7808, HDMI_RX_SRST_REG, TMDS_RST);
	anx7808_write_reg(anx7808, HDMI_RX_AEC_EN0_REG,
			  AEC_EN07 | AEC_EN06 | AEC_EN05 | AEC_EN02);
	anx7808_write_reg(anx7808, HDMI_RX_AEC_EN1_REG,
			  AEC_EN12 | AEC_EN10 | AEC_EN09 | AEC_EN08);
	anx7808_write_reg(anx7808, HDMI_RX_AEC_EN2_REG,
			  AEC_EN23 | AEC_EN22 | AEC_EN21 | AEC_EN20);
	anx7808_set_bits(anx7808, HDMI_RX_AEC_CTRL_REG, AVC_EN);
	anx7808_clear_bits(anx7808, HDMI_RX_SYS_PWDN1_REG, PWDN_CTRL);

	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK1_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK2_REG,
			AUTH_START | AUTH_DONE | HDCP_ERR);
	anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS2_REG,
			AUTH_START | AUTH_DONE | HDCP_ERR);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK3_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK4_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK7_REG, 0x00);

	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK5_REG, AUDIO_RCV);
	anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS5_REG, AUDIO_RCV);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK6_REG,
			CTS_RCV | NEW_AUD | NEW_AVI);
	anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS6_REG,
			CTS_RCV | NEW_AUD | NEW_AVI);

	anx7808_set_bits(anx7808, HDMI_RX_VID_DATA_RNG_CTRL_REG,
			 R2Y_INPUT_LIMIT);
	anx7808_write_reg(anx7808, HDMI_RX_CEC_CTRL_REG, CEC_RST);
	anx7808_write_reg(anx7808, HDMI_RX_CEC_SPEED_CTRL_REG, CEC_SPEED_27M);
	anx7808_write_reg(anx7808, HDMI_RX_CEC_CTRL_REG, CEC_RX_EN);

	anx7808_write_reg(anx7808, HDMI_RX_TMDS_CTRL_REG2, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_TMDS_CTRL_REG4, 0x28);
	anx7808_write_reg(anx7808, HDMI_RX_TMDS_CTRL_REG5, 0xE3);
	anx7808_write_reg(anx7808, HDMI_RX_TMDS_CTRL_REG7, 0x70);
	anx7808_write_reg(anx7808, HDMI_RX_TMDS_CTRL_REG19, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_TMDS_CTRL_REG21, 0x04);
	anx7808_write_reg(anx7808, HDMI_RX_TMDS_CTRL_REG22, 0x38);

	anx7808_set_hpd(anx7808, 0);
}

static void anx7808_tx_initialization(struct anx7808_data *anx7808)
{
	DRM_DEBUG_KMS("Initializing ANX7808 transmitter.\n");

	anx7808_write_reg(anx7808, SP_TX_EXTRA_ADDR_REG, I2C_EXTRA_ADDR);
	anx7808_set_bits(anx7808, SP_TX_HDCP_CTRL, LINK_POLLING);
	anx7808_clear_bits(anx7808, SP_TX_HDCP_CTRL, AUTO_START | AUTO_EN);
	anx7808_set_bits(anx7808, SP_TX_LINK_DEBUG_REG, M_VID_DEBUG);
	anx7808_set_bits(anx7808, SP_TX_DEBUG_REG1,
			 FORCE_HPD | FORCE_PLL_LOCK | POLLING_EN);
	anx7808_set_bits(anx7808, SP_TX_PLL_FILTER_CTRL1, AUX_TERM_50OHM);
	anx7808_clear_bits(anx7808, SP_TX_PLL_FILTER_CTRL5,
			   P5V_PROTECT_PD | SHORT_PROTECT_PD);
	anx7808_set_bits(anx7808, SP_TX_ANALOG_DEBUG_REG2, POWERON_TIME_1P5MS);
	anx7808_set_bits(anx7808, SP_TX_HDCP_CTRL0_REG,
			 BKSV_SRM_PASS | KSVLIST_VLD);

	anx7808_write_reg(anx7808, SP_TX_ANALOG_CTRL, 0xC5);
	anx7808_write_reg(anx7808, I2C_GEN_10US_TIMER0, 0x0E);
	anx7808_write_reg(anx7808, I2C_GEN_10US_TIMER1, 0x01);

	anx7808_write_reg(anx7808, SP_TX_DP_POLLING_CTRL_REG,
			  AUTO_POLLING_DISABLE);
	anx7808_write_reg(anx7808, SP_TX_LINK_CHK_TIMER, 0x1D);

	anx7808_set_bits(anx7808, SP_TX_MISC_CTRL_REG, EQ_TRAINING_LOOP);

	anx7808_write_reg(anx7808, SP_COMMON_INT_MASK1, 0x00);
	anx7808_write_reg(anx7808, SP_COMMON_INT_MASK2, 0x00);
	anx7808_write_reg(anx7808, SP_COMMON_INT_MASK3, 0x00);
	anx7808_write_reg(anx7808, SP_COMMON_INT_MASK4, 0x00);
	anx7808_write_reg(anx7808, SP_TX_INT_CTRL_REG, 0x01);

	anx7808_write_reg(anx7808, SP_INT_MASK, DPCD_IRQ_REQUEST | POLLING_ERR);
	anx7808_write_reg(anx7808, SP_TX_INT_STATUS1,
			DPCD_IRQ_REQUEST | POLLING_ERR);

	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG0, 0x19);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG4, 0x1B);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG7, 0x22);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG9, 0x23);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG14, 0x09);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG17, 0x16);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG19, 0x1F);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG1, 0x26);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG5, 0x28);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG8, 0x2F);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG15, 0x10);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG18, 0x1F);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG2, 0x36);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG6, 0x3C);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG16, 0x18);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG3, 0x3F);
}

static void anx7808_init_pipeline(struct anx7808_data *anx7808)
{
	anx7808_rx_initialization(anx7808);
	anx7808_tx_initialization(anx7808);
}

static int anx7808_power_on(struct anx7808_data *anx7808)
{
	int ret;

	WARN_ON(!mutex_is_locked(&anx7808->big_lock));

	if (anx7808->powered)
		return 0;

	DRM_INFO("Powering on ANX7808.\n");

	gpio_set_value(anx7808->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpio_set_value(anx7808->pd_gpio, 0);
	usleep_range(2000, 4000);
	ret = regulator_enable(anx7808->vdd_mydp);
	if (ret < 0) {
		DRM_ERROR("Failed to power on ANX7808: %d", ret);
		return ret;
	}
	msleep(20);

	anx7808->powered = true;

	gpio_set_value(anx7808->reset_gpio, 1);

	anx7808_clear_bits(anx7808, SP_POWERD_CTRL_REG, REGISTER_PD);
	anx7808_clear_bits(anx7808, SP_POWERD_CTRL_REG, TOTAL_PD);

	anx7808_init_pipeline(anx7808);

	/*
	 * This delay seems to help keep the hardware in a good state. Without
	 * it, there are times where it fails silently.
	 */
	usleep_range(10000, 15000);

	return ret;
}

static int anx7808_power_off(struct anx7808_data *anx7808, bool cancel_intp,
		bool reset_oneshot)
{
	int ret;

	WARN_ON(!mutex_is_locked(&anx7808->big_lock));

	if (!anx7808->powered)
		return 0;

	/* Must be nosync since the intp handler might be blocked on big_lock */
	if (cancel_intp)
		disable_irq_nosync(anx7808->intp_irq);

	DRM_INFO("Powering off ANX7808.\n");

	anx7808->powered = false;

	gpio_set_value(anx7808->reset_gpio, 0);
	usleep_range(1000, 2000);

	ret = regulator_disable(anx7808->vdd_mydp);
	if (ret < 0) {
		DRM_ERROR("Failed to power off ANX7808: %d", ret);
		goto out;
	}
	usleep_range(5000, 10000);
	if (reset_oneshot)
		atomic_set(&anx7808->cable_det_oneshot, 1);
	gpio_set_value(anx7808->pd_gpio, 1);

	/*
	 * Wait here for the cable_det line to settle. This time is not
	 * specified in the datasheet, however measurments show it does not
	 * exceed 10ms. Double that and add a half to be safe.
	 */
	msleep(25);

out:
	if (cancel_intp)
		enable_irq(anx7808->intp_irq);

	return ret;
}

static int anx7808_aux_wait(struct anx7808_data *anx7808)
{
	int err;
	uint8_t status, aux_ctrl;
	unsigned long start = jiffies;

	while ((jiffies - start) <= msecs_to_jiffies(AUX_WAIT_MS)) {
		usleep_range(100, 200);

		err = anx7808_read_reg(anx7808, SP_TX_AUX_CTRL_REG2, &aux_ctrl);
		if (err)
			return err;
		if (aux_ctrl & AUX_OP_EN)
			continue;

		err = anx7808_read_reg(anx7808, SP_TX_AUX_STATUS, &status);
		if (err)
			return err;
		if (status & AUX_BUSY)
			continue;

		break;
	}

	if (status || (aux_ctrl & AUX_OP_EN)) {
		DRM_ERROR("Failed to read AUX channel: 0x%02x 0x%02x\n",
			  status, aux_ctrl);
		return -EIO;
	}

	return 0;
}

static int anx7808_aux_read_block(struct anx7808_data *anx7808, uint32_t addr,
				  uint8_t cmd, uint8_t count, uint8_t *buf)
{
	int i;
	int err = 0;
	int addrl = (addr >> 0) & 0xFF;
	int addrm = (addr >> 8) & 0xFF;
	int addrh = (addr >> 16) & 0x0F;

	if (count > AUX_BUFFER_SIZE)
		return -EINVAL;

	err |= anx7808_write_reg(anx7808, SP_TX_BUF_DATA_COUNT_REG, BUF_CLR);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_CTRL_REG,
				 ((count - 1) << 4) | cmd | 0x01);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_ADDR_7_0_REG, addrl);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_ADDR_15_8_REG, addrm);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_ADDR_19_16_REG, addrh);
	err |= anx7808_set_bits(anx7808, SP_TX_AUX_CTRL_REG2, AUX_OP_EN);
	if (err)
		return -EIO;
	usleep_range(2000, 4000);

	err = anx7808_aux_wait(anx7808);
	if (err)
		return err;

	for (i = 0; i < count; i++)
		err |= anx7808_read_reg(anx7808, SP_TX_BUF_DATA_0_REG + i,
					buf + i);
	if (err)
		return -EIO;

	return 0;
}

static int anx7808_aux_write_block(struct anx7808_data *anx7808, uint32_t addr,
				   uint8_t cmd, uint8_t count, uint8_t *buf)
{
	int i;
	int err = 0;
	int addrl = (addr >> 0) & 0xFF;
	int addrm = (addr >> 8) & 0xFF;
	int addrh = (addr >> 16) & 0x0F;

	if (count > AUX_BUFFER_SIZE)
		return -EINVAL;

	err |= anx7808_write_reg(anx7808, SP_TX_BUF_DATA_COUNT_REG, BUF_CLR);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_CTRL_REG,
				 ((count - 1) << 4) | cmd);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_ADDR_7_0_REG, addrl);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_ADDR_15_8_REG, addrm);
	err |= anx7808_write_reg(anx7808, SP_TX_AUX_ADDR_19_16_REG, addrh);
	for (i = 0; i < count; i++)
		err |= anx7808_write_reg(anx7808, SP_TX_BUF_DATA_0_REG + i,
					*(buf + i));
	err |= anx7808_set_bits(anx7808, SP_TX_AUX_CTRL_REG2, AUX_OP_EN);

	if (err)
		return -EIO;

	return anx7808_aux_wait(anx7808);
}

static int anx7808_aux_read(struct anx7808_data *anx7808, uint32_t addr,
			    uint8_t cmd, uint8_t count, uint8_t *buf)
{
	int ret = 0;
	int offset, blocksize;

	WARN_ON(!mutex_is_locked(&anx7808->big_lock));
	for (offset = 0; offset < count; offset += AUX_BUFFER_SIZE) {
		blocksize = (count - offset > AUX_BUFFER_SIZE) ?
			     AUX_BUFFER_SIZE : count - offset;
		ret = anx7808_aux_read_block(anx7808, addr, cmd, blocksize,
					     &buf[offset]);
		if (ret)
			break;
	}
	return ret;
}

static int anx7808_aux_write(struct anx7808_data *anx7808, uint32_t addr,
			     uint8_t cmd, uint8_t count, uint8_t *buf)
{
	int ret = 0;
	int offset, blocksize;

	WARN_ON(!mutex_is_locked(&anx7808->big_lock));
	for (offset = 0; offset < count; offset += AUX_BUFFER_SIZE) {
		blocksize = (count - offset > AUX_BUFFER_SIZE) ?
			     AUX_BUFFER_SIZE : count - offset;
		ret = anx7808_aux_write_block(anx7808, addr, cmd, blocksize,
					      &buf[offset]);
		if (ret)
			break;
	}
	return ret;
}

static int anx7808_aux_dpcd_read(struct anx7808_data *anx7808, uint32_t addr,
				 uint8_t count, uint8_t *buf)
{
	return anx7808_aux_read(anx7808, addr, AUX_DPCD, count, buf);
}

static int anx7808_aux_dpcd_write(struct anx7808_data *anx7808, uint32_t addr,
				  uint8_t count, uint8_t *buf)
{
	return anx7808_aux_write(anx7808, addr, AUX_DPCD, count, buf);
}

static int anx7808_aux_ddc_read(struct anx7808_data *anx7808, uint32_t addr,
				uint8_t count, uint8_t *buf)
{
	return anx7808_aux_read(anx7808, addr, AUX_DDC, count, buf);
}

static int anx7808_aux_ddc_write(struct anx7808_data *anx7808, uint32_t addr,
				 uint8_t count, uint8_t *buf)
{
	return anx7808_aux_write(anx7808, addr, AUX_DDC, count, buf);
}

static int anx7808_detect_hdmi_input(struct anx7808_data *anx7808)
{
	uint8_t sys_status, hdmi_status, hdmi_enable;

	anx7808_read_reg(anx7808, HDMI_RX_SYS_STATUS_REG, &sys_status);
	if (!(sys_status & TMDS_CLOCK_DET)) {
		DRM_INFO("Waiting for HDMI clock.\n");
		return -EAGAIN;
	}
	if (!(sys_status & TMDS_DE_DET)) {
		DRM_INFO("Waiting for HDMI signal.\n");
		return -EAGAIN;
	}

	anx7808_write_reg(anx7808, HDMI_RX_HDMI_MUTE_CTRL_REG, 0x00);

	/* If this register write is moved to output config time, infoframes
	 * are not transmitted, but it is not clear why. */
	anx7808_read_reg(anx7808, HDMI_RX_HDMI_STATUS_REG, &hdmi_status);
	hdmi_enable = (hdmi_status & HDMI_MODE) ? 1 : 0;
	DRM_INFO("HDMI enable: %d\n", hdmi_enable);

	anx7808_aux_dpcd_write(anx7808, US_COMM_5, 1, &hdmi_enable);

	return 0;
}

static int anx7808_check_dp_link(struct anx7808_data *anx7808)
{
	int err;
	uint8_t status;

	err = anx7808_aux_dpcd_read(anx7808, DP_LANE0_1_STATUS, 1, &status);
	if (err)
		return err;
	if ((status & DP_CHANNEL_EQ_BITS) != DP_CHANNEL_EQ_BITS) {
		DRM_INFO("Waiting for DP Lane 0 to train: %02x\n", status);
		return -EAGAIN;
	}

	return 0;
}

static int anx7808_dp_link_training(struct anx7808_data *anx7808)
{
	int err;
	uint8_t dp_bw;

	err = anx7808_aux_dpcd_read(anx7808, DP_MAX_LINK_RATE, 1, &dp_bw);
	if (err)
		return err;
	switch ((enum dp_link_bw)dp_bw) {
	case BW_162G:
	case BW_27G:
	case BW_54G:
		break;
	case BW_NULL:
	case BW_OVER:
	default:
		DRM_INFO("Waiting to read DP bandwidth.\n");
		return -EAGAIN;
	}

	if (anx7808->dp_manual_bw != 0)
		dp_bw = (uint8_t)anx7808->dp_manual_bw;
	anx7808_write_reg(anx7808, SP_TX_LINK_BW_SET_REG, dp_bw);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG, SP_TX_LT_EN);
	anx7808_set_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_MUTE);
	anx7808_clear_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_EN);

	err = anx7808_check_dp_link(anx7808);
	if (err)
		return err;

	anx7808_clear_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_MUTE);

	return 0;
}

static void anx7808_config_dp_output(struct anx7808_data *anx7808)
{
	anx7808_set_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_EN);
}

static void anx7808_update_video_infoframe(struct anx7808_data *anx7808)
{
	anx7808_clear_bits(anx7808, SP_TX_PKT_EN_REG, AVI_IF_EN);
	anx7808_write_reg(anx7808, SP_TX_AVI_TYPE, SP_TX_AVI_KTYPE);
	anx7808_write_reg(anx7808, SP_TX_AVI_VER, SP_TX_AVI_KVER);
	anx7808_write_reg(anx7808, SP_TX_AVI_LEN, SP_TX_AVI_KLEN);
	anx7808_copy_regs(anx7808, HDMI_RX_AVI_DATA00_REG, SP_TX_AVI_DB0,
			SP_TX_AVI_KLEN);
	anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AVI_IF_UD);
	anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AVI_IF_EN);
}

static void anx7808_update_audio_infoframe(struct anx7808_data *anx7808)
{
	anx7808_clear_bits(anx7808, SP_TX_PKT_EN_REG, AUD_IF_EN);
	anx7808_write_reg(anx7808, SP_TX_AUD_TYPE, SP_TX_AUD_KTYPE);
	anx7808_write_reg(anx7808, SP_TX_AUD_VER, SP_TX_AUD_KVER);
	anx7808_write_reg(anx7808, SP_TX_AUD_LEN, SP_TX_AUD_KLEN);
	anx7808_copy_regs(anx7808, HDMI_RX_AUDIO_DATA00_REG, SP_TX_AUD_DB0,
			SP_TX_AUD_KLEN);
	anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AUD_IF_UP);
	anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AUD_IF_EN);
}

static void anx7808_update_audio(struct anx7808_data *anx7808)
{
	uint8_t force_audio[3] = {0x06, 0x85, 0x08};

	anx7808->recv_cts = false;
	anx7808->recv_audio = false;

	anx7808_clear_bits(anx7808, HDMI_RX_HDMI_MUTE_CTRL_REG,
			   AUD_MUTE);

	anx7808_write_reg(anx7808, SP_TX_AUD_INTERFACE_CTRL4, 0x05);
	anx7808_write_reg(anx7808, SP_TX_AUD_INTERFACE_CTRL5, 0x00);
	anx7808_write_reg(anx7808, SP_TX_AUD_INTERFACE_CTRL6, 0x00);

	anx7808_clear_bits(anx7808, SP_TX_AUD_INTERFACE_CTRL0,
			   AUD_INTERFACE_DISABLE);

	anx7808_set_bits(anx7808, SP_TX_AUD_INTERFACE_CTRL2,
			 M_AUD_ADJUST_ST);

	anx7808_write_reg(anx7808, SP_TX_AUD_CH_NUM_REG5, 0);

	anx7808_copy_regs(anx7808, HDMI_RX_AUD_IN_CH_STATUS1_REG,
			  SP_TX_AUD_CH_STATUS_REG1, 5);

	if (anx7808->ds_type == DOWNSTREAM_HDMI)
		anx7808_aux_dpcd_write(anx7808, SINK_DEV_SEL, 3, force_audio);

	/* enable audio */
	anx7808_set_bits(anx7808, SP_TX_AUD_CTRL, AUD_EN);

	return;
}

static void anx7808_config_downstream(struct anx7808_data *anx7808)
{
	anx7808_config_dp_output(anx7808);
	anx7808_update_video_infoframe(anx7808);
	anx7808_update_audio_infoframe(anx7808);
	anx7808_update_audio(anx7808);
}

static int anx7808_get_downstream_info(struct anx7808_data *anx7808)
{
	int ret;
	uint8_t val;

	ret = anx7808_aux_dpcd_read(anx7808, DP_SINK_COUNT, 1, &val);
	if (ret) {
		DRM_ERROR("Get sink count failed %d\n", ret);
		return ret;
	}
	if (!(DP_GET_SINK_COUNT(val))) {
		anx7808->ds_type = DOWNSTREAM_DISCONNECTED;
		return 0;
	}

	ret = anx7808_aux_dpcd_read(anx7808, DP_DOWNSTREAMPORT_PRESENT,
				    1, &val);
	if (ret) {
		DRM_ERROR("Failed to get DPCD downstream present %d\n", ret);
		return ret;
	}

	switch (val & DP_DWN_STRM_PORT_TYPE_MASK) {
	case DOWNSTREAMPORT_DP:
		anx7808->ds_type = DOWNSTREAM_DP;
		break;
	case DOWNSTREAMPORT_HDMI:
		anx7808->ds_type = DOWNSTREAM_HDMI;
		break;
	case DOWNSTREAMPORT_VGA:
		anx7808->ds_type = DOWNSTREAM_VGA;
		break;
	default:
		anx7808->ds_type = DOWNSTREAM_UNKNOWN;
		DRM_ERROR("Unknown downstream type DS_PRESENT=%x.\n", val);
		break;
	}

	return ret;
}

static int anx7808_get_ds_video_status(struct anx7808_data *anx7808)
{
	uint8_t reg_value;
	anx7808_aux_dpcd_read(anx7808, US_COMM_6, 1, &reg_value);
	/*
	 * This a ANX7730 specific register access via DPCD.
	 * No detailed info about this "Upstream command byte 6"
	 * in the ANX7730 register spec.
	 */
	return reg_value & 0x01;
}

static void anx7808_update_hdcp_property(struct anx7808_data *anx7808,
		bool enabled)
{
	struct drm_device *dev = anx7808->connector.dev;
	uint64_t val = DRM_MODE_CONTENT_PROTECTION_OFF;

	if (enabled)
		val = DRM_MODE_CONTENT_PROTECTION_ENABLED;
	else if (anx7808->hdcp_desired)
		val = DRM_MODE_CONTENT_PROTECTION_DESIRED;

	drm_object_property_set_value(&anx7808->connector.base,
			&anx7808->connector.propvals,
			dev->mode_config.content_protection_property,
			val, NULL);
}

static int anx7808_power_on_hdcp(struct anx7808_data *anx7808)
{
	int ret = 0;

	ret |= anx7808_clear_bits(anx7808, SP_POWERD_CTRL_REG, HDCP_PD);
	ret |= anx7808_set_bits(anx7808, SP_COMMON_INT_MASK2, HDCP_AUTH_DONE);

	msleep(50);
	return ret;
}

static int anx7808_power_off_hdcp(struct anx7808_data *anx7808)
{
	int ret = 0;

	ret |= anx7808_clear_bits(anx7808, SP_COMMON_INT_MASK2, HDCP_AUTH_DONE);
	ret |= anx7808_set_bits(anx7808, SP_POWERD_CTRL_REG, HDCP_PD);

	return ret;
}

static int anx7808_start_hdcp(struct anx7808_data *anx7808)
{
	int ret = 0;
	int i;
	uint8_t reg_value;

	DRM_DEBUG("Starting HDCP for ANX7808\n");
	WARN_ON(!mutex_is_locked(&anx7808->big_lock));

	/* Sink HDCP capability check */
	ret = anx7808_aux_dpcd_read(anx7808, BCAPS, 1, &reg_value);
	if (ret)
		goto out;
	if (!(reg_value & HDCP_CAPABLE)) {
		DRM_INFO("ANX7808 DP sink is not HDCP capable\n");
		ret = -EPERM;
		goto out;
	}

	/* In case ANX7730 video can not get ready */
	if (anx7808->ds_type == DOWNSTREAM_HDMI) {
		for (i = 0; i < 10; i++) {
			ret = anx7808_get_ds_video_status(anx7808);
			if (ret)
				break;
			usleep_range(5000, 20000);
		}
		if (!ret) {
			DRM_ERROR("Downstream video is not ready\n");
			ret = -EPERM;
			goto out;
		}
	}

	/* Issue HDCP after the HDMI Rx key loaddown */
	ret = anx7808_read_reg(anx7808, HDMI_RX_HDCP_STATUS_REG, &reg_value);
	if (ret)
		goto out;
	if (reg_value & AUTH_EN) {
		for (i = 0; i < 10; i++) {
			ret = anx7808_read_reg(anx7808,
					HDMI_RX_HDCP_STATUS_REG, &reg_value);
			if (ret)
				goto out;
			if (reg_value & LOAD_KEY_DONE)
				break;
			usleep_range(10000, 20000);
		}
		if (!(reg_value & LOAD_KEY_DONE)) {
			DRM_ERROR("HDMI Rx HDCP key load took > 100ms\n");
			ret = -EPERM;
			goto out;
		}
	} else {
		/*
		 * DP Tx HDCP should be enabled to protect AV content even
		 * if HDCP cannot be enabled on the HDMI Rx side somehow.
		 */
		DRM_INFO("ANX7808 upstream HDCP auth is not attempted\n");
	}

	ret = anx7808_power_on_hdcp(anx7808);
	if (ret)
		goto out;

	ret |= anx7808_clear_bits(anx7808, SP_TX_HDCP_CTRL0_REG,
			ENC_EN | HARD_AUTH_EN);
	ret |= anx7808_set_bits(anx7808, SP_TX_HDCP_CTRL0_REG,
			ENC_EN | HARD_AUTH_EN | BKSV_SRM_PASS | KSVLIST_VLD);
	ret |= anx7808_write_reg(anx7808, SP_TX_WAIT_R0_TIME,
			SP_TX_WAIT_R0_TIME_VALUE_MS);
	ret |= anx7808_write_reg(anx7808, SP_TX_WAIT_KSVR_TIME,
			SP_TX_WAIT_KSVR_TIME_VALUE);

out:
	/*
	 * If HDCP is desired but it cannot be started, the 'Content Protection'
	 * property on the connector stays as "Desired", and will not be
	 * updated to "Enabled".
	 */
	return ret;
}

static int anx7808_stop_hdcp(struct anx7808_data *anx7808)
{
	int ret = 0;

	DRM_DEBUG("Stopping HDCP for ANX7808\n");
	WARN_ON(!mutex_is_locked(&anx7808->big_lock));

	cancel_delayed_work(&anx7808->hdcp_en_work);

	anx7808_update_hdcp_property(anx7808, false);

	if (!anx7808->powered)
		return ret;

	/* Disable encryption and clean hdcp status */
	ret |= anx7808_write_reg(anx7808, SP_TX_HDCP_CTRL0_REG, 0x00);

	/* HDCP reauth */
	ret |= anx7808_set_bits(anx7808, SP_TX_HDCP_CTRL0_REG, RE_AUTH);
	ret |= anx7808_clear_bits(anx7808, SP_TX_HDCP_CTRL0_REG, RE_AUTH);

	ret |= anx7808_power_off_hdcp(anx7808);

	return ret;
}

static int anx7808_reset_hdcp(struct anx7808_data *anx7808)
{
	int ret = 0;
	DRM_DEBUG("Resetting HDCP for ANX7808\n");

	ret = anx7808_stop_hdcp(anx7808);
	if (!anx7808->hdcp_desired)
		return ret;

	anx7808->hdcp_tries++;
	if (anx7808->hdcp_tries >= HDCP_MAX_TRIES) {
		DRM_ERROR("Maximum HDCP tries exceeded, giving up\n");
		return -EPERM;
	}

	ret = anx7808_start_hdcp(anx7808);
	return ret;
}

static void anx7808_handle_hdcp_auth_done_int(struct anx7808_data *anx7808)
{
	int ret;
	uint8_t reg_value;
	uint8_t bytebuf[2];

	ret = anx7808_read_reg(anx7808, SP_TX_HDCP_STATUS, &reg_value);
	if (ret)
		return;
	if (!(reg_value & SP_TX_HDCP_AUTH_PASS)) {
		DRM_ERROR("ANX7808 downstream HDCP authentication failed %d\n",
				reg_value);
		anx7808_reset_hdcp(anx7808);
		return;
	}

	ret = anx7808_aux_dpcd_read(anx7808, BINFO_L, 2, bytebuf);
	if (ret)
		return;
	if (bytebuf[1] & MAX_CASCADE_EXCEEDED) {
		DRM_ERROR("HDCP max cascade level reached\n");
		anx7808_reset_hdcp(anx7808);
	} else {
		DRM_DEBUG("ANX7808 downstream HDCP authentication passed\n");
		anx7808_set_bits(anx7808, SP_TX_HDCP_CTRL0_REG, ENC_EN);
		anx7808_update_hdcp_property(anx7808, true);
	}
}

static void anx7808_handle_common_int_2(struct anx7808_data *anx7808,
		uint8_t irq)
{
	int ret;

	ret = anx7808_write_reg(anx7808, SP_COMMON_INT_STATUS2, irq);
	if (ret)
		DRM_ERROR("Failed to write SP_COMMON_INT_STATUS2 %d\n", ret);

	if (irq & HDCP_AUTH_DONE)
		anx7808_handle_hdcp_auth_done_int(anx7808);
}

static void anx7808_handle_hdmi_int_2(struct anx7808_data *anx7808, uint8_t irq)
{
	int ret;

	ret = anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS2_REG, irq);
	if (ret)
		DRM_ERROR("Write hdmi int 2 failed %d\n", ret);

	if (irq & AUTH_START) {
		anx7808->hdcp_tries = 0;
		anx7808_start_hdcp(anx7808);
	}

	if (irq & AUTH_DONE)
		DRM_DEBUG("ANX7808 upstream HDCP authentication ended\n");

	if (irq & HDCP_ERR)
		DRM_ERROR("ANX7808 upstream HDCP authentication error\n");
}

static void anx7808_handle_hdmi_int_5(struct anx7808_data *anx7808, uint8_t irq)
{
	int ret;

	ret = anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS5_REG, irq);
	if (ret)
		DRM_ERROR("Write hdmi int 5 failed %d\n", ret);

	if (irq & AUDIO_RCV)
		anx7808->recv_audio = true;

	if (anx7808->recv_audio && anx7808->recv_cts)
		anx7808_update_audio(anx7808);
}

static void anx7808_handle_hdmi_int_6(struct anx7808_data *anx7808, uint8_t irq)
{
	int ret;

	ret = anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS6_REG, irq);
	if (ret)
		DRM_ERROR("Write hdmi int 6 failed %d\n", ret);

	if (irq & CTS_RCV)
		anx7808->recv_cts = true;

	if (anx7808->recv_audio && anx7808->recv_cts)
		anx7808_update_audio(anx7808);

	if (irq & NEW_AVI)
		anx7808_update_video_infoframe(anx7808);

	if (irq & NEW_AUD)
		anx7808_update_audio_infoframe(anx7808);
}

static void anx7808_attempt_downstream_retrain(struct anx7808_data *anx7808)
{
	uint8_t val;
	int ret;

	/* Check upstream link is valid before retraining */
	ret = anx7808_read_reg(anx7808, HDMI_RX_SYS_STATUS_REG, &val);
	if (ret) {
		DRM_ERROR("Failed to read HDMI_RX_SYS_STATUS_REG %d\n", ret);
		return;
	}
	if (!(val & TMDS_CLOCK_DET) || !(val & TMDS_DE_DET))
		return;

	/* Video is unmuted during enable, only retrain after enabled */
	ret = anx7808_read_reg(anx7808, HDMI_RX_HDMI_MUTE_CTRL_REG, &val);
	if (ret) {
		DRM_ERROR("Failed to read HDMI_RX_HDMI_MUTE_CTRL_REG %d\n",
				ret);
		return;
	}
	if (val & VID_MUTE)
		return;

	/* Only retrain if downstream link is not up */
	ret = anx7808_aux_dpcd_read(anx7808, DP_LANE0_1_STATUS, 1, &val);
	if (ret) {
		DRM_ERROR("Failed to read DPCD lane status %d\n", ret);
		return;
	}
	if ((val & DP_CHANNEL_EQ_BITS) == DP_CHANNEL_EQ_BITS)
		return;

	/* Only retrain if downstream is terminated (ie active input) */
	ret = anx7808_aux_dpcd_read(anx7808, DOWN_STREAM_STATUS_1, 1, &val);
	if (ret) {
		DRM_ERROR("Failed to read DPCD downstream status %d\n", ret);
		return;
	}
	if (!(val & DOWN_R_TERM_DET))
		return;

	DRM_INFO("anx7808: attempting downstream retrain\n");
	if (!anx7808_dp_link_training(anx7808))
		anx7808_config_downstream(anx7808);
}

static bool anx7808_handle_sink_specific_int(struct anx7808_data *anx7808)
{
	uint8_t irq[2];
	int ret;
	bool status, irq_event = false;

	ret = anx7808_aux_dpcd_read(anx7808, SPECIFIC_INTERRUPT_1, 2, irq);
	if (ret) {
		DRM_ERROR("Failed to read DPCD specific interrupt %d\n", ret);
		return false;
	}
	ret = anx7808_aux_dpcd_write(anx7808, SPECIFIC_INTERRUPT_1, 2, irq);
	if (ret) {
		DRM_ERROR("Failed to write DPCD specific interrupt %d\n", ret);
		return false;
	}

	if (irq[0] & DWN_STREAM_CONNECTED) {
		ret = anx7808_get_downstream_info(anx7808);
		if (ret) {
			DRM_ERROR("Failed to get downstream info: %d\n", ret);
			return false;
		}
		irq_event = true;
	}

	if (irq[0] & DWN_STREAM_DISCONNECTED)
		return true;

	if (anx7808->ds_type != DOWNSTREAM_HDMI)
		return irq_event;

	/* ANX7730 specific interrupts */
	if (irq[0] & DWN_STREAM_HDCP_DONE) {
		status = (irq[0] & DWN_STREAM_HDCP_FAIL);
		if (status) {
			DRM_ERROR("ANX7730 downstream HDCP failed\n");
			anx7808_reset_hdcp(anx7808);
		} else {
			DRM_DEBUG("ANX7730 downstream HDCP passed\n");
		}
	}

	if (irq[0] & DWN_STREAM_HDCP_LINK_INTEGRITY_FAIL) {
		if (anx7808->hdcp_desired)
			DRM_ERROR("ANX7730 downstream HDCP sync lost\n");
		anx7808_reset_hdcp(anx7808);
	}

	anx7808_attempt_downstream_retrain(anx7808);

	return irq_event;
}

static bool anx7808_handle_dpcd_int(struct anx7808_data *anx7808)
{
	uint8_t irq;
	int ret;
	uint8_t reg_value;

	ret = anx7808_aux_dpcd_read(anx7808, DP_DEVICE_SERVICE_IRQ_VECTOR, 1,
			&irq);
	if (ret) {
		DRM_ERROR("Failed to read DPCD irq %d\n", ret);
		return false;
	}
	ret = anx7808_aux_dpcd_write(anx7808, DP_DEVICE_SERVICE_IRQ_VECTOR, 1,
			&irq);
	if (ret) {
		DRM_ERROR("Failed to write DPCD irq %d\n", ret);
		return false;
	}

	if (irq & DP_CP_IRQ) {
		ret = anx7808_aux_dpcd_read(anx7808, BSTATUS, 1, &reg_value);
		if (ret) {
			DRM_ERROR("Failed to read DPCD HDCP BSTATUS\n");
		} else if (reg_value & LINK_INTEGRITY_FAILURE) {
			if (anx7808->hdcp_desired)
				DRM_ERROR("ANX7808 downstream HDCP sync lost\n");
			anx7808_reset_hdcp(anx7808);
		}
	}

	if (irq & DP_SINK_SPECIFIC_IRQ)
		return anx7808_handle_sink_specific_int(anx7808);

	return false;
}

static bool anx7808_handle_tx_int(struct anx7808_data *anx7808, uint8_t irq)
{
	int ret;

	ret = anx7808_write_reg(anx7808, SP_TX_INT_STATUS1, irq);
	if (ret) {
		DRM_ERROR("Write slim int failed %d\n", ret);
		return false;
	}

	/* Cable has been disconnected, power off and wait for cable_det */
	if (irq & POLLING_ERR) {
		anx7808_power_off(anx7808, false, true);
		return true;
	}

	if (irq & DPCD_IRQ_REQUEST)
		return anx7808_handle_dpcd_int(anx7808);

	return false;
}

static irqreturn_t anx7808_intp_threaded_handler(int unused, void *data)
{
	struct anx7808_data *anx7808 = data;
	int ret;
	uint8_t irq;
	bool irq_event = false;

	mutex_lock(&anx7808->big_lock);

	/* Make sure we are still powered on after waiting for the big lock */
	if (!anx7808->powered)
		goto out;

	ret = anx7808_read_reg(anx7808, SP_TX_INT_STATUS1, &irq);
	if (ret)
		DRM_ERROR("Failed to read TX_INT_STATUS %d\n", ret);
	else if (irq)
		irq_event = anx7808_handle_tx_int(anx7808, irq);

	/*
	 * It's possible we'll be turned off if there's a polling error, in that
	 * case, exit early so we don't incur errors reading the other interrupt
	 * statuses
	 */
	if (!anx7808->powered)
		goto out;

	ret = anx7808_read_reg(anx7808, SP_COMMON_INT_STATUS2, &irq);
	if (ret)
		DRM_ERROR("Failed to read SP_COMMON_INT_STATUS2 %d\n", ret);
	else if (irq)
		anx7808_handle_common_int_2(anx7808, irq);

	ret = anx7808_read_reg(anx7808, HDMI_RX_INT_STATUS2_REG, &irq);
	if (ret)
		DRM_ERROR("Failed to read RX_INT_STATUS2 %d\n", ret);
	else if (irq)
		anx7808_handle_hdmi_int_2(anx7808, irq);

	ret = anx7808_read_reg(anx7808, HDMI_RX_INT_STATUS6_REG, &irq);
	if (ret)
		DRM_ERROR("Failed to read RX_INT_STATUS6 %d\n", ret);
	else if (irq)
		anx7808_handle_hdmi_int_6(anx7808, irq);

	ret = anx7808_read_reg(anx7808, HDMI_RX_INT_STATUS5_REG, &irq);
	if (ret)
		DRM_ERROR("Failed to read RX_INT_STATUS5 %d\n", ret);
	else if (irq)
		anx7808_handle_hdmi_int_5(anx7808, irq);

out:
	mutex_unlock(&anx7808->big_lock);

	if (irq_event)
		drm_helper_hpd_irq_event(anx7808->dev);

	return IRQ_HANDLED;
}

static void anx7808_cable_det_work(struct work_struct *work)
{
	struct anx7808_data *anx7808;
	int ret;
	bool irq_event = false;

	anx7808 = container_of(work, struct anx7808_data, cable_det_work.work);

	mutex_lock(&anx7808->big_lock);

	if (!gpio_get_value(anx7808->cable_det_gpio)) {
		irq_event = true;
		goto out;
	}

	if (!atomic_dec_and_test(&anx7808->cable_det_oneshot))
		goto out;

	ret = anx7808_power_on(anx7808);
	if (ret) {
		DRM_ERROR("power_on failed [%d]\n", ret);
		goto out;
	}

	irq_event = true;

out:
	mutex_unlock(&anx7808->big_lock);

	if (irq_event)
		drm_helper_hpd_irq_event(anx7808->dev);
}

static irqreturn_t anx7808_cable_det_isr(int irq, void *data)
{
	struct anx7808_data *anx7808 = data;

	mod_delayed_work(system_wq, &anx7808->cable_det_work,
			msecs_to_jiffies(CABLE_DET_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static ssize_t anx7808_bw_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct anx7808_data *anx7808 = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%x\n", anx7808->dp_manual_bw);
}

static ssize_t anx7808_bw_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct anx7808_data *anx7808 = i2c_get_clientdata(to_i2c_client(dev));
	int val;
	enum dp_link_bw dp_bw;
	if (!kstrtoint(buf, 16, &val)) {
		dp_bw = (enum dp_link_bw)val;
		switch (dp_bw) {
		case BW_NULL:
		case BW_162G:
		case BW_27G:
		case BW_54G:
			anx7808->dp_manual_bw = val;
			anx7808->dp_manual_bw_changed = 1;
			break;
		default:
			break;
		}
	}
	return count;
}

static struct device_attribute anx7808_device_attrs[] = {
	__ATTR(dp_manual_bw, S_IRUSR | S_IWUSR, anx7808_bw_show,
	       anx7808_bw_store),
};

static void anx7808_free_gpios(struct anx7808_data *anx7808)
{
	gpio_free(anx7808->cable_det_gpio);
	gpio_free(anx7808->intp_gpio);
	gpio_free(anx7808->reset_gpio);
	gpio_free(anx7808->pd_gpio);
}

static void unregister_i2c_clients(struct anx7808_data *anx7808)
{
	if (anx7808->rx_p1)
		i2c_unregister_device(anx7808->rx_p1);
	if (anx7808->rx_p0)
		i2c_unregister_device(anx7808->rx_p0);
	if (anx7808->tx_p2)
		i2c_unregister_device(anx7808->tx_p2);
	if (anx7808->tx_p1)
		i2c_unregister_device(anx7808->tx_p1);
}

static uint32_t anx7808_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static int anx7808_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			       int num)
{
	int err;
	int i;
	struct i2c_msg *msg;
	struct anx7808_data *anx7808 = adap->dev.platform_data;

	for (i = 0; i < num; i++) {
		msg = &msgs[i];
		if (msg->flags & I2C_M_RD)
			err = anx7808_aux_ddc_read(anx7808, msg->addr,
						   msg->len, msg->buf);
		else
			err = anx7808_aux_ddc_write(anx7808, msg->addr,
						    msg->len, msg->buf);
		if (err)
			return i;
	}
	return num;
}

static int anx7808_init_ddc_adapter(struct anx7808_data *anx7808,
				    struct device *dev)
{
	int ret;
	struct i2c_adapter *adap;

	anx7808->i2c_algorithm.functionality = &anx7808_functionality;
	anx7808->i2c_algorithm.master_xfer = &anx7808_master_xfer;
	anx7808->ddc_adapter.algo = &anx7808->i2c_algorithm;
	adap = &anx7808->ddc_adapter;

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "anx7808_ddc");
	adap->dev.parent = dev;
	adap->dev.of_node = dev->of_node;
	adap->dev.platform_data = anx7808;

	ret = i2c_add_adapter(adap);
	if (ret)
		return ret;

	of_i2c_register_devices(adap);

	return 0;
}

static int anx7808_show_regs(struct seq_file *s, void *data)
{
	struct anx7808_data *anx7808 = s->private;

	uint8_t val;

	mutex_lock(&anx7808->big_lock);
#define DUMP_REG(reg) ({ \
	if (!anx7808_read_reg(anx7808, reg, &val)) { \
		seq_printf(s, "%05x %s: %02x\n", reg, #reg, val); \
	} else { \
		seq_printf(s, "%05x %s: could not be read\n", reg, #reg); \
	} \
})
#define DUMP_DPCD(reg) ({ \
	if (!anx7808_aux_dpcd_read(anx7808, reg, 1, &val)) { \
		seq_printf(s, "DPCD %05x %s: %02x\n", reg, #reg, val); \
	} else { \
		seq_printf(s, "DPCD %05x %s: could not be read\n", reg, #reg); \
	} \
})

	DUMP_REG(SP_TX_HDCP_STATUS);
	DUMP_REG(SP_TX_HDCP_CTRL0_REG);
	DUMP_REG(SP_TX_LINK_CHK_TIMER);
	DUMP_REG(I2C_GEN_10US_TIMER0);
	DUMP_REG(I2C_GEN_10US_TIMER1);
	DUMP_REG(SP_TX_HDCP_CTRL);
	DUMP_REG(SP_TX_DEBUG_REG1);
	DUMP_REG(SP_TX_DP_POLLING_CTRL_REG);
	DUMP_REG(SP_TX_LINK_DEBUG_REG);
	DUMP_REG(SP_TX_SINK_STATUS);
	DUMP_REG(SP_TX_MISC_CTRL_REG);
	DUMP_REG(SP_TX_EXTRA_ADDR_REG);
	DUMP_REG(SP_TX_AUX_STATUS);

	DUMP_REG(SP_POWERD_CTRL_REG);
	DUMP_REG(SP_TX_RST_CTRL_REG);
	DUMP_REG(SP_TX_RST_CTRL2_REG);
	DUMP_REG(SP_TX_VID_CTRL1_REG);
	DUMP_REG(SP_TX_VID_CTRL2_REG);
	DUMP_REG(SP_TX_VID_CTRL3_REG);
	DUMP_REG(SP_TX_VID_CTRL5_REG);
	DUMP_REG(SP_TX_VID_CTRL6_REG);
	DUMP_REG(SP_TX_ANALOG_DEBUG_REG2);
	DUMP_REG(SP_TX_PLL_FILTER_CTRL1);
	DUMP_REG(SP_TX_PLL_FILTER_CTRL5);
	DUMP_REG(SP_TX_ANALOG_CTRL);
	DUMP_REG(SP_TX_INT_STATUS1);
	DUMP_REG(SP_COMMON_INT_MASK1);
	DUMP_REG(SP_COMMON_INT_MASK2);
	DUMP_REG(SP_COMMON_INT_MASK3);
	DUMP_REG(SP_COMMON_INT_MASK4);
	DUMP_REG(SP_INT_MASK);
	DUMP_REG(SP_TX_INT_CTRL_REG);

	DUMP_REG(SP_TX_LT_CTRL_REG0);
	DUMP_REG(SP_TX_LT_CTRL_REG1);
	DUMP_REG(SP_TX_LT_CTRL_REG2);
	DUMP_REG(SP_TX_LT_CTRL_REG3);
	DUMP_REG(SP_TX_LT_CTRL_REG4);
	DUMP_REG(SP_TX_LT_CTRL_REG5);
	DUMP_REG(SP_TX_LT_CTRL_REG6);
	DUMP_REG(SP_TX_LT_CTRL_REG7);
	DUMP_REG(SP_TX_LT_CTRL_REG8);
	DUMP_REG(SP_TX_LT_CTRL_REG9);
	DUMP_REG(SP_TX_LT_CTRL_REG14);
	DUMP_REG(SP_TX_LT_CTRL_REG15);
	DUMP_REG(SP_TX_LT_CTRL_REG16);
	DUMP_REG(SP_TX_LT_CTRL_REG17);
	DUMP_REG(SP_TX_LT_CTRL_REG18);
	DUMP_REG(SP_TX_LT_CTRL_REG19);

	DUMP_REG(HDMI_RX_PORT_SEL_REG);
	DUMP_REG(HDMI_RX_SRST_REG);
	DUMP_REG(HDMI_RX_SYS_STATUS_REG);
	DUMP_REG(HDMI_RX_HDMI_STATUS_REG);
	DUMP_REG(HDMI_RX_HDMI_MUTE_CTRL_REG);
	DUMP_REG(HDMI_RX_SYS_PWDN1_REG);
	DUMP_REG(HDMI_RX_AEC_CTRL_REG);
	DUMP_REG(HDMI_RX_AEC_EN0_REG);
	DUMP_REG(HDMI_RX_AEC_EN1_REG);
	DUMP_REG(HDMI_RX_AEC_EN2_REG);
	DUMP_REG(HDMI_RX_INT_STATUS5_REG);
	DUMP_REG(HDMI_RX_INT_STATUS6_REG);
	DUMP_REG(HDMI_RX_INT_MASK1_REG);
	DUMP_REG(HDMI_RX_INT_MASK2_REG);
	DUMP_REG(HDMI_RX_INT_MASK3_REG);
	DUMP_REG(HDMI_RX_INT_MASK4_REG);
	DUMP_REG(HDMI_RX_INT_MASK5_REG);
	DUMP_REG(HDMI_RX_INT_MASK6_REG);
	DUMP_REG(HDMI_RX_INT_MASK7_REG);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG2);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG4);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG5);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG6);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG7);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG18);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG19);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG21);
	DUMP_REG(HDMI_RX_TMDS_CTRL_REG22);
	DUMP_REG(HDMI_RX_VIDEO_STATUS_REG1);

	DUMP_REG(HDMI_RX_VID_DATA_RNG_CTRL_REG);
	DUMP_REG(HDMI_RX_CEC_CTRL_REG);
	DUMP_REG(HDMI_RX_CEC_SPEED_CTRL_REG);
	DUMP_REG(HDMI_RX_CHIP_CTRL_REG);

	DUMP_DPCD(DP_DPCD_REV);
	DUMP_DPCD(DP_PWR_VOLTAGE_CAP);
	DUMP_DPCD(DP_DOWNSTREAMPORT_PRESENT);
	DUMP_DPCD(RECEIVE_PORT0_CAP_0);
	DUMP_DPCD(RECEIVE_PORT0_CAP_1);
	DUMP_DPCD(RECEIVE_PORT1_CAP_0);
	DUMP_DPCD(RECEIVE_PORT1_CAP_1);
	DUMP_DPCD(DP_LINK_BW_SET);
	DUMP_DPCD(DP_LANE_COUNT_SET);
	DUMP_DPCD(DP_SINK_COUNT);
	DUMP_DPCD(DP_LANE0_1_STATUS);
	DUMP_DPCD(DP_SINK_STATUS);
	DUMP_DPCD(DOWN_STREAM_STATUS_1);
	DUMP_DPCD(DOWN_STREAM_STATUS_2);
	DUMP_DPCD(HPD_DP_PWR_STATUS);
	DUMP_DPCD(AUX_HPD_BRIDGE_POWER_CONTROL);

#undef DUMP_REG
#undef DUMP_DPCD

	mutex_unlock(&anx7808->big_lock);

	return 0;
}

static int regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, anx7808_show_regs, inode->i_private);
}

static const struct file_operations anx7808_regs_operations = {
	.open		= regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int anx7808_debugfs_init(struct anx7808_data *anx7808,
		struct drm_minor *minor)
{
	int err;

	anx7808->debugfs = debugfs_create_dir("anx7808", minor->debugfs_root);
	if (!anx7808->debugfs)
		return -ENOMEM;

	if (!debugfs_create_file("regs", S_IRUSR, anx7808->debugfs,
				anx7808, &anx7808_regs_operations)) {
		err = -ENOMEM;
		goto remove;
	}

	return 0;

remove:
	debugfs_remove_recursive(anx7808->debugfs);
	anx7808->debugfs = NULL;

	return err;
}

static void anx7808_debugfs_destroy(struct anx7808_data *anx7808)
{
	debugfs_remove_recursive(anx7808->debugfs);
	anx7808->debugfs = NULL;
}

void anx7808_disable(struct drm_bridge *bridge)
{
	struct anx7808_data *anx7808 = bridge->driver_private;
	struct drm_mode_config *mode_config = &bridge->dev->mode_config;
	int ret;

	mutex_lock(&anx7808->big_lock);

	anx7808_stop_hdcp(anx7808);

	ret = drm_object_property_set_value(&anx7808->encoder->base,
			&anx7808->encoder->propvals,
			mode_config->content_protection_property,
			DRM_MODE_CONTENT_PROTECTION_OFF, NULL);
	if (ret)
		DRM_ERROR("Failed to disable encoder hdcp\n");

	mutex_unlock(&anx7808->big_lock);
}

void anx7808_post_disable(struct drm_bridge *bridge)
{
	struct anx7808_data *anx7808 = bridge->driver_private;
	struct device *dev = bridge->dev->dev;
	bool entering_suspend = dev->power.power_state.event & PM_EVENT_SLEEP;

	mutex_lock(&anx7808->big_lock);

	anx7808_power_off(anx7808, true, !entering_suspend);

	mutex_unlock(&anx7808->big_lock);
}

void anx7808_pre_enable(struct drm_bridge *bridge)
{
	struct anx7808_data *anx7808 = bridge->driver_private;
	int ret;

	mutex_lock(&anx7808->big_lock);

	ret = anx7808_power_on(anx7808);
	if (ret) {
		DRM_ERROR("power_on failed [%d]\n", ret);
		return;
	}

	anx7808_set_hpd(anx7808, 1);

	mutex_unlock(&anx7808->big_lock);
}

static void anx7808_hdcp_en_work(struct work_struct *work)
{
	struct anx7808_data *anx7808;
	int ret;

	anx7808 = container_of(work, struct anx7808_data, hdcp_en_work.work);

	mutex_lock(&anx7808->dev->mode_config.mutex);
	ret = drm_object_property_set_value(&anx7808->encoder->base,
			&anx7808->encoder->propvals,
			anx7808->dev->mode_config.content_protection_property,
			DRM_MODE_CONTENT_PROTECTION_DESIRED, NULL);
	mutex_unlock(&anx7808->dev->mode_config.mutex);
	if (ret)
		DRM_ERROR("Failed to re-enable HDCP\n");
}

void anx7808_enable(struct drm_bridge *bridge)
{
	struct anx7808_data *anx7808 = bridge->driver_private;
	int ret;

	mutex_lock(&anx7808->big_lock);

	if (!anx7808->powered)
		goto out;

	ret = wait_for(!anx7808_detect_hdmi_input(anx7808),
			HDMI_DETECT_TIMEOUT_MS, HDMI_DETECT_INTERVAL_MS);
	if (ret) {
		DRM_ERROR("Timed out waiting for hdmi input\n");
		goto err;
	}

	ret = wait_for(!anx7808_dp_link_training(anx7808),
			LINK_TRAIN_TIMEOUT_MS, LINK_TRAIN_INTERVAL_MS);
	if (ret) {
		DRM_ERROR("Timed out trying to initiate link train\n");
		goto err;
	}

	anx7808_config_downstream(anx7808);

	goto out;

err:
	/*
	 * Reset oneshot in case an unplug while locked skipped handling
	 * POLLING_ERR
	 */
	anx7808_power_off(anx7808, true, true);

out:
	/*
	 * If we re-enable hdcp too soon, the bridge chip will lose its mind,
	 * so we'll kick it off in HDCP_EN_WAIT_MS milliseconds.
	 */
	if (anx7808->hdcp_desired)
		mod_delayed_work(system_wq, &anx7808->hdcp_en_work,
				msecs_to_jiffies(HDCP_EN_WAIT_MS));

	mutex_unlock(&anx7808->big_lock);
}

void anx7808_bridge_destroy(struct drm_bridge *bridge)
{
	int i;
	struct anx7808_data *anx7808 = bridge->driver_private;

	if (IS_ENABLED(CONFIG_DEBUG_FS))
		anx7808_debugfs_destroy(anx7808);

	drm_bridge_cleanup(bridge);
	cancel_delayed_work(&anx7808->hdcp_en_work);
	cancel_delayed_work_sync(&anx7808->cable_det_work);

	for (i = 0; i < ARRAY_SIZE(anx7808_device_attrs); i++)
		device_remove_file(bridge->dev->dev, &anx7808_device_attrs[i]);

	unregister_i2c_clients(anx7808);
	anx7808_free_gpios(anx7808);
	regulator_put(anx7808->vdd_mydp);
}


struct drm_bridge_funcs anx7808_bridge_funcs = {
	.disable = anx7808_disable,
	.post_disable = anx7808_post_disable,
	.pre_enable = anx7808_pre_enable,
	.enable = anx7808_enable,
	.destroy = anx7808_bridge_destroy,
};

int anx7808_get_modes(struct drm_connector *connector)
{
	struct anx7808_data *anx7808;
	u8 *edid;
	int ret, num_modes;
	bool power_cycle;
	uint8_t edidofs = 0;

	anx7808 = container_of(connector, struct anx7808_data, connector);

	mutex_lock(&anx7808->big_lock);

	power_cycle = !anx7808->powered;

	if (power_cycle) {
		ret = anx7808_power_on(anx7808);
		if (ret) {
			DRM_ERROR("Failed to power on anx7808, ret=%d\n", ret);
			num_modes = 0;
			goto out;
		}
	}

	ret = anx7808_get_downstream_info(anx7808);
	if (ret) {
		DRM_ERROR("Failed to get downstream info, ret=%d\n", ret);
		num_modes = 0;
		goto out;
	}

	if (anx7808->ds_type == DOWNSTREAM_DISCONNECTED) {
		DRM_ERROR("Downstream disconnected, no modes\n");
		num_modes = 0;
		goto out;
	}

	edid = (u8 *)drm_get_edid(connector, &anx7808->ddc_adapter);
	if (!edid) {
		DRM_ERROR("Failed to read edid\n");
		num_modes = 0;
		goto out;
	}

	/* Command 4 used for reading EDID is more reliable but puts ANX in
	 * a state where enabling HDCP fails. Write using command 0 to DDC to
	 * fix the state of the chip after reading EDID.
	 */
	anx7808_aux_write(anx7808, DDC_ADDR, 0, 1, &edidofs);

	drm_mode_connector_update_edid_property(connector, (struct edid *)edid);

	num_modes = drm_add_edid_modes(connector, (struct edid *)edid);

	kfree(edid);
out:
	if (power_cycle) {
		ret = anx7808_power_off(anx7808, true, false);
		if (ret)
			DRM_ERROR("Failed to power off anx7808, ret=%d\n", ret);
	}

	mutex_unlock(&anx7808->big_lock);

	return num_modes;
}

int anx7808_mode_valid(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	struct anx7808_data *anx7808 = container_of(connector,
				struct anx7808_data, connector);
	struct drm_encoder *encoder = anx7808->encoder;
	struct drm_encoder_helper_funcs *encoder_funcs =
				encoder->helper_private;

	if (encoder_funcs && encoder_funcs->mode_valid)
		return encoder_funcs->mode_valid(encoder, connector, mode);

	return MODE_OK;
}

struct drm_encoder *anx7808_best_encoder(struct drm_connector *connector)
{
	struct anx7808_data *anx7808;

	anx7808 = container_of(connector, struct anx7808_data, connector);

	return anx7808->encoder;
}

struct drm_connector_helper_funcs anx7808_connector_helper_funcs = {
	.get_modes = anx7808_get_modes,
	.mode_valid = anx7808_mode_valid,
	.best_encoder = anx7808_best_encoder,
};

enum drm_connector_status anx7808_detect(struct drm_connector *connector,
		bool force)
{
	struct anx7808_data *anx7808;
	int ret;
	bool power_cycle = false;
	enum drm_connector_status status;

	anx7808 = container_of(connector, struct anx7808_data, connector);

	mutex_lock(&anx7808->big_lock);

	if (!gpio_get_value(anx7808->cable_det_gpio)) {
		status = connector_status_disconnected;
		goto out;
	}

	power_cycle = !anx7808->powered;

	if (power_cycle) {
		ret = anx7808_power_on(anx7808);
		if (ret) {
			DRM_ERROR("Failed to power on anx7808, ret=%d\n", ret);
			status = connector_status_disconnected;
			goto out;
		}
	}

	ret = anx7808_get_downstream_info(anx7808);
	if (ret) {
		DRM_ERROR("Failed to get downstream info, ret=%d\n", ret);
		status = connector_status_disconnected;
		goto out;
	}

	status = anx7808->ds_type == DOWNSTREAM_DISCONNECTED ?
			connector_status_disconnected :
			connector_status_connected;
out:
	if (power_cycle) {
		ret = anx7808_power_off(anx7808, true, false);
		if (ret)
			DRM_ERROR("Failed to power off anx7808, ret=%d\n", ret);
	}

	mutex_unlock(&anx7808->big_lock);

	return status;
}

int anx7808_set_property(struct drm_connector *connector,
		struct drm_atomic_state *a_state,
		struct drm_property *property, uint64_t val, void *blob_data)
{
	struct anx7808_data *anx7808;
	struct drm_mode_config *mode_config = &connector->dev->mode_config;
	int ret = 0;

	anx7808 = container_of(connector, struct anx7808_data, connector);

	WARN_ON(!mutex_is_locked(&mode_config->mutex));

	if (property != mode_config->content_protection_property)
		return 0;

	if (val == DRM_MODE_CONTENT_PROTECTION_ENABLED)
		return -EINVAL;

	if (val == DRM_MODE_CONTENT_PROTECTION_DESIRED && anx7808->hdcp_desired)
		return -EALREADY;

	anx7808->hdcp_desired = val;

	ret |= anx7808->encoder->funcs->set_property(anx7808->encoder,
			a_state, property, val, blob_data);

	/*
	 * During HDCP enable:
	 * Enable upstream HDCP, then wait for HDMI_RX_INT_STATUS2_REG
	 * (AUTH_START) interrupt which notifies the anx7808 driver
	 * that upstream HDCP authentication has started, then
	 * anx7808_start_hdcp() enables HDCP in anx7808.
	 *
	 * During HDCP disable:
	 * HDCP is disabled in the upstream HDMI driver then here
	 */
	if (!val) {
		mutex_lock(&anx7808->big_lock);
		ret |= anx7808_stop_hdcp(anx7808);
		mutex_unlock(&anx7808->big_lock);
	}

	return ret;
}

void anx7808_connector_destroy(struct drm_connector *connector)
{
}

struct drm_connector_funcs anx7808_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = anx7808_detect,
	.set_property = anx7808_set_property,
	.destroy = anx7808_connector_destroy,
};

int anx7808_init(struct drm_encoder *encoder)
{
	struct drm_bridge *bridge;
	struct device_node *node;
	struct i2c_client *client;
	struct drm_device *dev = encoder->dev;
	struct anx7808_data *anx7808;
	int ret, i;

	node = of_find_compatible_node(NULL, NULL, "analogix,anx7808");
	if (!node)
		return -ENODEV;

	client = of_find_i2c_device_by_node(node);
	of_node_put(node);
	if (!client)
		return -ENODEV;

	bridge = devm_kzalloc(dev->dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge) {
		DRM_ERROR("Failed to allocate drm bridge\n");
		ret = -ENOMEM;
		goto err_client;
	}

	anx7808 = devm_kzalloc(dev->dev, sizeof(struct anx7808_data),
			       GFP_KERNEL);
	if (!anx7808) {
		DRM_ERROR("Failed to allocate platform_data.\n");
		ret = -ENOMEM;
		goto err_client;
	}

	anx7808->dev = dev;
	anx7808->encoder = encoder;

	atomic_set(&anx7808->cable_det_oneshot, 1);
	INIT_DELAYED_WORK(&anx7808->cable_det_work, anx7808_cable_det_work);
	INIT_DELAYED_WORK(&anx7808->hdcp_en_work, anx7808_hdcp_en_work);

	anx7808->vdd_mydp = regulator_get(dev->dev, "vdd_mydp");
	if (IS_ERR(anx7808->vdd_mydp)) {
		DRM_ERROR("Failed to find regulator vdd_mydp.\n");
		ret = PTR_ERR(anx7808->vdd_mydp);
		goto err_work;
	}

	anx7808->pd_gpio = of_get_named_gpio(node, "pd-gpio", 0);
	if (!gpio_is_valid(anx7808->pd_gpio)) {
		DRM_ERROR("Failed to locate pd-gpio.\n");
		ret = anx7808->pd_gpio;
		goto err_reg;
	}

	anx7808->reset_gpio = of_get_named_gpio(node, "reset-gpio", 0);
	if (!gpio_is_valid(anx7808->reset_gpio)) {
		DRM_ERROR("Failed to locate reset-gpio.\n");
		ret = anx7808->reset_gpio;
		goto err_reg;
	}

	anx7808->intp_gpio = of_get_named_gpio(node, "intp-gpio", 0);
	if (!gpio_is_valid(anx7808->intp_gpio)) {
		DRM_ERROR("Failed to locate intp-gpio.\n");
		ret = anx7808->intp_gpio;
		goto err_reg;
	}

	anx7808->cable_det_gpio = of_get_named_gpio(node, "cable-det-gpio", 0);
	if (!gpio_is_valid(anx7808->cable_det_gpio)) {
		DRM_ERROR("Failed to locate cable-det-gpio.\n");
		ret = anx7808->cable_det_gpio;
		goto err_reg;
	}

	ret = gpio_request_one(anx7808->pd_gpio, GPIOF_OUT_INIT_HIGH,
			       "anx7808_pd_gpio");
	if (ret) {
		DRM_ERROR("Failed to request pd_gpio.\n");
		goto err_gpio;
	}

	ret = gpio_request_one(anx7808->reset_gpio, GPIOF_OUT_INIT_LOW,
			       "anx7808_reset_gpio");
	if (ret) {
		DRM_ERROR("Failed to request reset_gpio.\n");
		goto err_gpio;
	}

	ret = gpio_request_one(anx7808->intp_gpio, GPIOF_DIR_IN,
			       "anx7808_intp_gpio");
	if (ret) {
		DRM_ERROR("Failed to request intp_gpio.\n");
		goto err_gpio;
	}

	ret = gpio_request_one(anx7808->cable_det_gpio, GPIOF_DIR_IN,
			       "anx7808_cable_det_gpio");
	if (ret) {
		DRM_ERROR("Failed to request cable_det_gpio.\n");
		goto err_gpio;
	}

	anx7808->tx_p0 = client;
	if (!((anx7808->tx_p0->addr) == TX_P0 >> 1)) {
		DRM_ERROR("I2C client address %02x != expected value %02x.\n",
			  anx7808->tx_p0->addr, TX_P0 >> 1);
		goto err_i2c;
	}
	anx7808->tx_p1 = i2c_new_dummy(client->adapter, TX_P1 >> 1);
	if (!anx7808->tx_p1) {
		DRM_ERROR("Failed to reserve i2c bus %02x.\n", TX_P1 >> 1);
		ret = -EINVAL;
		goto err_i2c;
	}
	anx7808->tx_p2 = i2c_new_dummy(client->adapter, TX_P2 >> 1);
	if (!anx7808->tx_p2) {
		DRM_ERROR("Failed to reserve i2c bus %02x.\n", TX_P2 >> 1);
		ret = -EINVAL;
		goto err_i2c;
	}

	anx7808->rx_p0 = i2c_new_dummy(client->adapter, RX_P0 >> 1);
	if (!anx7808->rx_p0) {
		DRM_ERROR("Failed to reserve i2c bus %02x.\n", RX_P0 >> 1);
		ret = -EINVAL;
		goto err_i2c;
	}
	anx7808->rx_p1 = i2c_new_dummy(client->adapter, RX_P1 >> 1);
	if (!anx7808->rx_p1) {
		DRM_ERROR("Failed to reserve i2c bus %02x.\n", RX_P1 >> 1);
		ret = -EINVAL;
		goto err_i2c;
	}

	anx7808->cable_det_irq = gpio_to_irq(anx7808->cable_det_gpio);
	if (anx7808->cable_det_irq < 0) {
		DRM_ERROR("Failed to get irq: %d\n", anx7808->cable_det_irq);
		goto err_i2c;
	}

	ret = devm_request_irq(dev->dev,
			       anx7808->cable_det_irq,
			       anx7808_cable_det_isr,
			       IRQF_TRIGGER_RISING, "anx7808_cable_det",
			       anx7808);
	if (ret < 0) {
		DRM_ERROR("Failed to request irq: %d\n", ret);
		goto err_i2c;
	}

#ifdef CONFIG_S5P_GPIO_INT
	ret = s5p_register_gpio_interrupt(anx7808->intp_gpio);
	if (ret < 0) {
		DRM_ERROR("cannot register/get GPIO irq\n");
		goto err_i2c;
	}
	s3c_gpio_cfgpin(anx7808->intp_gpio, S3C_GPIO_SFN(0xf));
#endif

	anx7808->intp_irq = gpio_to_irq(anx7808->intp_gpio);
	if (anx7808->intp_irq < 0) {
		DRM_ERROR("Failed to get intp irq %d %d\n", anx7808->intp_gpio,
			anx7808->intp_irq);
		goto err_i2c;
	}

	ret = devm_request_threaded_irq(&client->dev, anx7808->intp_irq, NULL,
			anx7808_intp_threaded_handler,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			"anx7808_intp", anx7808);
	if (ret < 0) {
		DRM_ERROR("Failed to request intp irq: %d\n", ret);
		goto err_i2c;
	}

	for (i = 0; i < ARRAY_SIZE(anx7808_device_attrs); i++) {
		ret = device_create_file(
			dev->dev, &anx7808_device_attrs[i]);
		if (ret) {
			DRM_ERROR("anx7808 sysfs register failed.\n");
			goto err_sysfs;
		}
	}

	mutex_init(&anx7808->big_lock);

	if (anx7808_init_ddc_adapter(anx7808, dev->dev))
		goto err_sysfs;

	bridge->driver_private = anx7808;
	encoder->bridge = bridge;

	ret = drm_bridge_init(dev, bridge, &anx7808_bridge_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		goto err_sysfs;
	}

	ret = drm_connector_init(dev, &anx7808->connector,
			&anx7808_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		goto err_bridge;
	}
	drm_connector_helper_add(&anx7808->connector,
			&anx7808_connector_helper_funcs);

	drm_object_attach_property(&anx7808->connector.base,
			dev->mode_config.content_protection_property,
			DRM_MODE_CONTENT_PROTECTION_OFF);

	drm_sysfs_connector_add(&anx7808->connector);
	drm_mode_connector_attach_encoder(&anx7808->connector, encoder);
	anx7808->connector.interlace_allowed = true;
	anx7808->connector.polled = DRM_CONNECTOR_POLL_HPD;

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		ret = anx7808_debugfs_init(anx7808, dev->primary);
		if (ret)
			DRM_ERROR("Failed to initialize debugfs\n");
	}

	return 0;

err_bridge:
	drm_bridge_cleanup(bridge);
err_sysfs:
	for (i = 0; i < ARRAY_SIZE(anx7808_device_attrs); i++)
		device_remove_file(dev->dev, &anx7808_device_attrs[i]);
err_i2c:
	unregister_i2c_clients(anx7808);
err_gpio:
	anx7808_free_gpios(anx7808);
err_reg:
	regulator_put(anx7808->vdd_mydp);
err_work:
	cancel_delayed_work_sync(&anx7808->hdcp_en_work);
	cancel_delayed_work_sync(&anx7808->cable_det_work);

err_client:
	put_device(&client->dev);
	DRM_ERROR("Failed to initialize anx7808 ret=%d\n", ret);
	return ret;
}
