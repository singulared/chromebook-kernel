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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include "drmP.h"
#include "anx7808regs.h"

#define ANX7808_DEVICE_ID 0x7808
#define AUX_WAIT_MS 100
#define AUX_BUFFER_SIZE 0x10
#define CABLE_DET_TIME_MS 1000

enum anx7808_state {
	STATE_CABLE_NOT_DETECTED,
	STATE_CABLE_DETECTED,
	STATE_HPD_DETECTED,
	STATE_HDMI_OK,
	STATE_DP_OK,
	STATE_PLAY,
};

enum dp_link_bw {
	BW_OVER = 0xFF,
	BW_54G = 0x14,
	BW_27G = 0x0A,
	BW_162G = 0x06,
	BW_NULL = 0x00
};

struct anx7808_data {
	int pd_gpio;
	int reset_gpio;
	int intp_gpio;
	int cable_det_gpio;
	int cable_det_irq;
	struct timer_list cable_det_timer;
	struct regulator *vdd_mydp;
	struct i2c_client *tx_p0;
	struct i2c_client *tx_p1;
	struct i2c_client *tx_p2;
	struct i2c_client *rx_p0;
	struct i2c_client *rx_p1;
	struct delayed_work play_video;
	struct workqueue_struct *wq;
	enum dp_link_bw dp_supported_bw;
	enum dp_link_bw dp_manual_bw;
	int dp_manual_bw_changed;
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

static int anx7808_power_on(struct anx7808_data *anx7808)
{
	int ret = 0;

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
	gpio_set_value(anx7808->reset_gpio, 1);
	return 0;
}

static int anx7808_power_off(struct anx7808_data *anx7808)
{
	int ret = 0;

	DRM_INFO("Powering off ANX7808.\n");

	gpio_set_value(anx7808->reset_gpio, 0);
	usleep_range(1000, 2000);
	ret = regulator_disable(anx7808->vdd_mydp);
	if (ret < 0) {
		DRM_ERROR("Failed to power off ANX7808: %d", ret);
		return ret;
	}
	usleep_range(5000, 10000);
	gpio_set_value(anx7808->pd_gpio, 1);
	usleep_range(1000, 2000);
	return ret;
}

static int anx7808_chip_located(struct anx7808_data *anx7808)
{
	int ret = 0;
	uint16_t id;
	uint8_t idh = 0, idl = 0;
	ret |= anx7808_read_reg(anx7808, SP_TX_DEV_IDL_REG, &idl);
	ret |= anx7808_read_reg(anx7808, SP_TX_DEV_IDH_REG, &idh);
	if (ret)
		return ret;
	id = idl | (idh << 8);
	if (id != ANX7808_DEVICE_ID) {
		DRM_ERROR("ANX7808 not found.  ID reg contains: %04x\n", id);
		return -ENODEV;
	}
	DRM_INFO("ANX7808 found.\n");
	return 0;
}

static int anx7808_vbus_power_on(struct anx7808_data *anx7808)
{
	uint8_t status;

	anx7808_set_bits(anx7808, SP_TX_PLL_FILTER_CTRL6,
			 P5V_PROTECT_PD | SHORT_PROTECT_PD);
	anx7808_clear_bits(anx7808, SP_TX_PLL_FILTER_CTRL11, V33_SWITCH_ON);
	anx7808_set_bits(anx7808, SP_TX_PLL_FILTER_CTRL11, V33_SWITCH_ON);

	anx7808_read_reg(anx7808, SP_TX_PLL_FILTER_CTRL6, &status);
	if (status & (P5V_PROTECT | SHORT_PROTECT)) {
		DRM_ERROR("Failed to enable VBUS: 0x%02x.\n", status);
		return -EIO;
	}
	DRM_DEBUG_KMS("Enabled VBUS.\n");
	return 0;
}

static int anx7808_aux_wait(struct anx7808_data *anx7808)
{
	int err;
	uint8_t status;
	unsigned long start = jiffies;

	while ((jiffies - start) <= msecs_to_jiffies(AUX_WAIT_MS)) {
		err = anx7808_read_reg(anx7808, SP_TX_AUX_STATUS, &status);
		if (err)
			return err;
		if (!(status & AUX_BUSY))
			break;
		usleep_range(100, 200);
	}

	if (status) {
		DRM_ERROR("Failed to read AUX channel: 0x%02x\n", status);
		return -EIO;
	}
	return 0;
}

static int anx7808_aux_read(struct anx7808_data *anx7808, uint32_t addr,
			    uint8_t cmd, uint8_t count, uint8_t *pBuf)
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
	err |= anx7808_set_bits(anx7808, SP_TX_AUX_CTRL_REG2, AUX_OP_EN);
	if (err)
		return -EIO;
	usleep_range(2000, 4000);

	err = anx7808_aux_wait(anx7808);
	if (err)
		return err;

	for (i = 0; i < count; i++)
		err |= anx7808_read_reg(anx7808, SP_TX_BUF_DATA_0_REG + i,
					pBuf + i);
	if (err)
		return -EIO;

	return 0;
}

static int anx7808_aux_dpcd_read(struct anx7808_data *anx7808, uint32_t addr,
				 uint8_t count, uint8_t *pBuf)
{
	return anx7808_aux_read(anx7808, addr, AUX_DPCD, count, pBuf);
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
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK2_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK3_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK4_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK5_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK6_REG, 0x00);
	anx7808_write_reg(anx7808, HDMI_RX_INT_MASK7_REG, 0x00);

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
	anx7808_set_bits(anx7808, SP_TX_PLL_FILTER_CTRL11, AUX_TERM_50OHM);
	anx7808_clear_bits(anx7808, SP_TX_PLL_FILTER_CTRL6,
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
	anx7808_write_reg(anx7808, SP_INT_MASK, 0x90);
	anx7808_write_reg(anx7808, SP_TX_INT_CTRL_REG, 0x01);

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

	anx7808_vbus_power_on(anx7808);
	msleep(20);
}

static int anx7808_detect_dp_hotplug(struct anx7808_data *anx7808)
{
	uint8_t status;

	if (anx7808_aux_dpcd_read(anx7808, DOWN_STREAM_STATUS_1, 1, &status))
		return -EAGAIN;
	if (!(status & DOWN_STRM_HPD)) {
		DRM_INFO("Waiting for downstream HPD.\n");
		return -EAGAIN;
	}
	return 0;
}

static int anx7808_detect_hdmi_input(struct anx7808_data *anx7808)
{
	uint8_t sys_status;

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

	return 0;
}

static int anx7808_dp_link_training(struct anx7808_data *anx7808)
{
	uint8_t dp_bw, status;
	int err;

	err = anx7808_aux_dpcd_read(anx7808, MAX_LINK_RATE, 1, &dp_bw);
	if (err)
		return err;
	anx7808->dp_supported_bw = (enum dp_link_bw)dp_bw;
	switch (anx7808->dp_supported_bw) {
	case BW_162G:
	case BW_27G:
	case BW_54G:
		break;
	case BW_NULL:
	case BW_OVER:
	default:
		DRM_INFO("Waiting to read DP bandwidth.");
		return -EAGAIN;
	}

	anx7808_clear_bits(anx7808, SP_POWERD_CTRL_REG, VIDEO_PD);
	anx7808_set_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_EN | VIDEO_MUTE);

	msleep(20);

	anx7808_read_reg(anx7808, SP_TX_SYS_CTRL2_REG, &status);
	anx7808_write_reg(anx7808, SP_TX_SYS_CTRL2_REG, status);
	anx7808_read_reg(anx7808, SP_TX_SYS_CTRL2_REG, &status);
	if (status & CHA_STA) {
		DRM_INFO("Waiting for DP clock: 0x%02x\n", status);
		return -EAGAIN;
	}

	anx7808_set_bits(anx7808, SP_TX_SYS_CTRL3_REG, STRM_VALID);
	anx7808_read_reg(anx7808, SP_TX_SYS_CTRL3_REG, &status);
	if (!(status & STRM_VALID)) {
		DRM_INFO("Waiting for DP signal: 0x%02x\n", status);
		return -EAGAIN;
	}

	anx7808_clear_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_EN);

	anx7808_set_bits(anx7808, SP_TX_ANALOG_PD_REG, CH0_PD);
	usleep_range(1000, 2000);
	anx7808_clear_bits(anx7808, SP_TX_ANALOG_PD_REG, CH0_PD);

	anx7808_set_bits(anx7808, SP_TX_PLL_CTRL_REG, PLL_RST);
	usleep_range(1000, 2000);
	anx7808_clear_bits(anx7808, SP_TX_PLL_CTRL_REG, PLL_RST);

	if (anx7808->dp_manual_bw == 0)
		dp_bw = (uint8_t)anx7808->dp_supported_bw;
	else
		dp_bw = (uint8_t)anx7808->dp_manual_bw;
	anx7808_write_reg(anx7808, SP_TX_LINK_BW_SET_REG, dp_bw);
	anx7808_write_reg(anx7808, SP_TX_LT_CTRL_REG, SP_TX_LT_EN);

	anx7808_read_reg(anx7808, SP_TX_LT_CTRL_REG, &status);
	if (status & 0x70) {
		DRM_INFO("Waiting for DP link training: 0x%02x\n", status);
		return -EAGAIN;
	}

	anx7808_clear_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_MUTE);

	return 0;
}

static void anx7808_config_dp_output(struct anx7808_data *anx7808)
{
	anx7808_set_bits(anx7808, SP_TX_VID_CTRL1_REG, VIDEO_EN);
}

static void anx7808_update_infoframes(struct anx7808_data *anx7808)
{
	uint8_t status6;

	anx7808_read_reg(anx7808, HDMI_RX_INT_STATUS6_REG, &status6);

	if (status6 & NEW_AVI) {
		anx7808_clear_bits(anx7808, SP_TX_PKT_EN_REG, AVI_IF_EN);
		anx7808_write_reg(anx7808, SP_TX_AVI_TYPE, SP_TX_AVI_KTYPE);
		anx7808_write_reg(anx7808, SP_TX_AVI_VER, SP_TX_AVI_KVER);
		anx7808_write_reg(anx7808, SP_TX_AVI_LEN, SP_TX_AVI_KLEN);
		anx7808_copy_regs(anx7808, HDMI_RX_AVI_DATA00_REG,
				  SP_TX_AVI_DB0, SP_TX_AVI_KLEN);
		anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AVI_IF_UD);
		anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AVI_IF_EN);
		anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS6_REG, NEW_AVI);
	}

	if (status6 & NEW_AUD) {
		anx7808_clear_bits(anx7808, SP_TX_PKT_EN_REG, AUD_IF_EN);
		anx7808_write_reg(anx7808, SP_TX_AUD_TYPE, SP_TX_AUD_KTYPE);
		anx7808_write_reg(anx7808, SP_TX_AUD_VER, SP_TX_AUD_KVER);
		anx7808_write_reg(anx7808, SP_TX_AUD_LEN, SP_TX_AUD_KLEN);
		anx7808_copy_regs(anx7808, HDMI_RX_AUDIO_DATA00_REG,
				  SP_TX_AUD_DB0, SP_TX_AUD_KLEN);
		anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AUD_IF_UP);
		anx7808_set_bits(anx7808, SP_TX_PKT_EN_REG, AUD_IF_EN);
		anx7808_write_reg(anx7808, HDMI_RX_INT_STATUS6_REG, NEW_AUD);
	}
}

static int anx7808_check_polling_err(struct anx7808_data *anx7808)
{
	uint8_t tx_int_status;
	anx7808_read_reg(anx7808, SP_TX_INT_STATUS1, &tx_int_status);
	anx7808_write_reg(anx7808, SP_TX_INT_STATUS1, tx_int_status);
	if (tx_int_status & POLLING_ERR) {
		DRM_INFO("Polling error: %02x", tx_int_status);
		return -EFAULT;
	}
	return 0;
}

static void anx7808_play_video(struct work_struct *work)
{
	struct delayed_work *dw =
		container_of(work, struct delayed_work, work);
	struct anx7808_data *anx7808 =
		container_of(dw, struct anx7808_data, play_video);
	enum anx7808_state state = STATE_CABLE_DETECTED;

	if (anx7808_power_on(anx7808))
		return;

	anx7808_clear_bits(anx7808, SP_POWERD_CTRL_REG, REGISTER_PD);
	anx7808_clear_bits(anx7808, SP_POWERD_CTRL_REG, TOTAL_PD);
	anx7808_init_pipeline(anx7808);

	anx7808->dp_manual_bw_changed = 0;

	while (true) {
		/* Make progress towards playback state. */
		if (anx7808->dp_manual_bw_changed)
			break;

		switch (state) {
		case STATE_CABLE_DETECTED:
			if (anx7808_detect_dp_hotplug(anx7808))
				break;
			anx7808_set_hpd(anx7808, 1);
			state = STATE_HPD_DETECTED;

		case STATE_HPD_DETECTED:
			if (anx7808_detect_hdmi_input(anx7808))
				break;
			state = STATE_HDMI_OK;

		case STATE_HDMI_OK:
			if (anx7808_dp_link_training(anx7808))
				break;
			state = STATE_DP_OK;

		case STATE_DP_OK:
			anx7808_config_dp_output(anx7808);
			state = STATE_PLAY;
		case STATE_PLAY:
			anx7808_update_infoframes(anx7808);
		default:
			break;
		}

		/* Check for failures */
		if (anx7808_check_polling_err(anx7808)) {
			DRM_INFO("Cable connection lost.\n");
			break;
		}

		if (state > STATE_CABLE_DETECTED) {
			if (anx7808_detect_dp_hotplug(anx7808)) {
				DRM_INFO("Hotplug detection lost.\n");
				anx7808_set_hpd(anx7808, 0);
				state = STATE_CABLE_DETECTED;
			}
		}

		msleep(300);
	}

	anx7808_power_off(anx7808);
}

static void anx7808_cable_det_timer(unsigned long data)
{
	struct anx7808_data *anx7808 = (struct anx7808_data *)data;
	DRM_INFO("anx7808_cable_det_timer");
	queue_delayed_work(anx7808->wq, &anx7808->play_video, 0);
}

static irqreturn_t anx7808_cable_det_isr(int irq, void *data)
{
	struct anx7808_data *anx7808 = data;
	if (gpio_get_value(anx7808->cable_det_gpio)) {
		DRM_INFO("Cable detected; Setting cable_det_timer.\n");
		mod_timer(&anx7808->cable_det_timer,
			  jiffies + msecs_to_jiffies(CABLE_DET_TIME_MS));
	} else {
		DRM_INFO("Cable unplugged; Deleting cable_det_timer.\n");
		del_timer(&anx7808->cable_det_timer);
	}
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

static int anx7808_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int i, ret = 0;
	struct device_node *node = client->dev.of_node;
	struct anx7808_data *anx7808;

	anx7808 = devm_kzalloc(&client->dev, sizeof(struct anx7808_data),
			       GFP_KERNEL);
	if (!anx7808) {
		DRM_ERROR("Failed to allocate platform_data.\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, anx7808);

	INIT_DELAYED_WORK(&anx7808->play_video, anx7808_play_video);

	setup_timer(&anx7808->cable_det_timer, anx7808_cable_det_timer,
		    (unsigned long)anx7808);

	anx7808->vdd_mydp = regulator_get(&client->dev, "vdd_mydp");
	if (IS_ERR(anx7808->vdd_mydp)) {
		DRM_ERROR("Failed to find regulator vdd_mydp.\n");
		return PTR_ERR(anx7808->vdd_mydp);
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

	anx7808_power_on(anx7808);
	ret = anx7808_chip_located(anx7808);
	anx7808_power_off(anx7808);
	if (ret)
		goto err_i2c;

	anx7808->cable_det_irq = gpio_to_irq(anx7808->cable_det_gpio);
	if (anx7808->cable_det_irq < 0) {
		DRM_ERROR("Failed to get irq: %d\n", anx7808->cable_det_irq);
		goto err_i2c;
	}

	anx7808->wq = create_singlethread_workqueue("anx7808_work");
	if (anx7808->wq == NULL) {
		DRM_ERROR("Failed to create work queue.\n");
		ret = -ENOMEM;
		goto err_i2c;
	}

	ret = devm_request_irq(&client->dev,
			       anx7808->cable_det_irq,
			       anx7808_cable_det_isr,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "anx7808_cable_det",
			       anx7808);
	if (ret < 0) {
		DRM_ERROR("Failed to request irq: %d\n", ret);
		goto err_wq;
	}

	for (i = 0; i < ARRAY_SIZE(anx7808_device_attrs); i++) {
		ret = device_create_file(
			&client->dev, &anx7808_device_attrs[i]);
		if (ret) {
			DRM_ERROR("anx7808 sysfs register failed.\n");
			goto err_sysfs;
		}
	}

	DRM_INFO("ANX7808 initialization successful.\n");

	return 0;
err_sysfs:
	for (i = 0; i < ARRAY_SIZE(anx7808_device_attrs); i++)
		device_remove_file(&client->dev, &anx7808_device_attrs[i]);
err_wq:
	destroy_workqueue(anx7808->wq);
err_i2c:
	unregister_i2c_clients(anx7808);
err_gpio:
	anx7808_free_gpios(anx7808);
err_reg:
	regulator_put(anx7808->vdd_mydp);
	return ret;
}

static int anx7808_remove(struct i2c_client *client)
{
	int i;
	struct anx7808_data *anx7808 = i2c_get_clientdata(client);

	for (i = 0; i < ARRAY_SIZE(anx7808_device_attrs); i++)
		device_remove_file(&client->dev, &anx7808_device_attrs[i]);
	destroy_workqueue(anx7808->wq);
	unregister_i2c_clients(anx7808);
	anx7808_free_gpios(anx7808);
	regulator_put(anx7808->vdd_mydp);
	return 0;
}

static const struct i2c_device_id anx7808_id[] = {
	{ "anx7808", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, anx7808_id);

static struct i2c_driver anx7808_driver = {
	.driver = {
		.name = "anx7808",
		.owner = THIS_MODULE,
	},
	.probe = anx7808_probe,
	.remove = anx7808_remove,
	.id_table = anx7808_id,
};

static int __init anx7808_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&anx7808_driver);
	if (ret < 0)
		DRM_ERROR("Failed to register anx7808 i2c driver.\n");
	return ret;
}

static void __exit anx7808_exit(void)
{
	i2c_del_driver(&anx7808_driver);
}

module_init(anx7808_init);
module_exit(anx7808_exit);

MODULE_DESCRIPTION("ANX7808 driver");
MODULE_AUTHOR("Jeremy Thorpe <jeremyt@chromium.org>");
MODULE_LICENSE("GPL");
