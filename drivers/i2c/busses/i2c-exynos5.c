/**
 * i2c-exynos5.c - Samsung Exynos5 I2C Controller Driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_i2c.h>

/*
 * HSI2C controller from Samsung supports 2 modes of operation
 * 1. Auto mode: Where in master automatically controls the whole transaction
 * 2. Manual mode: Software controls the transaction by issuing commands
 *    START, READ, WRITE, STOP, RESTART in I2C_MANUAL_CMD register.
 *
 * Operation mode can be selected by setting AUTO_MODE bit in I2C_CONF register
 *
 * Special bits are available for both modes of operation to set commands
 * and for checking transfer status
 */

/* Register Map */
#define HSI2C_CTL		0x00
#define HSI2C_FIFO_CTL		0x04
#define HSI2C_TRAILIG_CTL	0x08
#define HSI2C_CLK_CTL		0x0C
#define HSI2C_CLK_SLOT		0x10
#define HSI2C_INT_ENABLE	0x20
#define HSI2C_INT_STATUS	0x24
#define HSI2C_ERR_STATUS	0x2C
#define HSI2C_FIFO_STATUS	0x30
#define HSI2C_TX_DATA		0x34
#define HSI2C_RX_DATA		0x38
#define HSI2C_CONF		0x40
#define HSI2C_AUTO_CONF		0x44
#define HSI2C_TIMEOUT		0x48
#define HSI2C_MANUAL_CMD	0x4C
#define HSI2C_TRANS_STATUS	0x50
#define HSI2C_TIMING_HS1	0x54
#define HSI2C_TIMING_HS2	0x58
#define HSI2C_TIMING_HS3	0x5C
#define HSI2C_TIMING_FS1	0x60
#define HSI2C_TIMING_FS2	0x64
#define HSI2C_TIMING_FS3	0x68
#define HSI2C_TIMING_SLA	0x6C
#define HSI2C_ADDR		0x70

/* I2C_CTL Register bits */
#define HSI2C_FUNC_MODE_I2C			(1u << 0)
#define HSI2C_MASTER				(1u << 3)
#define HSI2C_RXCHON				(1u << 6)
#define HSI2C_TXCHON				(1u << 7)
#define HSI2C_SW_RST				(1u << 31)

/* I2C_FIFO_CTL Register bits */
#define HSI2C_RXFIFO_EN				(1u << 0)
#define HSI2C_TXFIFO_EN				(1u << 1)
#define HSI2C_FIFO_MAX				(0x40)
#define HSI2C_RXFIFO_TRIGGER_LEVEL(x)		((x) << 4)
#define HSI2C_TXFIFO_TRIGGER_LEVEL(x)		((x) << 16)
/* I2C_TRAILING_CTL Register bits */
#define HSI2C_TRAILING_COUNT			(0xf)

/* I2C_INT_EN Register bits */
#define HSI2C_INT_TX_ALMOSTEMPTY_EN		(1u << 0)
#define HSI2C_INT_RX_ALMOSTFULL_EN		(1u << 1)
#define HSI2C_INT_TRAILING_EN			(1u << 6)
#define HSI2C_INT_I2C_EN			(1u << 9)

/* I2C_INT_STAT Register bits */
#define HSI2C_INT_TX_ALMOSTEMPTY		(1u << 0)
#define HSI2C_INT_RX_ALMOSTFULL			(1u << 1)
#define HSI2C_INT_TX_UNDERRUN			(1u << 2)
#define HSI2C_INT_TX_OVERRUN			(1u << 3)
#define HSI2C_INT_RX_UNDERRUN			(1u << 4)
#define HSI2C_INT_RX_OVERRUN			(1u << 5)
#define HSI2C_INT_TRAILING			(1u << 6)
#define HSI2C_INT_I2C				(1u << 9)
#define HSI2C_RX_INT				(HSI2C_INT_RX_ALMOSTFULL | \
						 HSI2C_INT_RX_UNDERRUN | \
						 HSI2C_INT_RX_OVERRUN | \
						 HSI2C_INT_TRAILING)

/* I2C_FIFO_STAT Register bits */
#define HSI2C_RX_FIFO_EMPTY			(1u << 24)
#define HSI2C_RX_FIFO_FULL			(1u << 23)
#define HSI2C_RX_FIFO_LVL(x)			((x >> 16) & 0x7f)
#define HSI2C_TX_FIFO_EMPTY			(1u << 8)
#define HSI2C_TX_FIFO_FULL			(1u << 7)
#define HSI2C_TX_FIFO_LVL(x)			((x >> 0) & 0x7f)
#define HSI2C_FIFO_EMPTY			(HSI2C_RX_FIFO_EMPTY |	\
						HSI2C_TX_FIFO_EMPTY)

/* I2C_CONF Register bits */
#define HSI2C_AUTO_MODE				(1u << 31)
#define HSI2C_10BIT_ADDR_MODE			(1u << 30)
#define HSI2C_HS_MODE				(1u << 29)

/* I2C_AUTO_CONF Register bits */
#define HSI2C_READ_WRITE			(1u << 16)
#define HSI2C_STOP_AFTER_TRANS			(1u << 17)
#define HSI2C_MASTER_RUN			(1u << 31)

/* I2C_TIMEOUT Register bits */
#define HSI2C_TIMEOUT_EN			(1u << 31)

/* I2C_TRANS_STATUS register bits */
#define HSI2C_MASTER_BUSY			(1u << 17)
#define HSI2C_SLAVE_BUSY			(1u << 16)
#define HSI2C_TIMEOUT_AUTO			(1u << 4)
#define HSI2C_NO_DEV				(1u << 3)
#define HSI2C_NO_DEV_ACK			(1u << 2)
#define HSI2C_TRANS_ABORT			(1u << 1)
#define HSI2C_TRANS_DONE			(1u << 0)

/* I2C_ADDR register bits */
#define HSI2C_SLV_ADDR_SLV(x)			((x & 0x3ff) << 0)
#define HSI2C_SLV_ADDR_MAS(x)			((x & 0x3ff) << 10)
#define HSI2C_MASTER_ID(x)			((x & 0xff) << 24)
#define MASTER_ID(x)				((x & 0x7) + 0x08)

/*
 * Controller operating frequency, timing values for operation
 * are calculated against this frequency
 */
#define HSI2C_HS_TX_CLOCK	1000000
#define HSI2C_FS_TX_CLOCK	100000
#define HSI2C_HIGH_SPD		1
#define HSI2C_FAST_SPD		0

#define EXYNOS5_I2C_TIMEOUT (msecs_to_jiffies(1000))

/* timeout for pm runtime autosuspend */
#define EXYNOS5_I2C_PM_TIMEOUT		1000	/* ms */

struct exynos5_i2c {
	struct i2c_adapter	adap;
	unsigned int		suspended:1;

	struct i2c_msg		*msg;
	struct completion	msg_complete;
	unsigned int		msg_ptr;
	unsigned int		msg_len;

	unsigned int		irq;

	void __iomem		*regs;
	struct clk		*clk;
	struct device		*dev;
	int			state;

	/*
	 * Since the TRANS_DONE bit is cleared on read, and we may read it
	 * either during an IRQ or after a transaction, keep track of its
	 * state here.
	 */
	int			trans_done;

	/* Controller operating frequency */
	unsigned int		fs_clock;
	unsigned int		hs_clock;

	/*
	 * HSI2C Controller can operate in
	 * 1. High speed upto 3.4Mbps
	 * 2. Fast speed upto 1Mbps
	 */
	int			speed_mode;
	int			bus_id;
};

static const struct of_device_id exynos5_i2c_match[] = {
	{ .compatible = "samsung,exynos5-hsi2c" },
	{},
};
MODULE_DEVICE_TABLE(of, exynos5_i2c_match);

static void exynos5_i2c_clr_pend_irq(struct exynos5_i2c *i2c)
{
	writel(readl(i2c->regs + HSI2C_INT_STATUS),
				i2c->regs + HSI2C_INT_STATUS);
}

/*
 * exynos5_i2c_set_timing: updates the registers with appropriate
 * timing values calculated
 *
 * Returns 0 on success, -EINVAL if the cycle length cannot
 * be calculated.
 */
static int exynos5_i2c_set_timing(struct exynos5_i2c *i2c, int mode)
{
	u32 i2c_timing_s1;
	u32 i2c_timing_s2;
	u32 i2c_timing_s3;
	u32 i2c_timing_sla;
	unsigned int t_start_su, t_start_hd;
	unsigned int t_stop_su;
	unsigned int t_data_su, t_data_hd;
	unsigned int t_scl_l, t_scl_h;
	unsigned int t_sr_release;
	unsigned int t_ftl_cycle;
	unsigned int clkin = clk_get_rate(i2c->clk);
	unsigned int div, utemp0 = 0, utemp1 = 0, clk_cycle;
	unsigned int op_clk = (mode == HSI2C_HIGH_SPD) ?
				i2c->hs_clock : i2c->fs_clock;

	/*
	 * FPCLK / FI2C =
	 * (CLK_DIV + 1) * (TSCLK_L + TSCLK_H + 2) + 8 + 2 * FLT_CYCLE
	 * utemp0 = (CLK_DIV + 1) * (TSCLK_L + TSCLK_H + 2)
	 * utemp1 = (TSCLK_L + TSCLK_H + 2)
	 */
	t_ftl_cycle = (readl(i2c->regs + HSI2C_CONF) >> 16) & 0x7;
	utemp0 = (clkin / op_clk) - 8 - 2 * t_ftl_cycle;

	/* CLK_DIV max is 256 */
	for (div = 0; div < 256; div++) {
		utemp1 = utemp0 / (div + 1);

		/*
		 * SCL_L and SCL_H each has max value of 255
		 * Hence, For the clk_cycle to the have right value
		 * utemp1 has to be less then 512 and more than 4.
		 */
		if ((utemp1 < 512) && (utemp1 > 4)) {
			clk_cycle = utemp1 - 2;
			break;
		} else if (div == 255) {
			dev_warn(i2c->dev, "Failed to calculate divisor");
			return -EINVAL;
		}
	}

	t_scl_l = clk_cycle / 2;
	t_scl_h = clk_cycle / 2;
	t_start_su = t_scl_l;
	t_start_hd = t_scl_l;
	t_stop_su = t_scl_l;
	t_data_su = t_scl_l / 2;
	t_data_hd = t_scl_l / 2;
	t_sr_release = clk_cycle;

	i2c_timing_s1 = t_start_su << 24 | t_start_hd << 16 | t_stop_su << 8;
	i2c_timing_s2 = t_data_su << 24 | t_scl_l << 8 | t_scl_h << 0;
	i2c_timing_s3 = div << 16 | t_sr_release << 0;
	i2c_timing_sla = t_data_hd << 0;

	dev_dbg(i2c->dev, "tSTART_SU: %X, tSTART_HD: %X, tSTOP_SU: %X\n",
		t_start_su, t_start_hd, t_stop_su);
	dev_dbg(i2c->dev, "tDATA_SU: %X, tSCL_L: %X, tSCL_H: %X\n",
		t_data_su, t_scl_l, t_scl_h);
	dev_dbg(i2c->dev, "nClkDiv: %X, tSR_RELEASE: %X\n",
		div, t_sr_release);
	dev_dbg(i2c->dev, "tDATA_HD: %X\n", t_data_hd);

	if (mode == HSI2C_HIGH_SPD) {
		writel(i2c_timing_s1, i2c->regs + HSI2C_TIMING_HS1);
		writel(i2c_timing_s2, i2c->regs + HSI2C_TIMING_HS2);
		writel(i2c_timing_s3, i2c->regs + HSI2C_TIMING_HS3);
	} else {
		writel(i2c_timing_s1, i2c->regs + HSI2C_TIMING_FS1);
		writel(i2c_timing_s2, i2c->regs + HSI2C_TIMING_FS2);
		writel(i2c_timing_s3, i2c->regs + HSI2C_TIMING_FS3);
	}
	writel(i2c_timing_sla, i2c->regs + HSI2C_TIMING_SLA);

	return 0;
}

static int exynos5_hsi2c_clock_setup(struct exynos5_i2c *i2c)
{
	/*
	 * Configure the Fast speed timing values
	 * Even the High Speed mode initially starts with Fast mode
	 */
	if (exynos5_i2c_set_timing(i2c, HSI2C_FAST_SPD)) {
		dev_err(i2c->dev, "HSI2C FS Clock set up failed\n");
		return -EINVAL;
	}

	/* configure the High speed timing values */
	if (i2c->speed_mode == HSI2C_HIGH_SPD) {
		if (exynos5_i2c_set_timing(i2c, HSI2C_HIGH_SPD)) {
			dev_err(i2c->dev, "HSI2C HS Clock set up failed\n");
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * exynos5_i2c_init: configures the controller for I2C functionality
 * Programs I2C controller for Master mode operation
 */
static void exynos5_i2c_init(struct exynos5_i2c *i2c)
{
	u32 i2c_conf = readl(i2c->regs + HSI2C_CONF);

	writel((HSI2C_FUNC_MODE_I2C | HSI2C_MASTER),
					i2c->regs + HSI2C_CTL);
	writel(HSI2C_TRAILING_COUNT, i2c->regs + HSI2C_TRAILIG_CTL);

	if (i2c->speed_mode == HSI2C_HIGH_SPD) {
		writel(HSI2C_MASTER_ID(MASTER_ID(i2c->bus_id)),
					i2c->regs + HSI2C_ADDR);
		i2c_conf |= HSI2C_HS_MODE;
	}

	writel(i2c_conf | HSI2C_AUTO_MODE, i2c->regs + HSI2C_CONF);
}

static void exynos5_i2c_reset(struct exynos5_i2c *i2c)
{
	u32 i2c_ctl;

	/* Set and clear the bit for reset */
	i2c_ctl = readl(i2c->regs + HSI2C_CTL);
	i2c_ctl |= HSI2C_SW_RST;
	writel(i2c_ctl, i2c->regs + HSI2C_CTL);

	i2c_ctl = readl(i2c->regs + HSI2C_CTL);
	i2c_ctl &= ~HSI2C_SW_RST;
	writel(i2c_ctl, i2c->regs + HSI2C_CTL);

	/* We don't expect calculations to fail during the run */
	exynos5_hsi2c_clock_setup(i2c);
	/* Initialize the configure registers */
	exynos5_i2c_init(i2c);
}

/*
 * exynos5_i2c_irq: top level IRQ servicing routine
 *
 * INT_STATUS registers gives the interrupt details. Further,
 * FIFO_STATUS or TRANS_STATUS registers are to be check for detailed
 * state of the bus.
 */
static irqreturn_t exynos5_i2c_irq(int irqno, void *dev_id)
{
	struct exynos5_i2c *i2c = dev_id;
	u32 fifo_level, int_status, fifo_status, trans_status;
	unsigned char byte;
	int len = 0;

	i2c->state = -EINVAL;

	int_status = readl(i2c->regs + HSI2C_INT_STATUS);
	fifo_status = readl(i2c->regs + HSI2C_FIFO_STATUS);

	if (int_status & HSI2C_INT_I2C) {
		trans_status = readl(i2c->regs + HSI2C_TRANS_STATUS);
		if (trans_status & HSI2C_NO_DEV_ACK) {
			dev_dbg(i2c->dev, "No ACK from device\n");
			i2c->state = -ENXIO;
		} else if (trans_status & HSI2C_NO_DEV) {
			dev_dbg(i2c->dev, "No device\n");
			i2c->state = -ENXIO;
		} else if (trans_status & HSI2C_TRANS_ABORT) {
			dev_dbg(i2c->dev, "Deal with arbitration lose\n");
			i2c->state = -EAGAIN;
		} else if (trans_status & HSI2C_TIMEOUT_AUTO) {
			dev_dbg(i2c->dev, "Accessing device timed out\n");
			i2c->state = -EAGAIN;
		} else if (trans_status & HSI2C_TRANS_DONE) {
			i2c->trans_done = 1;
			i2c->state = 0;
		}
	}
	/* TX_ALMOSTEMPTY can happen along with HSI2C_INT_I2C */
	else if (int_status &
			(HSI2C_INT_TX_UNDERRUN | HSI2C_INT_TX_ALMOSTEMPTY)) {
		fifo_level = HSI2C_TX_FIFO_LVL(fifo_status);

		/* To support probing the devices for detection */
		if (i2c->msg->len == 0) {
			i2c->state = -ENXIO;
			goto stop;
		}

		len = HSI2C_FIFO_MAX - fifo_level;
		if (len > i2c->msg->len)
			len = i2c->msg->len;

		i2c->msg_len += len;
		while (len > 0) {
			byte = i2c->msg->buf[i2c->msg_ptr++];
			writel(byte, i2c->regs + HSI2C_TX_DATA);
			len--;
		}
		i2c->state = 0;
		goto stop;
	}
	/* If TX FIFO is full (give chance to clear) */
	else if (int_status & HSI2C_INT_TX_OVERRUN)
		i2c->state = 0;

	if (int_status & (HSI2C_INT_RX_OVERRUN | HSI2C_INT_TRAILING |
		HSI2C_INT_RX_UNDERRUN | HSI2C_INT_RX_ALMOSTFULL)) {
		fifo_level = HSI2C_RX_FIFO_LVL(fifo_status);

		if (fifo_level >= i2c->msg->len)
			len = i2c->msg->len;
		else
			len = fifo_level;

		i2c->msg_len += len;
		while (len > 0) {
			byte = (unsigned char)
				readl(i2c->regs + HSI2C_RX_DATA);
			i2c->msg->buf[i2c->msg_ptr++] = byte;
			len--;
		}
		i2c->state = 0;
	}


 stop:
	if ((i2c->msg_len == i2c->msg->len) || (i2c->state < 0)) {
		writel(0, i2c->regs + HSI2C_INT_ENABLE);
		complete(&i2c->msg_complete);
	}

	exynos5_i2c_clr_pend_irq(i2c);

	return IRQ_HANDLED;
}

/*
 * exynos5_i2c_wait_bus_idle
 *
 * Wait for the transaction to complete (indicated by the TRANS_DONE bit
 * being set), and, if this is the last message in a transfer, wait for the
 * MASTER_BUSY bit to be cleared.
 *
 * Returns -EBUSY if the bus cannot be bought to idle
 */
static int exynos5_i2c_wait_bus_idle(struct exynos5_i2c *i2c, int stop)
{
	unsigned long stop_time;
	u32 trans_status;

	/* wait for 100 milli seconds for the bus to be idle */
	stop_time = jiffies + msecs_to_jiffies(100) + 1;
	do {
		trans_status = readl(i2c->regs + HSI2C_TRANS_STATUS);
		if (trans_status & HSI2C_TRANS_DONE)
			i2c->trans_done = 1;
		/*
		 * Only wait for MASTER_BUSY to be cleared if this is the last
		 * message.
		 */
		if ((!stop || !(trans_status & HSI2C_MASTER_BUSY)) &&
		    i2c->trans_done)
			return 0;

		usleep_range(50, 200);
	} while (time_before(jiffies, stop_time));

	return -EBUSY;
}

/*
 * exynos5_i2c_message_start: Configures the bus and starts the xfer
 * i2c: struct exynos5_i2c pointer for the current bus
 * stop: Enables stop after transfer if set. Set for last transfer of
 *       in the list of messages.
 *
 * Configures the bus for read/write function
 * Sets chip address to talk to, message length to be sent.
 * Enables appropriate interrupts and sends start xfer command.
 */
static void exynos5_i2c_message_start(struct exynos5_i2c *i2c, int stop)
{
	u32 i2c_ctl;
	u32 int_en = HSI2C_INT_I2C_EN;
	u32 i2c_auto_conf = 0;
	u32 fifo_ctl;
	u32 i2c_timeout;

	/*
	 * When the message length is > FIFO depth, set the FIFO trigger
	 * at FIFO_MAX - 4. Just for ease of handling.
	 */
	unsigned short len = (i2c->msg->len > HSI2C_FIFO_MAX) ?
					(HSI2C_FIFO_MAX - 4) : i2c->msg->len;

	/* Clear to enable Timeout */
	i2c_timeout = readl(i2c->regs + HSI2C_TIMEOUT);
	i2c_timeout &= ~HSI2C_TIMEOUT_EN;
	writel(i2c_timeout, i2c->regs + HSI2C_TIMEOUT);

	fifo_ctl = HSI2C_RXFIFO_EN | HSI2C_TXFIFO_EN;
	writel(fifo_ctl, i2c->regs + HSI2C_FIFO_CTL);

	i2c_ctl = readl(i2c->regs + HSI2C_CTL);
	i2c_ctl &= ~(HSI2C_TXCHON | HSI2C_RXCHON);
	if (i2c->msg->flags & I2C_M_RD) {
		i2c_ctl |= HSI2C_RXCHON;

		i2c_auto_conf |= HSI2C_READ_WRITE;

		fifo_ctl |= HSI2C_RXFIFO_TRIGGER_LEVEL(len);
		int_en |= (HSI2C_INT_RX_ALMOSTFULL_EN |
			HSI2C_INT_TRAILING_EN);
	} else {
		i2c_ctl |= HSI2C_TXCHON;

		fifo_ctl |= HSI2C_TXFIFO_TRIGGER_LEVEL(len);
		int_en |= HSI2C_INT_TX_ALMOSTEMPTY_EN;
	}

	if (stop == 1)
		i2c_auto_conf |= HSI2C_STOP_AFTER_TRANS;

	writel(HSI2C_SLV_ADDR_MAS(i2c->msg->addr), i2c->regs + HSI2C_ADDR);

	writel(fifo_ctl, i2c->regs + HSI2C_FIFO_CTL);
	writel(i2c_ctl, i2c->regs + HSI2C_CTL);

	/* In auto mode the length of xfer cannot be 0 */
	if (i2c->msg->len == 0)
		i2c_auto_conf |= 0x1;
	else
		i2c_auto_conf |= i2c->msg->len;

	writel(i2c_auto_conf, i2c->regs + HSI2C_AUTO_CONF);

	/* Start data transfer in Master mode */
	i2c_auto_conf = readl(i2c->regs + HSI2C_AUTO_CONF);
	i2c_auto_conf |= HSI2C_MASTER_RUN;
	writel(i2c_auto_conf, i2c->regs + HSI2C_AUTO_CONF);

	writel(int_en, i2c->regs + HSI2C_INT_ENABLE);
}

static int exynos5_i2c_xfer_msg(struct exynos5_i2c *i2c,
			      struct i2c_msg *msgs, int stop)
{
	unsigned long timeout;
	int ret;

	i2c->msg = msgs;
	i2c->msg_ptr = 0;
	i2c->msg_len = 0;
	i2c->trans_done = 0;

	INIT_COMPLETION(i2c->msg_complete);

	exynos5_i2c_message_start(i2c, stop);

	timeout = wait_for_completion_timeout(&i2c->msg_complete,
					      EXYNOS5_I2C_TIMEOUT);
	if (timeout == 0)
		ret = -ETIMEDOUT;
	else
		ret = i2c->state;

	if (ret < 0) {
		exynos5_i2c_reset(i2c);
		if (ret == -ETIMEDOUT) {
			dev_warn(i2c->dev, "%s timeout\n",
				 (msgs->flags & I2C_M_RD) ? "rx" : "tx");
			return ret;
		} else if (ret == -EAGAIN) {
			return ret;
		}
	}

	/*
	 * If this is the last message to be transfered (stop == 1)
	 * Then check if the bus can be brought back to idle.
	 *
	 * Return -EBUSY if the bus still busy.
	 */
	if (exynos5_i2c_wait_bus_idle(i2c, stop))
		return -EBUSY;

	/* Return the state as in interrupt routine */
	return ret;
}

static int exynos5_i2c_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msgs, int num)
{
	struct exynos5_i2c *i2c = (struct exynos5_i2c *)adap->algo_data;
	struct i2c_msg *msgs_ptr = msgs;
	int retry, i = 0;
	int ret = 0, ret_pm;
	int stop = 0;

	if (i2c->suspended) {
		dev_err(i2c->dev, "HS-I2C is not initialzed.\n");
		return -EIO;
	}

	ret_pm = pm_runtime_get_sync(i2c->dev);
	if (IS_ERR_VALUE(ret_pm)) {
		ret = -EIO;
		goto out;
	}

	clk_prepare_enable(i2c->clk);

	for (retry = 0; retry < adap->retries; retry++) {
		for (i = 0; i < num; i++) {
			stop = (i == num - 1);

			ret = exynos5_i2c_xfer_msg(i2c, msgs_ptr, stop);
			msgs_ptr++;

			if (ret == -EAGAIN) {
				msgs_ptr = msgs;
				break;
			} else if (ret < 0) {
				goto out;
			}
		}

		if ((i == num) && (ret != -EAGAIN))
			break;

		dev_dbg(i2c->dev, "retrying transfer (%d)\n", retry);

		udelay(100);
	}

	if (i == num) {
		ret = num;
	} else {
		/* Only one message, cannot access the device */
		if (i == 1)
			ret = -EREMOTEIO;
		else
			ret = i;

		dev_warn(i2c->dev, "xfer message failed\n");
	}

 out:
	clk_disable_unprepare(i2c->clk);
	pm_runtime_mark_last_busy(i2c->dev);
	pm_runtime_put_autosuspend(i2c->dev);
	return ret;
}

static u32 exynos5_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm exynos5_i2c_algorithm = {
	.master_xfer		= exynos5_i2c_xfer,
	.functionality		= exynos5_i2c_func,
};

static int exynos5_i2c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct exynos5_i2c *i2c;
	struct resource *mem;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "no device node\n");
		return -ENOENT;
	}

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct exynos5_i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	/* Mode of operation High/Fast Speed mode */
	if (of_get_property(np, "samsung,hs-mode", NULL)) {
		i2c->speed_mode = HSI2C_HIGH_SPD;
		i2c->fs_clock = HSI2C_FS_TX_CLOCK;
		if (of_property_read_u32(np, "clock-frequency", &i2c->hs_clock))
			i2c->hs_clock = HSI2C_HS_TX_CLOCK;
	} else {
		i2c->speed_mode = HSI2C_FAST_SPD;
		if (of_property_read_u32(np, "clock-frequency", &i2c->fs_clock))
			i2c->fs_clock = HSI2C_FS_TX_CLOCK;
	}

	strlcpy(i2c->adap.name, "exynos5-i2c", sizeof(i2c->adap.name));
	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &exynos5_i2c_algorithm;
	i2c->adap.retries = 2;
	i2c->adap.class   = I2C_CLASS_HWMON | I2C_CLASS_SPD;

	i2c->dev = &pdev->dev;
	i2c->clk = devm_clk_get(&pdev->dev, "hsi2c");
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return -ENOENT;
	}

	clk_prepare_enable(i2c->clk);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2c->regs = devm_request_and_ioremap(&pdev->dev, mem);
	if (IS_ERR(i2c->regs)) {
		dev_err(&pdev->dev, "cannot map HS-I2C IO\n");
		ret = PTR_ERR(i2c->regs);
		goto err_clk;
	}

	i2c->adap.dev.of_node = np;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &pdev->dev;

	/* Clear pending interrupts from u-boot or misc causes */
	exynos5_i2c_clr_pend_irq(i2c);

	init_completion(&i2c->msg_complete);

	i2c->irq = ret = irq_of_parse_and_map(np, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "cannot find HS-I2C IRQ\n");
		ret = -EINVAL;
		goto err_clk;
	}

	ret = devm_request_irq(&pdev->dev, i2c->irq, exynos5_i2c_irq,
				0, dev_name(&pdev->dev), i2c);

	if (ret != 0) {
		dev_err(&pdev->dev, "cannot request HS-I2C IRQ %d\n", i2c->irq);
		goto err_clk;
	}

	/*
	 * TODO: Use private lock to avoid race conditions as
	 * mentioned in pm_runtime.txt
	 */
	pm_runtime_enable(i2c->dev);
	pm_runtime_set_autosuspend_delay(i2c->dev, EXYNOS5_I2C_PM_TIMEOUT);
	pm_runtime_use_autosuspend(i2c->dev);

	ret = pm_runtime_get_sync(i2c->dev);
	if (IS_ERR_VALUE(ret))
		goto err_clk;

	ret = exynos5_hsi2c_clock_setup(i2c);
	if (ret)
		goto err_pm;

	i2c->bus_id = of_alias_get_id(i2c->adap.dev.of_node, "hsi2c");

	exynos5_i2c_init(i2c);

	i2c->adap.nr = -1;
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_pm;
	}

	of_i2c_register_devices(&i2c->adap);
	platform_set_drvdata(pdev, i2c);

	clk_disable_unprepare(i2c->clk);
	pm_runtime_mark_last_busy(i2c->dev);
	pm_runtime_put_autosuspend(i2c->dev);

	return 0;

 err_pm:
	pm_runtime_put(i2c->dev);
	pm_runtime_disable(&pdev->dev);
 err_clk:
	clk_disable_unprepare(i2c->clk);
	return ret;
}

static int exynos5_i2c_remove(struct platform_device *pdev)
{
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);
	int ret;

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret))
		return ret;

	i2c_del_adapter(&i2c->adap);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	clk_disable_unprepare(i2c->clk);

	return 0;
}

#ifdef CONFIG_PM
static int exynos5_i2c_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);

	i2c->suspended = 1;

	return 0;
}

static int exynos5_i2c_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos5_i2c *i2c = platform_get_drvdata(pdev);
	int ret = 0;

	clk_prepare_enable(i2c->clk);

	ret = exynos5_hsi2c_clock_setup(i2c);
	if (ret) {
		clk_disable_unprepare(i2c->clk);
		return ret;
	}

	exynos5_i2c_init(i2c);
	clk_disable_unprepare(i2c->clk);
	i2c->suspended = 0;

	return 0;
}

static const struct dev_pm_ops exynos5_i2c_dev_pm_ops = {
	.suspend_noirq = exynos5_i2c_suspend_noirq,
	.resume_noirq	= exynos5_i2c_resume_noirq,
};

#define EXYNOS5_DEV_PM_OPS (&exynos5_i2c_dev_pm_ops)
#else
#define EXYNOS5_DEV_PM_OPS NULL
#endif

static struct platform_driver exynos5_i2c_driver = {
	.probe		= exynos5_i2c_probe,
	.remove		= exynos5_i2c_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "exynos5-hsi2c",
		.pm	= EXYNOS5_DEV_PM_OPS,
		.of_match_table = exynos5_i2c_match,
	},
};

static int __init i2c_adap_exynos5_init(void)
{
	return platform_driver_register(&exynos5_i2c_driver);
}
subsys_initcall(i2c_adap_exynos5_init);

static void __exit i2c_adap_exynos5_exit(void)
{
	platform_driver_unregister(&exynos5_i2c_driver);
}
module_exit(i2c_adap_exynos5_exit);

MODULE_DESCRIPTION("Exynos5 HS-I2C Bus driver");
MODULE_AUTHOR("Naveen Krishna Chatradhi, <ch.naveen@samsung.com>");
MODULE_AUTHOR("Taekgyun Ko, <taeggyun.ko@samsung.com>");
MODULE_LICENSE("GPL v2");
