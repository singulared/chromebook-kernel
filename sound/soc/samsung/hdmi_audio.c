/* sound/soc/samsung/hdmi_audio.c
 *
 * ALSA SoC Audio Layer - Samsung Hdmi Audio Plugin
 *
 * Copyright (c) 2012 Samsung Electronics Co. Ltd.
 *	Rahul Sharma <rahul.sharma@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <plat/gpio-cfg.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>

#include "codec_plugin.h"
#include "hdmi_audio.h"

#define DRV_NAME "exynos-hdmi-audio"
#define HDMI_POWERON_WAIT_COUNT (60)

struct hdmi_audio_params {
	u32 sample_rate;
	u32 cts;
	u32 n;
	u32 sample_frq;
};

struct hdmi_audio_params audio_params[] = {
		{ 32000, 27000, 4096, 0x3 },
		{ 44100, 30000, 6272, 0x0 },
		{ 88200, 30000, 12544, 0x8 },
		{ 176400, 30000, 25088, 0xc },
		{ 48000, 27000, 6144, 0x2 },
		{ 96000, 27000, 12288, 0xa },
		{ 192000, 27000, 24576, 0xe },
};

static inline u32 hdmi_reg_read(struct hdmi_audio_context *ctx, u32 reg_id)
{
	return readl(ctx->regs + reg_id);
}

static inline void hdmi_reg_writeb(struct hdmi_audio_context *ctx,
				 u32 reg_id, u8 value)
{
	writeb(value, ctx->regs + reg_id);
}

static inline void hdmi_reg_writemask(struct hdmi_audio_context *ctx,
				 u32 reg_id, u32 value, u32 mask)
{
	u32 old = readl(ctx->regs + reg_id);
	value = (value & mask) | (old & ~mask);
	writel(value, ctx->regs + reg_id);
}

static int hdmi_get_audio_params(u32 freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(audio_params); ++i)
		if (freq == audio_params[i].sample_rate)
			return i;

	return -ENXIO;
}

static void hdmi_set_acr(struct hdmi_audio_context *ctx, u8 *acr)
{
	u32 n, cts;

	n = ctx->params.n;
	cts = ctx->params.cts;

	acr[1] = cts >> 16;
	acr[2] = cts >> 8 & 0xff;
	acr[3] = cts & 0xff;

	acr[4] = n >> 16;
	acr[5] = n >> 8 & 0xff;
	acr[6] = n & 0xff;
}

static void hdmi_reg_acr(struct hdmi_audio_context *ctx, u8 *acr)
{
	hdmi_reg_writeb(ctx, HDMI_ACR_N0, acr[6]);
	hdmi_reg_writeb(ctx, HDMI_ACR_N1, acr[5]);
	hdmi_reg_writeb(ctx, HDMI_ACR_N2, acr[4]);
	hdmi_reg_writeb(ctx, HDMI_ACR_MCTS0, acr[3]);
	hdmi_reg_writeb(ctx, HDMI_ACR_MCTS1, acr[2]);
	hdmi_reg_writeb(ctx, HDMI_ACR_MCTS2, acr[1]);
	hdmi_reg_writeb(ctx, HDMI_ACR_CTS0, acr[3]);
	hdmi_reg_writeb(ctx, HDMI_ACR_CTS1, acr[2]);
	hdmi_reg_writeb(ctx, HDMI_ACR_CTS2, acr[1]);
	hdmi_reg_writeb(ctx, HDMI_ACR_CON, 4);
}

static void hdmi_audio_i2s_init(struct hdmi_audio_context *ctx)
{
	u32 sample_rate, bits_per_sample, frame_size_code;
	u32 data_num, bit_ch, sample_frq;
	u32 val;
	u8 acr[7];

	sample_rate = ctx->params.sample_rate;
	bits_per_sample = ctx->params.bits_per_sample;
	frame_size_code = 0;

	switch (bits_per_sample) {
	case 16:
		data_num = 1;
		bit_ch = 0;
		break;
	case 20:
		data_num = 2;
		bit_ch  = 1;
		break;
	case 24:
		data_num = 3;
		bit_ch  = 1;
		break;
	case 32:
		data_num = 1;
		bit_ch  = 2;
		break;
	default:
		data_num = 1;
		bit_ch  = 0;
		break;
	}

	hdmi_set_acr(ctx, acr);
	hdmi_reg_acr(ctx, acr);

	hdmi_reg_writeb(ctx, HDMI_I2S_MUX_CON, HDMI_I2S_IN_DISABLE
				| HDMI_I2S_AUD_I2S | HDMI_I2S_CUV_I2S_ENABLE
				| HDMI_I2S_MUX_ENABLE);

	hdmi_reg_writeb(ctx, HDMI_I2S_MUX_CH, HDMI_I2S_CH0_EN
			| HDMI_I2S_CH1_EN | HDMI_I2S_CH2_EN);

	hdmi_reg_writeb(ctx, HDMI_I2S_MUX_CUV, HDMI_I2S_CUV_RL_EN);

	sample_frq = ctx->params.sample_frq;

	hdmi_reg_writeb(ctx, HDMI_I2S_CLK_CON, HDMI_I2S_CLK_DIS);
	hdmi_reg_writeb(ctx, HDMI_I2S_CLK_CON, HDMI_I2S_CLK_EN);

	val = hdmi_reg_read(ctx, HDMI_I2S_DSD_CON) | 0x01;
	hdmi_reg_writeb(ctx, HDMI_I2S_DSD_CON, val);

	/* Configuration I2S input ports. Configure I2S_PIN_SEL_0~4 */
	hdmi_reg_writeb(ctx, HDMI_I2S_PIN_SEL_0, HDMI_I2S_SEL_SCLK(5)
			| HDMI_I2S_SEL_LRCK(6));
	hdmi_reg_writeb(ctx, HDMI_I2S_PIN_SEL_1, HDMI_I2S_SEL_SDATA1(1)
			| HDMI_I2S_SEL_SDATA2(4));
	hdmi_reg_writeb(ctx, HDMI_I2S_PIN_SEL_2, HDMI_I2S_SEL_SDATA3(1)
			| HDMI_I2S_SEL_SDATA2(2));
	hdmi_reg_writeb(ctx, HDMI_I2S_PIN_SEL_3, HDMI_I2S_SEL_DSD(0));

	/* I2S_CON_1 & 2 */
	hdmi_reg_writeb(ctx, HDMI_I2S_CON_1, HDMI_I2S_SCLK_FALLING_EDGE
			| HDMI_I2S_L_CH_LOW_POL);
	hdmi_reg_writeb(ctx, HDMI_I2S_CON_2, HDMI_I2S_MSB_FIRST_MODE
			| HDMI_I2S_SET_BIT_CH(bit_ch)
			| HDMI_I2S_SET_SDATA_BIT(data_num)
			| HDMI_I2S_BASIC_FORMAT);

	/* Configure register related to CUV information */
	hdmi_reg_writeb(ctx, HDMI_I2S_CH_ST_0, HDMI_I2S_CH_STATUS_MODE_0
			| HDMI_I2S_2AUD_CH_WITHOUT_PREEMPH
			| HDMI_I2S_COPYRIGHT
			| HDMI_I2S_LINEAR_PCM
			| HDMI_I2S_CONSUMER_FORMAT);
	hdmi_reg_writeb(ctx, HDMI_I2S_CH_ST_1, HDMI_I2S_CD_PLAYER);
	hdmi_reg_writeb(ctx, HDMI_I2S_CH_ST_2, HDMI_I2S_SET_SOURCE_NUM(0));
	hdmi_reg_writeb(ctx, HDMI_I2S_CH_ST_3, HDMI_I2S_CLK_ACCUR_LEVEL_2
			| HDMI_I2S_SET_SMP_FREQ(sample_frq));
	hdmi_reg_writeb(ctx, HDMI_I2S_CH_ST_4,
			HDMI_I2S_ORG_SMP_FREQ_44_1
			| HDMI_I2S_WORD_LEN_MAX24_24BITS
			| HDMI_I2S_WORD_LEN_MAX_24BITS);

	hdmi_reg_writeb(ctx, HDMI_I2S_CH_ST_CON, HDMI_I2S_CH_STATUS_RELOAD);
}

static void hdmi_audio_init(struct hdmi_audio_context *ctx)
{
	hdmi_audio_i2s_init(ctx);
}

static bool hdmi_phy_stable(struct hdmi_audio_context *ctx)
{
	u32 phy_state = hdmi_reg_read(ctx, HDMI_PHY_STATUS_0);
	return phy_state & HDMI_PHY_STATUS_READY;
}

static int hdmi_audio_control(struct hdmi_audio_context *ctx,
	bool onoff)
{
	u32 mod;

	snd_printdd("[%d] %s on %d\n", __LINE__, __func__, onoff);

	mod = hdmi_reg_read(ctx, HDMI_MODE_SEL);

	/*
	* DVI mode bit carries correct mode information only when hdmi
	* phy is in stable state. Else should be ignored.
	*/
	if ((mod & HDMI_DVI_MODE_EN) && hdmi_phy_stable(ctx) && onoff) {
		snd_printdd("dvi mode on\n");
		return -EINVAL;
	}

	hdmi_reg_writeb(ctx, HDMI_AUI_CON, onoff ?
			HDMI_AUI_CON_TRANS_EVERY_VSYNC : HDMI_AUI_CON_NO_TRAN);
	hdmi_reg_writemask(ctx, HDMI_CON_0, onoff ?
			HDMI_ASP_EN : HDMI_ASP_DIS, HDMI_ASP_MASK);
	return 0;
}

static u8 hdmi_chksum(struct hdmi_audio_context *ctx,
			u32 start, u8 len, u32 hdr_sum)
{
	int i;
	/* hdr_sum : header0 + header1 + header2
	* start : start address of packet byte1
	* len : packet bytes - 1 */
	for (i = 0; i < len; ++i)
		hdr_sum += 0xff & hdmi_reg_read(ctx, start + i * 4);

	return (u8)(0x100 - (hdr_sum & 0xff));
}

static void hdmi_reg_infoframe(struct hdmi_audio_context *ctx,
			struct hdmi_infoframe *infoframe)
{
	u32 hdr_sum;
	u8 chksum;
	hdmi_reg_writeb(ctx, HDMI_AUI_HEADER0, infoframe->type);
	hdmi_reg_writeb(ctx, HDMI_AUI_HEADER1, infoframe->ver);
	hdmi_reg_writeb(ctx, HDMI_AUI_HEADER2, infoframe->len);
	hdr_sum = infoframe->type + infoframe->ver + infoframe->len;
	chksum = hdmi_chksum(ctx, HDMI_AUI_BYTE(1),
				infoframe->len, hdr_sum);
	hdmi_reg_writeb(ctx, HDMI_AUI_CHECK_SUM, chksum);
}

static void hdmi_conf_init(struct hdmi_audio_context *ctx)
{
	struct hdmi_infoframe infoframe;

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	/* enable AVI packet every vsync, fixes purple line problem */
	hdmi_reg_writemask(ctx, HDMI_CON_1, 2, 3 << 5);

	infoframe.type = HDMI_PACKET_TYPE_AUI;
	infoframe.ver = HDMI_AUI_VERSION;
	infoframe.len = HDMI_AUI_LENGTH;
	hdmi_reg_infoframe(ctx, &infoframe);
}

static int hdmi_audio_hw_params(struct device *dev,
		struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct hdmi_audio_context *ctx = NULL;
	struct audio_codec_plugin *plugin;
	int ret = 0, index;

	if (!dev) {
		dev_err(dev, "invalid device.\n");
		ret = -EINVAL;
		goto err;
	}

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	plugin = dev_get_drvdata(dev);
	ctx = container_of(plugin, struct hdmi_audio_context, plugin);

	switch (params_channels(params)) {
	case 6:
	case 4:
	case 2:
	case 1:
		ctx->params.chan_count = params_channels(params);
		break;
	default:
		dev_err(dev, "%d channels not supported\n",
				params_channels(params));
		ret = -EINVAL;
		goto err;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		ctx->params.bits_per_sample = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		ctx->params.bits_per_sample = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ctx->params.bits_per_sample = 24;
		break;
	default:
		dev_err(dev, "Format(%d) not supported\n",
				params_format(params));
		ret = -EINVAL;
		goto err;
	}

	index = hdmi_get_audio_params(params_rate(params));
	if (index >= 0) {
		ctx->params.sample_rate = audio_params[index].sample_rate;
		ctx->params.sample_frq = audio_params[index].sample_frq;
		ctx->params.cts = audio_params[index].cts;
		ctx->params.n = audio_params[index].n;
	} else {
		dev_err(dev, "Sampling Rate (%d) not supported\n",
				params_rate(params));
		ret = -EINVAL;
		goto err;
	}

	snd_printdd("chan cnt [%d]\n", ctx->params.chan_count);
	snd_printdd("bps [%d]\n", ctx->params.bits_per_sample);
	snd_printdd("sample_rate [%d]\n", ctx->params.sample_rate);

	/* checking here to cache audioparms for hpd plug handling */
	if (!atomic_read(&ctx->plugged))
		return -EINVAL;

	hdmi_audio_control(ctx, false);
	hdmi_conf_init(ctx);
	hdmi_audio_init(ctx);
	return ret;

err:
	hdmi_audio_control(ctx, false);
	return ret;
}

static int hdmi_audio_trigger(struct device *dev,
				struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct hdmi_audio_context *ctx = NULL;
	struct audio_codec_plugin *plugin;
	int ret = 0;

	if (!dev) {
		dev_err(dev, "invalid device.\n");
		return -EINVAL;
	}

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	plugin = dev_get_drvdata(dev);
	ctx = container_of(plugin, struct hdmi_audio_context, plugin);

	if (!atomic_read(&ctx->plugged))
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (!ctx->enabled)
			return -EINVAL;
		ret = hdmi_audio_control(ctx, true);
		if (ret) {
			dev_err(dev, "audio enable failed.\n");
			return -EINVAL;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		hdmi_audio_control(ctx, false);
		break;
	}
	return ret;
}

static int hdmi_set_state(struct device *dev, int enable)
{
	struct hdmi_audio_context *ctx = NULL;
	struct audio_codec_plugin *plugin;

	if (!dev) {
		dev_err(dev, "invalid device.\n");
		return -EINVAL;
	}

	snd_printdd("[%d] %s en %d\n", __LINE__, __func__, enable);

	plugin = dev_get_drvdata(dev);
	ctx = container_of(plugin, struct hdmi_audio_context, plugin);
	ctx->enabled = !!enable;

	if (!atomic_read(&ctx->plugged))
		return -EINVAL;

	hdmi_audio_control(ctx, !!enable);

	return 0;
}

static int hdmi_get_state(struct device *dev, int *is_enabled)
{
	struct hdmi_audio_context *ctx = NULL;
	struct audio_codec_plugin *plugin;

	if (!dev) {
		dev_err(dev, "invalid device.\n");
		return -EINVAL;
	}

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	plugin = dev_get_drvdata(dev);
	ctx = container_of(plugin, struct hdmi_audio_context, plugin);

	if (is_enabled && ctx)
		*is_enabled = ctx->enabled;
	else
		return -EINVAL;
	return 0;
}

static int hdmi_get_jack_state(struct device *dev, int *is_connected)
{
	struct hdmi_audio_context *ctx = NULL;
	struct audio_codec_plugin *plugin;

	if (!dev) {
		dev_err(dev, "invalid device.\n");
		return -EINVAL;
	}

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	plugin = dev_get_drvdata(dev);
	ctx = container_of(plugin, struct hdmi_audio_context, plugin);

	if (is_connected && ctx)
		*is_connected = atomic_read(&ctx->plugged);
	else
		return -EINVAL;
	return 0;
}

static void hdmi_audio_hotplug_func(struct work_struct *work)
{
	struct hdmi_audio_context *ctx = container_of(work,
	struct hdmi_audio_context, hotplug_work.work);
	int plugged;
	static int tries = HDMI_POWERON_WAIT_COUNT;

	snd_printdd("[%d] %s plugged %d tries %d.\n",
		__LINE__, __func__, atomic_read(&ctx->plugged), tries);

	plugged = atomic_read(&ctx->plugged);

	/* skip if unplugged */
	if (!plugged)
		goto report_event;

	/* wait till hdmiphy is stable */
	if (!hdmi_phy_stable(ctx)) {
		if (--tries) {
			cancel_delayed_work(&ctx->hotplug_work);
			queue_delayed_work(ctx->hpd_wq, &ctx->hotplug_work,
					msecs_to_jiffies(50));
		} else {
			/* hdmi is still off, reset count. */
			tries = HDMI_POWERON_WAIT_COUNT;
		}
		return;
	}

	if (hdmi_reg_read(ctx, HDMI_MODE_SEL) & HDMI_DVI_MODE_EN) {
		/* If HDMI operates in DVI mode,
		 * for audio purposes it is the same as nothing plugged.
		 * So, change the "ctx->plugged" state to unplugged.
		 * Also, simulate unplugging for jack_cb, as this
		 * takes care of swapping HDMI with DVI when suspended.
		 */
		atomic_set(&ctx->plugged, 0);
		plugged = false;
		goto report_event;
	}

	snd_printdd("[%d] %s hdmi powered on after %d tries\n",
		__LINE__, __func__, HDMI_POWERON_WAIT_COUNT - tries);

	hdmi_audio_control(ctx, false);
	hdmi_conf_init(ctx);
	hdmi_audio_init(ctx);
	if (ctx->enabled)
		hdmi_audio_control(ctx, true);

report_event:
	tries = HDMI_POWERON_WAIT_COUNT;
	if (ctx->plugin.jack_cb)
		ctx->plugin.jack_cb(plugged);
}

static irqreturn_t hdmi_audio_irq_handler(int irq, void *arg)
{
	struct hdmi_audio_context *ctx = arg;
	u32 val = gpio_get_value(ctx->hpd_gpio);

	atomic_set(&ctx->plugged, !!val);
	snd_printdd("%s %s\n", __func__,
			atomic_read(&ctx->plugged) ? "plugged" : "unplugged");

	queue_delayed_work(ctx->hpd_wq, &ctx->hotplug_work, 0);
	return IRQ_HANDLED;
}


static int hdmi_audio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct hdmi_audio_context *ctx;
	struct resource *res;
	struct device_node *parent_node;

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(&pdev->dev, "failed to alloc common hdmi audio context.\n");
		ret = -ENOMEM;
		goto err;
	}

	ctx->pdev = pdev;
	ctx->enabled = true;
	ctx->plugin.dev = &pdev->dev;
	ctx->plugin.ops.hw_params = hdmi_audio_hw_params;
	ctx->plugin.ops.trigger = hdmi_audio_trigger;
	ctx->plugin.ops.get_state = hdmi_get_state;
	ctx->plugin.ops.set_state = hdmi_set_state;
	ctx->plugin.ops.get_jack_state = hdmi_get_jack_state;
	ctx->params.sample_rate = DEFAULT_RATE;
	ctx->params.bits_per_sample = DEFAULT_BPS;

	dev_set_drvdata(&pdev->dev, &ctx->plugin);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to find registers.\n");
		ret = -ENOENT;
		goto err_mem;
	}

	ctx->regs = ioremap(res->start, resource_size(res));
	if (!ctx->regs) {
		dev_err(&pdev->dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_mem;
	}

	/* create workqueue and hotplug work */
	ctx->hpd_wq = alloc_workqueue("exynos-hdmi-audio",
			WQ_UNBOUND | WQ_NON_REENTRANT, 1);
	if (ctx->hpd_wq == NULL) {
		dev_err(&pdev->dev, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto err_unmap;
	}
	INIT_DELAYED_WORK(&ctx->hotplug_work, hdmi_audio_hotplug_func);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "get interrupt res failed.\n");
		ret = -ENXIO;
		goto err_workq;
	}

	parent_node = of_get_parent(pdev->dev.of_node);
	if (!parent_node) {
		dev_err(&pdev->dev, "no parent node found.\n");
		ret = -EINVAL;
		goto err_workq;
	}

	ctx->parent_pdev = of_find_device_by_node(parent_node);
	if (!ctx->parent_pdev) {
		dev_err(&pdev->dev, "no parent pdev found.\n");
		ret = -EINVAL;
		goto err_workq;
	}

	ctx->hpd_gpio = (int)pdev->dev.platform_data;

	if (!gpio_is_valid(ctx->hpd_gpio)) {
		dev_err(&pdev->dev, "failed to get hpd gpio.");
		ret = -EINVAL;
		goto err_workq;
	}

	atomic_set(&ctx->plugged, gpio_get_value(ctx->hpd_gpio));
	ctx->ext_irq = gpio_to_irq(ctx->hpd_gpio);

	ret = request_irq(ctx->ext_irq,
			hdmi_audio_irq_handler, IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED,
			"ext_hdmi_audio", ctx);
	if (ret) {
		dev_err(&pdev->dev, "request interrupt failed.\n");
		goto err_irq;
	}

	return ret;

err_irq:
	disable_irq(ctx->int_irq);
	free_irq(ctx->int_irq, ctx);
err_workq:
	destroy_workqueue(ctx->hpd_wq);
err_unmap:
	iounmap(ctx->regs);
err_mem:
	kfree(ctx);
err:
	return ret;
}

static int hdmi_audio_remove(struct platform_device *pdev)
{
	struct hdmi_audio_context *ctx = NULL;
	int ret = 0;

	ctx = dev_get_drvdata(&pdev->dev);

	disable_irq(ctx->int_irq);
	free_irq(ctx->int_irq, ctx);
	disable_irq(ctx->ext_irq);
	free_irq(ctx->ext_irq, ctx);
	destroy_workqueue(ctx->hpd_wq);
	iounmap(ctx->regs);
	kfree(ctx);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int hdmi_audio_suspend(struct device *dev)
{
	struct hdmi_audio_context *ctx = NULL;
	struct audio_codec_plugin *plugin;

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	plugin = dev_get_drvdata(dev);
	ctx = container_of(plugin, struct hdmi_audio_context, plugin);

	disable_irq(ctx->int_irq);
	atomic_set(&ctx->plugged, 0);
	return 0;
}

static int hdmi_audio_resume(struct device *dev)
{
	struct hdmi_audio_context *ctx = NULL;
	struct audio_codec_plugin *plugin;
	u32 val;

	snd_printdd("[%d] %s\n", __LINE__, __func__);

	plugin = dev_get_drvdata(dev);
	ctx = container_of(plugin, struct hdmi_audio_context, plugin);

	val = gpio_get_value(ctx->hpd_gpio);
	atomic_set(&ctx->plugged, !!val);

	queue_delayed_work(ctx->hpd_wq, &ctx->hotplug_work, 0);
	enable_irq(ctx->int_irq);
	return 0;
}
#endif

static const struct dev_pm_ops hdmi_audio_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hdmi_audio_suspend, hdmi_audio_resume)
};

static struct platform_driver hdmi_audio_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &hdmi_audio_pm_ops,
	},
	.probe = hdmi_audio_probe,
	.remove = hdmi_audio_remove,
};

module_platform_driver(hdmi_audio_driver);

MODULE_AUTHOR("Rahul Sharma <rahul.sharma@samsung.com>");
MODULE_DESCRIPTION("HDMI Audio Codec Plugin");
MODULE_VERSION("1:1.0");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
