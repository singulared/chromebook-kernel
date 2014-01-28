/*
 * Exynos machine ASoC driver for boards using MAX98095 codec.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/max98095.h>

#include <mach/regs-clock.h>
#include <mach/gpio.h>

#include "i2s.h"
#include "s3c-i2s-v2.h"
#include "../codecs/max98095.h"
#include "codec_plugin.h"

#define DRV_NAME "daisy-snd-max98095"

/*
 * The initial rate that EPLL will be set to.  This is the smallest multiple (4)
 * of the desired master clock frequency 256 * FS for FS = 44.1khz that can
 * be generated on both the 5250 and 5420 SoCs.
 */
#define DEFAULT_EPLL_RATE (256 * 44100 * 4)

/* Audio clock settings are belonged to board specific part. Every
 * board can set audio source clock setting which is matched with H/W
 * like this function-'set_audio_clock_heirachy'.
 */
static int set_audio_clock_heirachy(struct platform_device *pdev)
{
	struct clk *fout_epll, *sclk_epll, *mout_audio0, *sclk_audio0;
	struct clk *mout_audss, *mout_i2s;
	int ret = 0;

	fout_epll = clk_get(&pdev->dev, "fout_epll");
	if (IS_ERR(fout_epll)) {
		printk(KERN_WARNING "%s: Cannot find fout_epll.\n",
				__func__);
		return -EINVAL;
	}

	sclk_epll = clk_get(&pdev->dev, "sclk_epll");
	if (IS_ERR(sclk_epll)) {
		printk(KERN_WARNING "%s: Cannot find sclk_epll.\n", __func__);
		ret = -EINVAL;
		goto out1;
	}

	mout_audio0 = clk_get(&pdev->dev, "mout_audio0");
	if (IS_ERR(mout_audio0)) {
		printk(KERN_WARNING "%s: Cannot find mout_audio0.\n", __func__);
		ret = -EINVAL;
		goto out2;
	}

	sclk_audio0 = clk_get(&pdev->dev, "sclk_audio0");
	if (IS_ERR(sclk_audio0)) {
		printk(KERN_WARNING "%s: Cannot find sclk_audio0.\n", __func__);
		ret = -EINVAL;
		goto out3;
	}

	mout_audss = clk_get(&pdev->dev, "mout_audss");
	if (IS_ERR(mout_audss)) {
		printk(KERN_WARNING
			"%s: Cannot find mout_audss clocks.\n", __func__);
		ret = -EINVAL;
		goto out4;
	}

	mout_i2s = clk_get(&pdev->dev, "mout_i2s");
	if (IS_ERR(mout_i2s)) {
		printk(KERN_WARNING
			"%s: Cannot find mout_i2s clocks.\n", __func__);
		ret = -EINVAL;
		goto out5;
	}

	/*
	 * fout_epll may have been initialized to operate at a frequency higher
	 * than the audio block's maximum (192Mhz on 5250, 200Mhz on 5420),
	 * so lower it to a reasonable rate here.  If we attempt to set
	 * fout_epll as the parent of mout_audss when fout_epll is operating
	 * at a frequency higher than the audio block's maximum, the system
	 * may hang.
	 */
	ret = clk_set_rate(fout_epll, DEFAULT_EPLL_RATE);
	if (ret < 0) {
		printk(KERN_WARNING "Failed to set epll rate.\n");
		goto out6;
	}

	/* Set audio clock hierarchy for S/PDIF */
	ret = clk_set_parent(sclk_epll, fout_epll);
	if (ret < 0) {
		printk(KERN_WARNING "Failed to set parent of epll.\n");
		goto out6;
	}
	ret = clk_set_parent(mout_audio0, sclk_epll);
	if (ret < 0) {
		printk(KERN_WARNING "Failed to set parent of mout audio0.\n");
		goto out6;
	}
	ret = clk_set_parent(mout_audss, fout_epll);
	if (ret < 0) {
		printk(KERN_WARNING "Failed to set parent of audss.\n");
		goto out6;
	}
	ret = clk_set_parent(mout_i2s, sclk_audio0);
	if (ret < 0) {
		printk(KERN_WARNING "Failed to set parent of mout i2s.\n");
		goto out6;
	}

	/* Ensure that the divider between mout_audio0 and sclk_audio0 is 1. */
	ret = clk_set_rate(sclk_audio0, clk_get_rate(mout_audio0));
	if (ret < 0)
		printk(KERN_WARNING "Failed to set audio bus rate (%d).\n",
			ret);

out6:
	clk_put(mout_i2s);
out5:
	clk_put(mout_audss);
out4:
	clk_put(sclk_audio0);
out3:
	clk_put(mout_audio0);
out2:
	clk_put(sclk_epll);
out1:
	clk_put(fout_epll);

	return ret;
}

static int set_epll_rate(struct device *card_dev, unsigned long rate)
{
	int ret;
	struct clk *fout_epll;

	fout_epll = clk_get(card_dev, "fout_epll");

	if (IS_ERR(fout_epll)) {
		printk(KERN_ERR "%s: failed to get fout_epll\n", __func__);
		return PTR_ERR(fout_epll);
	}

	if (rate == clk_get_rate(fout_epll))
		goto out;

	ret = clk_set_rate(fout_epll, rate);
	if (ret < 0) {
		printk(KERN_ERR "failed to clk_set_rate of fout_epll for audio\n");
		goto out;
	}
out:
	clk_put(fout_epll);

	return 0;
}

static int daisy_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int bfs, psr, rfs, ret;
	unsigned long rclk;
	unsigned long fin_rate;
	struct clk *fin_pll;
	struct device *card_dev = substream->pcm->card->dev;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U24:
	case SNDRV_PCM_FORMAT_S24:
		bfs = 48;
		break;
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		if (bfs == 48)
			rfs = 384;
		else
			rfs = 256;
		break;
	case 64000:
		rfs = 384;
		break;
	case 8000:
	case 11025:
	case 12000:
		if (bfs == 48)
			rfs = 768;
		else
			rfs = 512;
		break;
	default:
		return -EINVAL;
	}

	rclk = params_rate(params) * rfs;

	switch (rclk) {
	case 4096000:
	case 5644800:
	case 6144000:
	case 8467200:
	case 9216000:
		psr = 8;
		break;
	case 8192000:
	case 11289600:
	case 12288000:
	case 16934400:
	case 18432000:
		psr = 4;
		break;
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		psr = 2;
		break;
	case 67737600:
	case 73728000:
		psr = 1;
		break;
	default:
		printk(KERN_ERR "rclk = %lu is not yet supported!\n", rclk);
		return -EINVAL;
	}

	ret = set_epll_rate(card_dev, rclk * psr);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
					     SND_SOC_DAIFMT_NB_NF |
					     SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
					   SND_SOC_DAIFMT_NB_NF |
					   SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	fin_pll = clk_get(NULL, "fin_pll");
	if (IS_ERR(fin_pll)) {
		printk(KERN_ERR "%s: failed to get fin_pll clock\n", __func__);
		return PTR_ERR(fin_pll);
	}

	fin_rate = clk_get_rate(fin_pll);
	clk_put(fin_pll);

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, fin_rate, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, bfs);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * MAX98095 DAI operations.
 */
static struct snd_soc_ops daisy_ops = {
	.hw_params = daisy_hw_params,
};

static struct snd_soc_jack daisy_hp_jack;
static struct snd_soc_jack_pin daisy_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_gpio daisy_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
};

static struct snd_soc_jack daisy_mic_jack;
static struct snd_soc_jack_pin daisy_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static struct snd_soc_jack_gpio daisy_mic_jack_gpio = {
	.name = "mic detect",
	.report = SND_JACK_MICROPHONE,
};

static const struct snd_soc_dapm_route daisy_audio_map[] = {
	{"Mic Jack", "NULL", "MICBIAS2"},
	{"MIC2", "NULL", "Mic Jack"},
};

static const struct snd_soc_dapm_route max98090_audio_map[] = {
	{"Mic Jack", "NULL", "MICBIAS"},
	{"MIC2", "NULL", "Mic Jack"},
};

static const struct snd_soc_dapm_widget daisy_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static struct snd_soc_jack daisy_hdmi_jack;

static int get_hdmi(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct audio_codec_plugin *plugin;
	int ret = 0, state = 0;

	plugin = (struct audio_codec_plugin *)kcontrol->private_value;

	if (!plugin)
		return 0;

	if (!plugin->ops.hw_params)
		return 0;

	ret = plugin->ops.get_state(plugin->dev, &state);
	if (ret < 0)
		return 0;

	ucontrol->value.integer.value[0] = (long int)state;
	return 1;
}

static int put_hdmi(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct audio_codec_plugin *plugin;
	int ret = 0, state;

	plugin = (struct audio_codec_plugin *)kcontrol->private_value;

	if (!plugin)
		return 0;

	if (!plugin->ops.hw_params)
		return 0;

	state = (int)ucontrol->value.integer.value[0];
	ret = plugin->ops.set_state(plugin->dev,
		ucontrol->value.integer.value[0]);

	if (ret < 0)
		return 0;
	return 1;
}

static int daisy_hdmi_jack_report(int plugged)
{
	snd_soc_jack_report(&daisy_hdmi_jack,
			    plugged ? SND_JACK_AVOUT : 0,
			    SND_JACK_AVOUT);
	return 0;
}

static struct snd_kcontrol_new daisy_dapm_controls[] = {
	SOC_SINGLE_BOOL_EXT("HDMI Playback Switch", 0, get_hdmi, put_hdmi),
};

static int daisy_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct device_node *dn = card->dev->of_node;
	struct device_node *codec_dn = codec->dev->of_node;
	struct audio_codec_plugin *plugin;

	if (dn) {
		enum of_gpio_flags flags;

		daisy_mic_jack_gpio.gpio = of_get_named_gpio_flags(
				dn, "samsung,mic-det-gpios", 0, &flags);
		daisy_mic_jack_gpio.invert = !!(flags & OF_GPIO_ACTIVE_LOW);

		daisy_hp_jack_gpio.gpio = of_get_named_gpio_flags(
				dn, "samsung,hp-det-gpios", 0, &flags);
		daisy_hp_jack_gpio.invert = !!(flags & OF_GPIO_ACTIVE_LOW);
	}

	if (gpio_is_valid(daisy_mic_jack_gpio.gpio)) {
		snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
				 &daisy_mic_jack);
		snd_soc_jack_add_pins(&daisy_mic_jack,
				      ARRAY_SIZE(daisy_mic_jack_pins),
				      daisy_mic_jack_pins);
		snd_soc_jack_add_gpios(&daisy_mic_jack, 1,
				       &daisy_mic_jack_gpio);
	}

	if (gpio_is_valid(daisy_hp_jack_gpio.gpio)) {
		snd_soc_jack_new(codec, "Headphone Jack",
				 SND_JACK_HEADPHONE, &daisy_hp_jack);
		snd_soc_jack_add_pins(&daisy_hp_jack,
				      ARRAY_SIZE(daisy_hp_jack_pins),
				      daisy_hp_jack_pins);
		snd_soc_jack_add_gpios(&daisy_hp_jack, 1,
				       &daisy_hp_jack_gpio);
	}

	plugin = (void *)daisy_dapm_controls[0].private_value;
	if (plugin) {
		int state;
		snd_soc_jack_new(codec, "HDMI Jack",
				 SND_JACK_AVOUT, &daisy_hdmi_jack);
		plugin->jack_cb = daisy_hdmi_jack_report;

		plugin->ops.get_jack_state(plugin->dev, &state);
		daisy_hdmi_jack_report(state);
	}

	/* Microphone BIAS is needed to power the analog mic.
	 * MICBIAS2 is connected to analog mic (MIC3, which is in turn
	 * connected to MIC2 via 'External MIC') on the max98095 codec.
	 * Microphone BIAS is controlled by MICBIAS
	 * on the max98090 / max98091 codec.
	 *
	 * Ultimately, the following should hold:
	 *
	 *   Microphone in jack	    => MICBIAS/MICBIAS2 enabled &&
	 *			       'External Mic' = MIC2
	 *   Microphone not in jack => MICBIAS/MICBIAS2 disabled &&
	 *			       'External Mic' = MIC1
	*/
	if (of_device_is_compatible(codec_dn, "maxim,max98095"))
		snd_soc_dapm_force_enable_pin(dapm, "MICBIAS2");
	else
		snd_soc_dapm_force_enable_pin(dapm, "MICBIAS");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static int daisy_suspend_post(struct snd_soc_card *card)
{
	struct clk *mout_audss, *mout_i2s;
	int ret;

	mout_audss = clk_get(card->dev, "mout_audss");
	if (IS_ERR(mout_audss)) {
		pr_warn("Can't find mout_audss clock\n");
		ret = PTR_ERR(mout_audss);
		goto out1;
	}
	mout_i2s = clk_get(card->dev, "mout_i2s");
	if (IS_ERR(mout_i2s)) {
		pr_warn("Can't find mout_i2s clock\n");
		ret = PTR_ERR(mout_i2s);
		goto out2;
	}

	/*
	 * This really shouldn't fail. We probably won't come back from
	 * suspend if it does.
	 */
	ret = clk_set_parent(mout_i2s, mout_audss);
	WARN_ON(ret < 0);

	clk_put(mout_i2s);
out2:
	clk_put(mout_audss);
out1:
	return ret;
}

static int daisy_resume_pre(struct snd_soc_card *card)
{
	struct clk *sclk_audio0, *mout_i2s;
	int ret;

	sclk_audio0 = clk_get(card->dev, "sclk_audio0");
	if (IS_ERR(sclk_audio0)) {
		pr_warn("Can't find sclk_audio0 clock\n");
		ret = PTR_ERR(sclk_audio0);
		goto out1;
	}
	mout_i2s = clk_get(card->dev, "mout_i2s");
	if (IS_ERR(mout_i2s)) {
		pr_warn("Can't find mout_i2s clock\n");
		ret = PTR_ERR(mout_i2s);
		goto out2;
	}

	/*
	 * If we were able to reparent this for suspend, we ought to be
	 * able to change it back.
	 */
	ret = clk_set_parent(mout_i2s, sclk_audio0);
	WARN_ON(ret < 0);

	clk_put(mout_i2s);
out2:
	clk_put(sclk_audio0);
out1:
	return ret;
}

static int daisy_resume_post(struct snd_soc_card *card)
{
	if (gpio_is_valid(daisy_mic_jack_gpio.gpio))
		snd_soc_jack_gpio_detect(&daisy_mic_jack_gpio);

	if (gpio_is_valid(daisy_hp_jack_gpio.gpio))
		snd_soc_jack_gpio_detect(&daisy_hp_jack_gpio);

	return 0;
}

static struct snd_soc_dai_link daisy_dai[] = {
	{ /* Primary DAI i/f */
		.name = "MAX98095 RX",
		.stream_name = "Playback",
		.codec_dai_name = "HiFi",
		.init = daisy_init,
		.ops = &daisy_ops,
	}, { /* Capture i/f */
		.name = "MAX98095 TX",
		.stream_name = "Capture",
		.codec_dai_name = "HiFi",
		.ops = &daisy_ops,
	},
};

static struct snd_soc_card daisy_snd = {
	.name = "DAISY-I2S",
	.dai_link = daisy_dai,
	.num_links = ARRAY_SIZE(daisy_dai),
	.controls = daisy_dapm_controls,
	.num_controls = ARRAY_SIZE(daisy_dapm_controls),
	.dapm_widgets = daisy_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(daisy_dapm_widgets),
	.dapm_routes = daisy_audio_map,
	.num_dapm_routes = ARRAY_SIZE(daisy_audio_map),
	.suspend_post = daisy_suspend_post,
	.resume_pre = daisy_resume_pre,
	.resume_post = daisy_resume_post,
};

static int plugin_init(struct audio_codec_plugin **pplugin)
{
	struct device_node *plugin_node = NULL;
	struct platform_device *plugin_pdev;
	struct audio_codec_plugin *plugin;

	plugin_node = of_find_node_by_name(NULL, "hdmi-audio");
	if (!plugin_node)
		return -EFAULT;

	plugin_pdev = of_find_device_by_node(plugin_node);
	if (!plugin_pdev)
		return -EFAULT;

	plugin = dev_get_drvdata(&plugin_pdev->dev);
	if (!plugin)
		return -EFAULT;
	else
		*pplugin = plugin;

	return 0;
}

static int daisy_max98095_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &daisy_snd;
	struct device_node *i2s_node, *codec_node;
	struct audio_codec_plugin *plugin = NULL;
	const char *name;
	int i, ret;

	if (!pdev->dev.platform_data && !pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	name = of_get_property(pdev->dev.of_node, "card-name", NULL);
	if (name)
		card->name = name;

	i2s_node = of_parse_phandle(pdev->dev.of_node,
				    "samsung,i2s-controller", 0);
	if (!i2s_node) {
		dev_err(&pdev->dev,
			"Property 'i2s-controller' missing or invalid\n");
		return -EINVAL;
	}
	codec_node = of_parse_phandle(pdev->dev.of_node,
				      "samsung,audio-codec", 0);
	if (!codec_node) {
		dev_err(&pdev->dev,
			"Property 'audio-codec' missing or invalid\n");
		return -EINVAL;
	}

	if (of_device_is_compatible(codec_node, "maxim,max98091") ||
	    of_device_is_compatible(codec_node, "maxim,max98090") ||
	    of_device_is_compatible(codec_node, "maxim,max98089")) {
		card->dapm_routes = max98090_audio_map;
		card->num_dapm_routes = ARRAY_SIZE(max98090_audio_map);
	}

	for (i = 0; i < ARRAY_SIZE(daisy_dai); i++) {
		daisy_dai[i].codec_of_node = codec_node;
		daisy_dai[i].cpu_of_node =  i2s_node;
		daisy_dai[i].platform_of_node = i2s_node;
	}

	plugin_init(&plugin);
	daisy_dapm_controls[0].private_value = (unsigned long)plugin;

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		return ret;
	}

	ret = set_audio_clock_heirachy(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to set up clock hierarchy (%d)\n",
			ret);
		snd_soc_unregister_card(card);
	}

	return ret;
}

static int daisy_max98095_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id daisy_max98095_of_match[] = {
	{ .compatible = "google,daisy-audio-max98095", },
	{ .compatible = "google,daisy-audio-max98091", },
	{ .compatible = "google,daisy-audio-max98090", },
	{ .compatible = "google,daisy-audio-max98089", },
	{},
};

static struct platform_driver daisy_max98095_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = daisy_max98095_of_match,
	},
	.probe = daisy_max98095_driver_probe,
	.remove = daisy_max98095_driver_remove,
};

module_platform_driver(daisy_max98095_driver);

MODULE_DESCRIPTION("ALSA SoC DAISY MAX98095 machine driver");
MODULE_LICENSE("GPL");
