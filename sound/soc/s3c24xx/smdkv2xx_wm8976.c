/*
 *  smdk64xx_wm8976.c
 *
 *  Copyright (c) 
 *  Author: 
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/wm8976.h"
#include "s3c-dma.h"
#include "s3c64xx-i2s.h"

#include <linux/io.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

#define I2S_NUM 1

//#define CONFIG_SND_DEBUG
//#undef CONFIG_SND_DEBUG
#ifdef CONFIG_SND_DEBUG
#define s3cdbg(x...) printk(x)
#else
#define s3cdbg(x...)
#endif

#if 0
#define wait_stable(utime_out)					\
	do {							\
		if (!utime_out)					\
			utime_out = 1000;			\
		utime_out = loops_per_jiffy / HZ * utime_out;	\
		while (--utime_out) { 				\
			cpu_relax();				\
		}						\
	} while (0);
#else

#define wait_stable(utime_out)					\
	do {							\
		while (--utime_out) { 				\
			cpu_relax();				\
		}						\
	} while (0);
#endif

extern struct snd_soc_platform s3c_dma_wrapper;

static int set_epll_rate(unsigned long rate);

#if 1

extern void msleep(unsigned int msecs);

/* define the scenarios */
#define SMT_AUDIO_OFF			0
#define SMT_GSM_CALL_AUDIO_HANDSET	1
#define SMT_GSM_CALL_AUDIO_HEADSET	2
#define SMT_GSM_CALL_AUDIO_BLUETOOTH	3
#define SMT_STEREO_TO_SPEAKERS		4
#define SMT_STEREO_TO_HEADPHONES	5
#define SMT_CAPTURE_HANDSET		6
#define SMT_CAPTURE_HEADSET		7
#define SMT_CAPTURE_BLUETOOTH		8
#define SMT_ENABLE_SPEAKERS_AND_HANDSET	9

static int smdk6410_scenario = 0;
extern int wm8976_write(struct snd_soc_codec  *codec, unsigned int reg,
	unsigned int value);

static int set_scenario_endpoints(struct snd_soc_codec *codec, int scenario)
{
        switch (smdk6410_scenario) {
        case SMT_AUDIO_OFF:
                snd_soc_dapm_disable_pin(codec, "Audio Out1");
                snd_soc_dapm_disable_pin(codec, "Audio Out2");
                snd_soc_dapm_disable_pin(codec, "GSM Line Out");
                snd_soc_dapm_disable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
                break;
        case SMT_ENABLE_SPEAKERS_AND_HANDSET:
                snd_soc_dapm_enable_pin(codec, "Audio Out1");
                snd_soc_dapm_enable_pin(codec, "Audio Out2");
                snd_soc_dapm_enable_pin(codec, "GSM Line Out");
                snd_soc_dapm_enable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_enable_pin(codec, "Call Mic");
                break;
        case SMT_GSM_CALL_AUDIO_HANDSET:
                snd_soc_dapm_enable_pin(codec, "Audio Out1");
                snd_soc_dapm_enable_pin(codec, "GSM Line Out");
                snd_soc_dapm_enable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_enable_pin(codec, "Call Mic");
                break;
        case SMT_GSM_CALL_AUDIO_HEADSET:
                snd_soc_dapm_enable_pin(codec, "Audio Out1");
                snd_soc_dapm_enable_pin(codec, "GSM Line Out");
                snd_soc_dapm_enable_pin(codec, "GSM Line In");
                snd_soc_dapm_enable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
                break;
        case SMT_GSM_CALL_AUDIO_BLUETOOTH:
		snd_soc_dapm_disable_pin(codec, "Audio Out2");
                snd_soc_dapm_disable_pin(codec, "Audio Out1");
                snd_soc_dapm_enable_pin(codec, "GSM Line Out");
                snd_soc_dapm_enable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
                break;
        case SMT_STEREO_TO_SPEAKERS:
                snd_soc_dapm_enable_pin(codec, "Audio Out2");
                snd_soc_dapm_disable_pin(codec, "Audio Out1");
                snd_soc_dapm_disable_pin(codec, "GSM Line Out");
                snd_soc_dapm_disable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
                break;
        case SMT_STEREO_TO_HEADPHONES:
                snd_soc_dapm_disable_pin(codec, "Audio Out2");
                snd_soc_dapm_enable_pin(codec, "Audio Out1");
                snd_soc_dapm_disable_pin(codec, "GSM Line Out");
                snd_soc_dapm_disable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
                break;
        case SMT_CAPTURE_HANDSET:
                snd_soc_dapm_disable_pin(codec, "Audio Out1");
                snd_soc_dapm_disable_pin(codec, "GSM Line Out");
                snd_soc_dapm_disable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_enable_pin(codec, "Call Mic");

                break;
        case SMT_CAPTURE_HEADSET:
                snd_soc_dapm_disable_pin(codec, "Audio Out1");
                snd_soc_dapm_disable_pin(codec, "GSM Line Out");
                snd_soc_dapm_disable_pin(codec, "GSM Line In");
                snd_soc_dapm_enable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
                break;
        case SMT_CAPTURE_BLUETOOTH:
                snd_soc_dapm_disable_pin(codec, "Audio Out2");
                snd_soc_dapm_disable_pin(codec, "Audio Out1");
                snd_soc_dapm_disable_pin(codec, "GSM Line Out");
                snd_soc_dapm_disable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
                break;
        default:
                snd_soc_dapm_disable_pin(codec, "Audio Out2");
                snd_soc_dapm_disable_pin(codec, "Audio Out1");
                snd_soc_dapm_disable_pin(codec, "GSM Line Out");
                snd_soc_dapm_disable_pin(codec, "GSM Line In");
                snd_soc_dapm_disable_pin(codec, "Headset Mic");
                snd_soc_dapm_disable_pin(codec, "Call Mic");
        }

        snd_soc_dapm_sync(codec);

        return 0;
}
static int smdk6410_get_scenario(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        ucontrol->value.integer.value[0] = smdk6410_scenario;
        return 0;
}
static int smdk6410_set_scenario(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

        if (smdk6410_scenario == ucontrol->value.integer.value[0])
                return 0;

        smdk6410_scenario = ucontrol->value.integer.value[0];
        set_scenario_endpoints(codec, smdk6410_scenario);
        return 1;
}

static int s3c6410_hifi_hw_params(struct snd_pcm_substream *substream,	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct s3c_i2sv2_rate_calc div;
	unsigned int epll_out_rate;
	int ret;
	int rfs, bfs, psr, rclk;

	s3cdbg("Entered %s\n",__FUNCTION__);

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
		printk("Not yet supported!\n");
		return -EINVAL;
	}


	epll_out_rate = rclk * psr;
	s3cdbg("epll_out=%d, rclk=%d, psr=%d\n", epll_out_rate, rclk, psr);

	/* Set EPLL clock rate */
	ret = set_epll_rate(epll_out_rate);
	if (ret < 0) {
		printk(KERN_ERR "%s: set epll rate failed\n", __func__);
		return ret;
	}

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk(KERN_ERR "%s: set codec dai failed\n", __func__);
		return ret;
	}
	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk(KERN_ERR "%s: set cpu dai failed\n", __func__);
		return ret;
	}

#if 1
	//ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_MUX,
	//				0, SND_SOC_CLOCK_OUT);
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_CDCLK,
					0, SND_SOC_CLOCK_OUT);
	/* We use SCLK_AUDIO for basic ops in SoC-Master mode */
	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_MUX,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "%s: set CLKSRC for cpu dai failed\n", __func__);
		return ret;
	}
#endif

#if 1
	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_RCLK, rfs);
	if (ret < 0) {
		printk(KERN_ERR "%s:set clk divider RCLK for cpu dai failed\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_BCLK, bfs);
	if (ret < 0) {
		printk(KERN_ERR "%s:set clk divider BCLK for cpu dai failed\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_PRESCALER,psr-1);
	//ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_PRESCALER, 6 -1);

	if (ret < 0) {
		printk(KERN_ERR "%s:set clk prescaler for cpu dai failed\n", __func__);
		return ret;
	}
#endif


	return 0;
}

/* machine dapm widgets */
static const struct snd_soc_dapm_widget s3c6410_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Audio Out1", NULL),
       	SND_SOC_DAPM_LINE("Audio Out2", NULL),
	SND_SOC_DAPM_LINE("GSM Line Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line In", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Call Mic", NULL),
};
static const char *smt_scenarios[] = {
	"Off",
	"GSM Handset",
	"GSM Headset",
	"GSM Bluetooth",
	"Speakers",
	"Headphones",
	"Capture Handset",
	"Capture Headset",
	"Capture Bluetooth"
};

static const struct soc_enum smdk_scenario_enum[] = {
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smt_scenarios), smt_scenarios),
};

static const struct snd_kcontrol_new wm8976_s3c6410_controls[] = {
	SOC_ENUM_EXT("SMT Mode", smdk_scenario_enum[0], smdk6410_get_scenario, smdk6410_set_scenario),
};

/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Connections to the ... */
	{"Audio Out1", NULL, "LOUT1"},
	{"Audio Out1", NULL, "ROUT1"},
        {"Audio Out2", NULL, "LOUT2"},
        {"Audio Out2", NULL, "ROUT2"},
	/* Connections to the GSM Module */
	{"GSM Line Out", NULL, "OUT4"},
	{"GSM Line Out", NULL, "OUT4"},
	{"MICP", NULL, "GSM Line In"},
	{"MICN", NULL, "GSM Line In"},

	/* Connections to Headset */
	//{"L2", NULL, "Mic Bias"},
	//{"Mic Bias", NULL, "Headset Mic"},

	/* Call Mic */
	{"MICP", NULL, "Mic Bias"},
	{"MICN", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Call Mic"},
};

static int s3C6410_wm8976_init(struct snd_soc_codec *codec)
{
	int i, err, ret;

	s3cdbg("Entered %s\n",__FUNCTION__);
	//snd_soc_dapm_disable_pin(codec, "ROUT2");
	//snd_soc_dapm_disable_pin(codec, "LOUT2");
	/* Add s3c6410 specific widgets */
	for(i = 0; i < ARRAY_SIZE(s3c6410_dapm_widgets); i++) 
	{
		snd_soc_dapm_new_control(codec, &s3c6410_dapm_widgets[i]);
	}
        /* set endpoints to default mode */
        set_scenario_endpoints(codec, SMT_AUDIO_OFF);

	/* add s3c6410 specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8976_s3c6410_controls); i++)
	{
		err = snd_ctl_add(codec->card, snd_soc_cnew(&wm8976_s3c6410_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}
	smdk6410_scenario = SMT_ENABLE_SPEAKERS_AND_HANDSET;
	set_scenario_endpoints(codec, SMT_ENABLE_SPEAKERS_AND_HANDSET);
	/* set up s3c6410 specific audio paths */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	
	
	snd_soc_dapm_sync(codec);
#if 0

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(&wm8976_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(&s3c64xx_i2s_dai[I2S_NUM], SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set WM8580 to drive MCLK from its MCLK-pin */
	ret = snd_soc_dai_set_clkdiv(&wm8976_dai, 1, 1);
	if (ret < 0)
		return ret;

	/* Explicitly set WM8580-DAC to source from MCLK */
	ret = snd_soc_dai_set_clkdiv(&wm8976_dai, 2, 1);
	if (ret < 0)
		return ret;

#endif
	return 0;
}

static struct snd_soc_ops s3C6410_hifi_ops = {
	.hw_params = s3c6410_hifi_hw_params,
};

#endif

static struct snd_soc_dai_link smdk64xx_dai[] = {
{ /* Primary Playback i/f */
	.name = "WM8976 PAIF RX",
	.stream_name = "Playback",
	.cpu_dai = &s3c64xx_i2s_dai[I2S_NUM],
	.codec_dai = &wm8976_dai,	
	.init = s3C6410_wm8976_init,		
	.ops = &s3C6410_hifi_ops,
},
#if 0
{ /* Primary Capture i/f */
	.name = "WM8976 PAIF TX",
	.stream_name = "Capture",
	.cpu_dai = &s3c64xx_i2s_dai[I2S_NUM],
	.codec_dai = &wm8976_dai[WM8976_DAI_PAIFTX],
	.init = smdk64xx_wm8976_init_paiftx,
	.ops = &smdk64xx_ops,
},
#endif
};

static struct snd_soc_card smdk64xx = {
	.name = "smdk",
	.platform = &s3c_dma_wrapper,
	.dai_link = smdk64xx_dai,
	.num_links = ARRAY_SIZE(smdk64xx_dai),
};

static struct snd_soc_device smdk64xx_snd_devdata = {
	.card = &smdk64xx,
	.codec_dev = &soc_codec_dev_wm8976,
};

static struct platform_device *smdk64xx_snd_device;



static int set_epll_rate(unsigned long rate)
{
	//printk("Entered %s\n", __func__);
	struct clk *fout_epll;
	unsigned int wait_utime = 500;

	fout_epll = clk_get(NULL, "fout_epll");
	if (IS_ERR(fout_epll)) {
		printk(KERN_ERR "%s: failed to get fout_epll\n", __func__);
		return -ENOENT;
	}

	if (rate == clk_get_rate(fout_epll)) {
		//printk("set_epll_rate, rate=%d\n", clk_get_rate(fout_epll));
		goto out;
	}

	clk_disable(fout_epll);
	wait_stable(wait_utime);

	clk_set_rate(fout_epll, rate);
	
	//wait_stable(wait_utime);

	clk_enable(fout_epll);

out:
	clk_put(fout_epll);

	return 0;
}

static int __init smdk64xx_audio_init(void)
{
	int ret;
#if 1
		u32 val;
		u32 reg;
	
#include <mach/map.h>
#define S3C_VA_AUDSS	S3C_ADDR(0x01600000)	/* Audio SubSystem */
#include <mach/regs-audss.h>
		/* We use I2SCLK for rate generation, so set EPLLout as
		 * the parent of I2SCLK.
		 */
		val = readl(S5P_CLKSRC_AUDSS);
		val &= ~(0x3<<2);
		val |= (1<<0);
		writel(val, S5P_CLKSRC_AUDSS);
	
		val = readl(S5P_CLKGATE_AUDSS);
		val |= (0x7f<<0);
		writel(val, S5P_CLKGATE_AUDSS);
	
#ifdef CONFIG_S5P_LPAUDIO
		/* yman.seo CLKOUT is prior to CLK_OUT of SYSCON. XXTI & XUSBXTI work in sleep mode */
		reg = __raw_readl(S5P_OTHERS);
		reg &= ~(0x0003 << 8);
		reg |= 0x0003 << 8; /* XUSBXTI */
		__raw_writel(reg, S5P_OTHERS);
#else
		/* yman.seo Set XCLK_OUT as 24MHz (XUSBXTI -> 24MHz) */
		reg = __raw_readl(S5P_CLK_OUT);
		reg &= ~S5P_CLKOUT_CLKSEL_MASK;
		reg |= S5P_CLKOUT_CLKSEL_XUSBXTI;
		reg &= ~S5P_CLKOUT_DIV_MASK;
		reg |= 0x0001 << S5P_CLKOUT_DIV_SHIFT;	/* DIVVAL = 1, Ratio = 2 = DIVVAL + 1 */
		__raw_writel(reg, S5P_CLK_OUT);
	
		reg = __raw_readl(S5P_OTHERS);
		reg &= ~(0x0003 << 8);
		reg |= 0x0000 << 8; /* Clock from SYSCON */
		__raw_writel(reg, S5P_OTHERS);
#endif

#endif

	smdk64xx_snd_device = platform_device_alloc("soc-audio", -1);
	if (!smdk64xx_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdk64xx_snd_device, &smdk64xx_snd_devdata);
	smdk64xx_snd_devdata.dev = &smdk64xx_snd_device->dev;
	ret = platform_device_add(smdk64xx_snd_device);

	if (ret)
		platform_device_put(smdk64xx_snd_device);

	return ret;
}

module_init(smdk64xx_audio_init);

MODULE_AUTHOR("shengm");
MODULE_DESCRIPTION("ALSA SoC SMDK64XX WM8976");
MODULE_LICENSE("GPL");


