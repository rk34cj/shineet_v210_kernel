/* sound/soc/s3c24xx/s3c-i2c-v2.c
 *
 * ALSA Soc Audio Layer - I2S core for newer Samsung SoCs.
 *
 * Copyright (c) 2006 Wolfson Microelectronics PLC.
 *	Graeme Gregory graeme.gregory@wolfsonmicro.com
 *	linux@wolfsonmicro.com
 *
 * Copyright (c) 2008, 2007, 2004-2005 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <plat/regs-iis.h>

#include <mach/dma.h>

#include "s3c-i2s-v2.h"
#include "s3c-dma.h"

#undef S3C_IIS_V2_SUPPORTED

#if defined(CONFIG_CPU_S3C2412) || defined(CONFIG_CPU_S3C2413)
#define S3C_IIS_V2_SUPPORTED
#endif

#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5P)
#define S3C_IIS_V2_SUPPORTED
#endif

#ifndef S3C_IIS_V2_SUPPORTED
#error Unsupported CPU model
#endif

#define S3C2412_I2S_DEBUG_CON 0

static int s_suspend = 0;

static inline struct s3c_i2sv2_info *to_info(struct snd_soc_dai *cpu_dai)
{
	return cpu_dai->private_data;
}

#define bit_set(v, b) (((v) & (b)) ? 1 : 0)

#if S3C2412_I2S_DEBUG_CON
static void dbg_showcon(const char *fn, u32 con)
{
	printk(KERN_DEBUG "%s: LRI=%d, TXFEMPT=%d, RXFEMPT=%d, TXFFULL=%d, RXFFULL=%d\n", fn,
	       bit_set(con, S3C2412_IISCON_LRINDEX),
	       bit_set(con, S3C2412_IISCON_TXFIFO_EMPTY),
	       bit_set(con, S3C2412_IISCON_RXFIFO_EMPTY),
	       bit_set(con, S3C2412_IISCON_TXFIFO_FULL),
	       bit_set(con, S3C2412_IISCON_RXFIFO_FULL));

	printk(KERN_DEBUG "%s: PAUSE: TXDMA=%d, RXDMA=%d, TXCH=%d, RXCH=%d\n",
	       fn,
	       bit_set(con, S3C2412_IISCON_TXDMA_PAUSE),
	       bit_set(con, S3C2412_IISCON_RXDMA_PAUSE),
	       bit_set(con, S3C2412_IISCON_TXCH_PAUSE),
	       bit_set(con, S3C2412_IISCON_RXCH_PAUSE));
	printk(KERN_DEBUG "%s: ACTIVE: TXDMA=%d, RXDMA=%d, IIS=%d\n", fn,
	       bit_set(con, S3C2412_IISCON_TXDMA_ACTIVE),
	       bit_set(con, S3C2412_IISCON_RXDMA_ACTIVE),
	       bit_set(con, S3C2412_IISCON_IIS_ACTIVE));
}
#else
static inline void dbg_showcon(const char *fn, u32 con)
{
}
#endif


/* Turn on or off the transmission path. */
static void s3c2412_snd_txctrl(struct s3c_i2sv2_info *i2s, int on)
{
	void __iomem *regs = i2s->regs;
	u32 fic, con, mod, psr;

	pr_debug("%s(%d)\n", __func__, on);

	fic = readl(regs + S3C2412_IISFIC);
	con = readl(regs + S3C2412_IISCON);
	mod = readl(regs + S3C2412_IISMOD);
	psr = readl(regs + S3C2412_IISPSR);

	pr_debug("%s: IIS: CON=%x MOD=%x FIC=%x\n", __func__, con, mod, fic);
	//printk("s3c2412_snd_txctrl: base=0x%x PSR=0x%x\n",regs, psr);

	if (on) {
		con |= S3C2412_IISCON_TXDMA_ACTIVE | S3C2412_IISCON_IIS_ACTIVE;
		con &= ~S3C2412_IISCON_TXDMA_PAUSE;
		con &= ~S3C2412_IISCON_TXCH_PAUSE;

		switch (mod & S3C2412_IISMOD_MODE_MASK) {
		case S3C2412_IISMOD_MODE_TXONLY:
		case S3C2412_IISMOD_MODE_TXRX:
			/* do nothing, we are in the right mode */
			break;

		case S3C2412_IISMOD_MODE_RXONLY:
			mod &= ~S3C2412_IISMOD_MODE_MASK;
			mod |= S3C2412_IISMOD_MODE_TXRX;
			break;

		default:
			dev_err(i2s->dev, "TXEN: Invalid MODE %x in IISMOD\n",
				mod & S3C2412_IISMOD_MODE_MASK);
			break;
		}

		writel(con, regs + S3C2412_IISCON);
		writel(mod, regs + S3C2412_IISMOD);
	} else {
		/* Note, we do not have any indication that the FIFO problems
		 * tha the S3C2410/2440 had apply here, so we should be able
		 * to disable the DMA and TX without resetting the FIFOS.
		 */

		con |=  S3C2412_IISCON_TXDMA_PAUSE;
		con &= ~S3C2412_IISCON_TXDMA_ACTIVE;
		if (con & S5P_IISCON_TXSDMACTIVE) { /* If sec is active */
			writel(con, regs + S3C2412_IISCON);
			return;
		}
		con |=  S3C2412_IISCON_TXCH_PAUSE;

		switch (mod & S3C2412_IISMOD_MODE_MASK) {
		case S3C2412_IISMOD_MODE_TXRX:
			mod &= ~S3C2412_IISMOD_MODE_MASK;
			mod |= S3C2412_IISMOD_MODE_RXONLY;
			break;

		case S3C2412_IISMOD_MODE_TXONLY:
			mod &= ~S3C2412_IISMOD_MODE_MASK;
			con &= ~S3C2412_IISCON_IIS_ACTIVE;
			break;

		default:
			dev_err(i2s->dev, "TXDIS: Invalid MODE %x in IISMOD\n",
				mod & S3C2412_IISMOD_MODE_MASK);
			break;
		}

		writel(mod, regs + S3C2412_IISMOD);
		writel(con, regs + S3C2412_IISCON);
	}

	fic = readl(regs + S3C2412_IISFIC);
	dbg_showcon(__func__, con);
	pr_debug("%s: IIS: CON=%x MOD=%x FIC=%x\n", __func__, con, mod, fic);
}

static void s3c2412_snd_rxctrl(struct s3c_i2sv2_info *i2s, int on)
{
	void __iomem *regs = i2s->regs;
	u32 fic, con, mod;

	pr_debug("%s(%d)\n", __func__, on);

	fic = readl(regs + S3C2412_IISFIC);
	con = readl(regs + S3C2412_IISCON);
	mod = readl(regs + S3C2412_IISMOD);

	pr_debug("%s: IIS: CON=%x MOD=%x FIC=%x\n", __func__, con, mod, fic);

	if (on) {
		con |= S3C2412_IISCON_RXDMA_ACTIVE | S3C2412_IISCON_IIS_ACTIVE;
		con &= ~S3C2412_IISCON_RXDMA_PAUSE;
		con &= ~S3C2412_IISCON_RXCH_PAUSE;

		switch (mod & S3C2412_IISMOD_MODE_MASK) {
		case S3C2412_IISMOD_MODE_TXRX:
		case S3C2412_IISMOD_MODE_RXONLY:
			/* do nothing, we are in the right mode */
			break;

		case S3C2412_IISMOD_MODE_TXONLY:
			mod &= ~S3C2412_IISMOD_MODE_MASK;
			mod |= S3C2412_IISMOD_MODE_TXRX;
			break;

		default:
			dev_err(i2s->dev, "RXEN: Invalid MODE %x in IISMOD\n",
				mod & S3C2412_IISMOD_MODE_MASK);
		}

		writel(mod, regs + S3C2412_IISMOD);
		writel(con, regs + S3C2412_IISCON);
	} else {
		/* See txctrl notes on FIFOs. */

		con &= ~S3C2412_IISCON_RXDMA_ACTIVE;
		con |=  S3C2412_IISCON_RXDMA_PAUSE;
		con |=  S3C2412_IISCON_RXCH_PAUSE;

		switch (mod & S3C2412_IISMOD_MODE_MASK) {
		case S3C2412_IISMOD_MODE_RXONLY:
			con &= ~S3C2412_IISCON_IIS_ACTIVE;
			mod &= ~S3C2412_IISMOD_MODE_MASK;
			break;

		case S3C2412_IISMOD_MODE_TXRX:
			mod &= ~S3C2412_IISMOD_MODE_MASK;
			mod |= S3C2412_IISMOD_MODE_TXONLY;
			break;

		default:
			dev_err(i2s->dev, "RXDIS: Invalid MODE %x in IISMOD\n",
				mod & S3C2412_IISMOD_MODE_MASK);
		}

		writel(con, regs + S3C2412_IISCON);
		writel(mod, regs + S3C2412_IISMOD);
	}

	fic = readl(regs + S3C2412_IISFIC);
	pr_debug("%s: IIS: CON=%x MOD=%x FIC=%x\n", __func__, con, mod, fic);
}

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

/*
 * Wait for the LR signal to allow synchronisation to the L/R clock
 * from the codec. May only be needed for slave mode.
 */
static int s3c2412_snd_lrsync(struct s3c_i2sv2_info *i2s)
{
	u32 iiscon;
	unsigned long loops = msecs_to_loops(5);

	pr_debug("Entered %s\n", __func__);

	while (--loops) {
		iiscon = readl(i2s->regs + S3C2412_IISCON);
		if (iiscon & S3C2412_IISCON_LRINDEX)
			break;

		cpu_relax();
	}

	if (!loops) {
		printk(KERN_ERR "%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Set S3C2412 I2S DAI format
 */
static int s3c2412_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
			       unsigned int fmt)
{
	struct s3c_i2sv2_info *i2s = to_info(cpu_dai);
	u32 iismod;

	pr_debug("Entered %s\n", __func__);

	iismod = readl(i2s->regs + S3C2412_IISMOD);
	pr_debug("hw_params r: IISMOD: %x \n", iismod);

#if defined(CONFIG_CPU_S3C2412) || defined(CONFIG_CPU_S3C2413)
#define IISMOD_MASTER_MASK S3C2412_IISMOD_MASTER_MASK
#define IISMOD_SLAVE S3C2412_IISMOD_SLAVE
#define IISMOD_MASTER S3C2412_IISMOD_MASTER_INTERNAL
#endif

#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5P)
/* From Rev1.1 datasheet, we have two master and two slave modes:
 * IMS[11:10]:
 *	00 = master mode, fed from PCLK
 *	01 = master mode, fed from CLKAUDIO
 *	10 = slave mode, using PCLK
 *	11 = slave mode, using I2SCLK
 */
#define IISMOD_MASTER_MASK (1 << 11)
#define IISMOD_SLAVE (1 << 11)
#define IISMOD_MASTER (0 << 11)
#endif

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		i2s->master = 0;
		iismod &= ~IISMOD_MASTER_MASK;
		iismod |= IISMOD_SLAVE;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		i2s->master = 1;
		iismod &= ~IISMOD_MASTER_MASK;
		iismod |= IISMOD_MASTER;
		break;
	default:
		pr_err("unknwon master/slave format\n");
		return -EINVAL;
	}

	iismod &= ~S3C2412_IISMOD_SDF_MASK;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		iismod |= S3C2412_IISMOD_LR_RLOW;
		iismod |= S3C2412_IISMOD_SDF_MSB;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iismod |= S3C2412_IISMOD_LR_RLOW;
		iismod |= S3C2412_IISMOD_SDF_LSB;
		break;
	case SND_SOC_DAIFMT_I2S:
		iismod &= ~S3C2412_IISMOD_LR_RLOW;
		iismod |= S3C2412_IISMOD_SDF_IIS;
		break;
	default:
		pr_err("Unknown data format\n");
		return -EINVAL;
	}

	writel(iismod, i2s->regs + S3C2412_IISMOD);
	pr_debug("hw_params w: IISMOD: %x \n", iismod);
	return 0;
}

int s3c2412_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *socdai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai = rtd->dai;
	struct s3c_i2sv2_info *i2s = to_info(dai->cpu_dai);
	u32 dma_tsfr_size = 0;
	u32 iismod;

	pr_debug("Entered %s\n", __func__);

	/* TODO */
	switch (params_channels(params)) {
		case 1:
			dma_tsfr_size = 2;
			break;
		case 2:
			dma_tsfr_size = 4;
			break;
		case 4:
			break;
		case 6:
			break;
		default:
			break;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		i2s->dma_playback->dma_size = dma_tsfr_size;
		dai->cpu_dai->dma_data = i2s->dma_playback;
	} else {
		i2s->dma_capture->dma_size = dma_tsfr_size;
		dai->cpu_dai->dma_data = i2s->dma_capture;
	}

	/* Working copies of register */
	iismod = readl(i2s->regs + S3C2412_IISMOD);
	pr_debug("%s: r: IISMOD: %x\n", __func__, iismod);

#if defined(CONFIG_CPU_S3C2412) || defined(CONFIG_CPU_S3C2413)
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		iismod |= S3C2412_IISMOD_8BIT;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		iismod &= ~S3C2412_IISMOD_8BIT;
		break;
	}
#endif

#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5P)
	iismod &= ~(S3C64XX_IISMOD_BLC_MASK | S3C2412_IISMOD_BCLK_MASK);
	/* Sample size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		/* 8 bit sample, 16fs BCLK */
		iismod |= (S3C64XX_IISMOD_BLC_8BIT | S3C2412_IISMOD_BCLK_16FS);
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		/* 16 bit sample, 32fs BCLK */
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		/* 24 bit sample, 48fs BCLK */
		iismod |= (S3C64XX_IISMOD_BLC_24BIT | S3C2412_IISMOD_BCLK_48FS);
		break;
	}

	/* Set the IISMOD[25:24](BLC_P) to same value */
	iismod &= ~(S5P_IISMOD_BLCPMASK);
	iismod |= ((iismod & S3C64XX_IISMOD_BLC_MASK) << 11);
#endif

	writel(iismod, i2s->regs + S3C2412_IISMOD);
	pr_debug("%s: w: IISMOD: %x\n", __func__, iismod);
	return 0;
}
EXPORT_SYMBOL_GPL(s3c2412_i2s_hw_params);

int s3c2412_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct s3c_i2sv2_info *i2s = to_info(rtd->dai->cpu_dai);
	int capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	unsigned long irqs;
	int ret = 0;

	pr_debug("Entered %s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!i2s->master) {
			ret = s3c2412_snd_lrsync(i2s);
			if (ret)
				goto exit_err;
		}

		local_irq_save(irqs);

		if (capture) {
			s3c2412_snd_rxctrl(i2s, 1);
			//printk("rxctrl(1)\n");
		} else {
			s3c2412_snd_txctrl(i2s, 1);
		}
		local_irq_restore(irqs);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		local_irq_save(irqs);

		if (capture) {
			//JY.Lai
			s3c2412_snd_rxctrl(i2s, 0);
			//printk("rxctrl(0)\n");
		} else {
			s3c2412_snd_txctrl(i2s, 0);
		}
		local_irq_restore(irqs);
		break;
	default:
		ret = -EINVAL;
		break;
	}

exit_err:
	return ret;
}
EXPORT_SYMBOL_GPL(s3c2412_i2s_trigger);

/*
 * Set S3C2412 Clock dividers
 */
static int s3c2412_i2s_set_clkdiv(struct snd_soc_dai *cpu_dai,
				  int div_id, int div)
{
	struct s3c_i2sv2_info *i2s = to_info(cpu_dai);
	u32 reg;

	pr_debug("%s(%p, %d, %d)\n", __func__, cpu_dai, div_id, div);

	switch (div_id) {
	case S3C_I2SV2_DIV_BCLK:
		if (div > 3) {
			/* convert value to bit field */

			switch (div) {
			case 16:
				div = S3C2412_IISMOD_BCLK_16FS;
				break;

			case 32:
				div = S3C2412_IISMOD_BCLK_32FS;
				break;

			case 24:
				div = S3C2412_IISMOD_BCLK_24FS;
				break;

			case 48:
				div = S3C2412_IISMOD_BCLK_48FS;
				break;

			default:
				return -EINVAL;
			}
		}

		reg = readl(i2s->regs + S3C2412_IISMOD);
		reg &= ~S3C2412_IISMOD_BCLK_MASK;
		writel(reg | div, i2s->regs + S3C2412_IISMOD);

		pr_debug("%s: MOD=%08x\n", __func__, readl(i2s->regs + S3C2412_IISMOD));
		break;

	case S3C_I2SV2_DIV_RCLK:
		if (div > 3) {
			/* convert value to bit field */

			switch (div) {
			case 256:
				div = S3C2412_IISMOD_RCLK_256FS;
				break;

			case 384:
				div = S3C2412_IISMOD_RCLK_384FS;
				break;

			case 512:
				div = S3C2412_IISMOD_RCLK_512FS;
				break;

			case 768:
				div = S3C2412_IISMOD_RCLK_768FS;
				break;

			default:
				return -EINVAL;
			}
		}

		reg = readl(i2s->regs + S3C2412_IISMOD);
		reg &= ~S3C2412_IISMOD_RCLK_MASK;
		writel(reg | div, i2s->regs + S3C2412_IISMOD);
		pr_debug("%s: MOD=%08x\n", __func__, readl(i2s->regs + S3C2412_IISMOD));
		break;

	case S3C_I2SV2_DIV_PRESCALER:
		if (div >= 0) {
			writel((div << 8) | S3C2412_IISPSR_PSREN,
			       i2s->regs + S3C2412_IISPSR);
		} else {
			writel(0x0, i2s->regs + S3C2412_IISPSR);
		}
		//pr_debug("%s: PSR=%08x\n", __func__, readl(i2s->regs + S3C2412_IISPSR));
		//printk("s3c-i2s-v2.c: PSR=%08x\n", __func__, readl(i2s->regs + S3C2412_IISPSR));
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* default table of all avaialable root fs divisors */
static unsigned int iis_fs_tab[] = { 256, 512, 384, 768 };

int s3c_i2sv2_iis_calc_rate(struct s3c_i2sv2_rate_calc *info,
			    unsigned int *fstab,
			    unsigned int rate, struct clk *clk)
{
	unsigned long clkrate = clk_get_rate(clk);
	unsigned int div;
	unsigned int fsclk;
	unsigned int actual;
	unsigned int fs;
	unsigned int fsdiv;
	signed int deviation = 0;
	unsigned int best_fs = 0;
	unsigned int best_div = 0;
	unsigned int best_rate = 0;
	unsigned int best_deviation = INT_MAX;

	pr_debug("Input clock rate %ldHz\n", clkrate);

	if (fstab == NULL)
		fstab = iis_fs_tab;

	for (fs = 0; fs < ARRAY_SIZE(iis_fs_tab); fs++) {
		fsdiv = iis_fs_tab[fs];

		fsclk = clkrate / fsdiv;
		div = fsclk / rate;

		if ((fsclk % rate) > (rate / 2))
			div++;

		if (div <= 1)
			continue;

		actual = clkrate / (fsdiv * div);
		deviation = actual - rate;

		printk(KERN_DEBUG "%ufs: div %u => result %u, deviation %d\n",
		       fsdiv, div, actual, deviation);

		deviation = abs(deviation);

		if (deviation < best_deviation) {
			best_fs = fsdiv;
			best_div = div;
			best_rate = actual;
			best_deviation = deviation;
		}

		if (deviation == 0)
			break;
	}

	printk(KERN_DEBUG "best: fs=%u, div=%u, rate=%u\n",
	       best_fs, best_div, best_rate);

	info->fs_div = best_fs;
	info->clk_div = best_div;

	return 0;
}
EXPORT_SYMBOL_GPL(s3c_i2sv2_iis_calc_rate);

int s3c_i2sv2_probe(struct platform_device *pdev,
		    struct snd_soc_dai *dai,
		    struct s3c_i2sv2_info *i2s,
		    unsigned long base)
{
	struct device *dev = &pdev->dev;
	unsigned int iismod;

	i2s->dev = dev;

	/* record our i2s structure for later use in the callbacks */
	dai->private_data = i2s;

	i2s->regs = ioremap(base, 0x100);
	if (i2s->regs == NULL) {
		dev_err(dev, "cannot ioremap registers\n");
		return -ENXIO;
	}

	i2s->iis_pclk = clk_get(dev, "iis");
	if (IS_ERR(i2s->iis_pclk)) {
		dev_err(dev, "failed to get iis_clock\n");
		iounmap(i2s->regs);
		return -ENOENT;
	}

	clk_enable(i2s->iis_pclk);

#if defined(CONFIG_PLAT_S5P)
	writel(((1<<0)|(1<<31)), i2s->regs + S3C2412_IISCON);
#endif

	/* Mark ourselves as in TXRX mode so we can run through our cleanup
	 * process without warnings. */
	iismod = readl(i2s->regs + S3C2412_IISMOD);
	iismod |= S3C2412_IISMOD_MODE_TXRX;
	writel(iismod, i2s->regs + S3C2412_IISMOD);
	s3c2412_snd_txctrl(i2s, 0);
	s3c2412_snd_rxctrl(i2s, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(s3c_i2sv2_probe);

#ifdef CONFIG_PM

#include <mach/map.h>
#define S3C_VA_AUDSS	S3C_ADDR(0x01600000)	/* Audio SubSystem */
#include <mach/regs-audss.h>
static int s3c2412_i2s_suspend(struct snd_soc_dai *dai)
{
	struct s3c_i2sv2_info *i2s = to_info(dai);
	u32 iismod;

	i2s->suspend_iismod = readl(i2s->regs + S3C2412_IISMOD);
	i2s->suspend_iiscon = readl(i2s->regs + S3C2412_IISCON);
	i2s->suspend_iispsr = readl(i2s->regs + S3C2412_IISPSR);

	/* Is this dai for I2Sv5? */
	if(dai->id == 0)
		i2s->suspend_audss_clksrc = readl(S5P_CLKSRC_AUDSS);

	/* some basic suspend checks */

	iismod = readl(i2s->regs + S3C2412_IISMOD);

	if (iismod & S3C2412_IISCON_RXDMA_ACTIVE)
		pr_warning("%s: RXDMA active?\n", __func__);

	if (iismod & S3C2412_IISCON_TXDMA_ACTIVE)
		pr_warning("%s: TXDMA active?\n", __func__);

	if (iismod & S3C2412_IISCON_IIS_ACTIVE)
		pr_warning("%s: IIS active\n", __func__);

	return 0;
}

static int s3c2412_i2s_resume(struct snd_soc_dai *dai)
{
	struct s3c_i2sv2_info *i2s = to_info(dai);

	pr_info("dai_active %d, IISMOD %08x, IISCON %08x\n",
		dai->active, i2s->suspend_iismod, i2s->suspend_iiscon);

	writel(i2s->suspend_iiscon, i2s->regs + S3C2412_IISCON);
	writel(i2s->suspend_iismod, i2s->regs + S3C2412_IISMOD);
	writel(i2s->suspend_iispsr, i2s->regs + S3C2412_IISPSR);

	/* Is this dai for I2Sv5? */
	if(dai->id == 0)
		writel(i2s->suspend_audss_clksrc, S5P_CLKSRC_AUDSS);

	writel(S3C2412_IISFIC_RXFLUSH | S3C2412_IISFIC_TXFLUSH,
	       i2s->regs + S3C2412_IISFIC);

	ndelay(250);
	writel(0x0, i2s->regs + S3C2412_IISFIC);

	return 0;
}
#else
#define s3c2412_i2s_suspend NULL
#define s3c2412_i2s_resume  NULL
#endif

int s3c_i2sv2_register_dai(struct snd_soc_dai *dai)
{
	struct snd_soc_dai_ops *ops = dai->ops;

	ops->trigger = s3c2412_i2s_trigger;
	ops->hw_params = s3c2412_i2s_hw_params;
	ops->set_fmt = s3c2412_i2s_set_fmt;
	ops->set_clkdiv = s3c2412_i2s_set_clkdiv;

	dai->suspend = s3c2412_i2s_suspend;
	dai->resume = s3c2412_i2s_resume;

	return snd_soc_register_dai(dai);
}
EXPORT_SYMBOL_GPL(s3c_i2sv2_register_dai);

MODULE_LICENSE("GPL");
