/*
 * s3c-idma.c  --  I2S0's Internal Dma driver
 *
 * Copyright (c) 2010 Samsung Electronics Co. Ltd
 * 	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <plat/regs-iis.h>

#include "s3c-dma.h"
#include "s3c-idma.h"

/** Debug **/
#include <mach/map.h>
#include <mach/regs-clock.h>
#define dump_i2s()	do {	\
				printk("%s:%s:%d\n", __FILE__, __func__, __LINE__);	\
				printk("\tS3C_IISCON : %x", readl(s3c_idma.regs + S3C2412_IISCON));	\
				printk("\tS3C_IISMOD : %x\n", readl(s3c_idma.regs + S3C2412_IISMOD));	\
				printk("\tS3C_IISFIC : %x", readl(s3c_idma.regs + S3C2412_IISFIC));	\
				printk("\tS3C_IISFICS : %x", readl(s3c_idma.regs + S5P_IISFICS));	\
				printk("\tS3C_IISPSR : %x\n", readl(s3c_idma.regs + S3C2412_IISPSR));	\
				printk("\tS3C_IISAHB : %x\n", readl(s3c_idma.regs + S5P_IISAHB));	\
				printk("\tS3C_IISSTR : %x\n", readl(s3c_idma.regs + S5P_IISSTR));	\
				printk("\tS3C_IISSIZE : %x\n", readl(s3c_idma.regs + S5P_IISSIZE));	\
				printk("\tS3C_IISADDR0 : %x\n", readl(s3c_idma.regs + S5P_IISADDR0));	\
				printk("\tS5P_CLKGATE_D20 : %x\n", readl(S5P_CLKGATE_D20));	\
				printk("\tS5P_LPMP_MODE_SEL : %x\n", readl(S5P_LPMP_MODE_SEL));	\
			} while (0)

#define S5P_IISLVLINTMASK       (0xf<<20)
#define LP_BUFSIZE		(64 * 1024)

static const struct snd_pcm_hardware s3c_idma_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		    SNDRV_PCM_INFO_BLOCK_TRANSFER |
		    SNDRV_PCM_INFO_MMAP |
		    SNDRV_PCM_INFO_MMAP_VALID |
		    SNDRV_PCM_INFO_PAUSE |
		    SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_DOUBLE |
			SNDRV_PCM_INFO_BATCH,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		    SNDRV_PCM_FMTBIT_U16_LE |
		    SNDRV_PCM_FMTBIT_U8 |
		    SNDRV_PCM_FMTBIT_S8,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = MAX_LP_BUFF,
	.period_bytes_min = 128,
	.period_bytes_max = 16 * 1024,
	.periods_min = 2,
	.periods_max = 128,
	.fifo_size = 32,
};

struct lpam_i2s_pdata {
	spinlock_t 	lock;
	int 		state;
	dma_addr_t	start;
	dma_addr_t	pos;
	dma_addr_t	end;
	dma_addr_t	period;

};

	/********************
	 * Internal DMA i/f *
	 ********************/
static struct s3c_idma_info {
	void __iomem  *regs;
	unsigned int   dma_prd;
	unsigned int   dma_end;
	unsigned int   period_val;
	spinlock_t    lock;
	void          *token;
	void (*cb)(void *dt, int bytes_xfer);
} s3c_idma;

/* s5p_i2s_is_clk_enabled() is from s3c64xx_i2s.c */
extern int s5p_i2s_is_clk_enabled(void);

void s3c_idma_getpos(dma_addr_t *src)
{
	if (s5p_i2s_is_clk_enabled())
		*src = LP_TXBUFF_ADDR + (readl(s3c_idma.regs + S5P_IISTRNCNT) & 0xffffff) * 4;
	else
		*src = LP_TXBUFF_ADDR + 0x2000;	/* Report 0x2000, when audio clk is gated. */
}

static int s3c_idma_enqueue(void *token)
{
	u32 val;

	spin_lock(&s3c_idma.lock);
	s3c_idma.token = token;
	spin_unlock(&s3c_idma.lock);

	pr_debug("%s: %x@%x\n", __func__, MAX_LP_BUFF, LP_TXBUFF_ADDR);

	/* Internal DMA Level0 Interrupt Address */
	val = LP_TXBUFF_ADDR;
	writel(val, s3c_idma.regs + S5P_IISADDR3);

	val += s3c_idma.dma_prd;
	writel(val, s3c_idma.regs + S5P_IISADDR0);

	val += s3c_idma.dma_prd;
	writel(val, s3c_idma.regs + S5P_IISADDR1);

	val += s3c_idma.dma_prd;
	writel(val, s3c_idma.regs + S5P_IISADDR2);

	/* Start address0 of I2S internal DMA operation. */
	val = readl(s3c_idma.regs + S5P_IISSTR);
	val = LP_TXBUFF_ADDR;
	writel(val, s3c_idma.regs + S5P_IISSTR);

	/*
	 * Transfer block size for I2S internal DMA.
	 * Should decide transfer size before start dma operation
	 */
	val = readl(s3c_idma.regs + S5P_IISSIZE);
	val &= ~(S5P_IISSIZE_TRNMSK << S5P_IISSIZE_SHIFT);

	val |= ((((s3c_idma.dma_end & 0x1ffff) >> 2) & S5P_IISSIZE_TRNMSK) << S5P_IISSIZE_SHIFT);
	writel(val, s3c_idma.regs + S5P_IISSIZE);

	return 0;
}

static void s3c_idma_setcallbk(void (*cb)(void *, int), unsigned prd)
{
	spin_lock(&s3c_idma.lock);
	s3c_idma.cb = cb;
	s3c_idma.dma_prd = prd;
	spin_unlock(&s3c_idma.lock);

	pr_debug("%s:%d dma_period=%x\n", __func__, __LINE__, s3c_idma.dma_prd);
}

static void s3c_idma_ctrl(int op)
{
	u32 val;

	spin_lock(&s3c_idma.lock);

	val = readl(s3c_idma.regs + S5P_IISAHB);

	switch (op) {
		case LPAM_DMA_START:
			val &= ~S5P_IISAHB_INTENLVL3;
			val |= S5P_IISAHB_INTENLVL0 | S5P_IISAHB_INTENLVL1 | S5P_IISAHB_INTENLVL2 | S5P_IISAHB_DMAEN;
			break;

		case LPAM_DMA_STOP:
			/* Disable LVL Interrupt and DMA Operation */
			val &= ~(S5P_IISAHB_INTENLVL0 | S5P_IISAHB_INTENLVL1 | S5P_IISAHB_INTENLVL2 | S5P_IISAHB_INTENLVL3 | S5P_IISAHB_DMAEN);
			break;

		default:
			spin_unlock(&s3c_idma.lock);
			return;
	}

	writel(val, s3c_idma.regs + S5P_IISAHB);

	spin_unlock(&s3c_idma.lock);
}

static void s3c_idma_done(void *id, int bytes_xfer)
{
	struct snd_pcm_substream *substream = id;
	struct lpam_i2s_pdata *prtd = substream->runtime->private_data;

	pr_debug("%s:%d\n", __func__, __LINE__);

#if 0
	/* For size of transferred data  */
    prtd->pos += bytes_xfer;
    if (prtd->pos >= prtd->end)
            prtd->pos = prtd->start;
#endif
	if (prtd && (prtd->state & ST_RUNNING))
		snd_pcm_period_elapsed(substream);
	else
		pr_debug("%s:%d\n", __func__, __LINE__);
}

static int s3c_idma_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpam_i2s_pdata *prtd = substream->runtime->private_data;
	unsigned long idma_totbytes;

	pr_debug("Entered %s\n", __func__);

	idma_totbytes = params_buffer_bytes(params);
	prtd->end = LP_TXBUFF_ADDR + idma_totbytes;
	prtd->period = params_periods(params);
	s3c_idma.dma_end    = prtd->end;
	s3c_idma.period_val = prtd->period;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	memset(runtime->dma_area, 0, LP_BUFSIZE);

	runtime->dma_bytes = idma_totbytes;

	s3c_idma_setcallbk(s3c_idma_done, params_period_bytes(params));

	prtd->start = runtime->dma_addr;
	prtd->pos = prtd->start;
	prtd->end = prtd->start + idma_totbytes;

	printk("DmaAddr=@%x Total=%lubytes PrdSz=%u #Prds=%u\n",
			prtd->start, idma_totbytes, params_period_bytes(params), prtd->period);

	return 0;
}

static int s3c_idma_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("Entered %s\n", __func__);

	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int s3c_idma_prepare(struct snd_pcm_substream *substream)
{
	struct lpam_i2s_pdata *prtd = substream->runtime->private_data;

	pr_debug("Entered %s\n", __func__);

	prtd->pos = prtd->start;

	/* flush the DMA channel */
	s3c_idma_ctrl(LPAM_DMA_STOP);
	s3c_idma_enqueue((void *)substream);

	return 0;
}

static int s3c_idma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct lpam_i2s_pdata *prtd = substream->runtime->private_data;
	int ret = 0;

	pr_debug("Entered %s\n", __func__);

	spin_lock(&prtd->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		prtd->state |= ST_RUNNING;
		s3c_idma_ctrl(LPAM_DMA_START);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		prtd->state &= ~ST_RUNNING;
		s3c_idma_ctrl(LPAM_DMA_STOP);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&prtd->lock);

	return ret;
}

static snd_pcm_uframes_t
	s3c_idma_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct s3c_dma_params *dma = rtd->dai->cpu_dai->dma_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpam_i2s_pdata *prtd = runtime->private_data;
	dma_addr_t src, dst;
	unsigned long res;

	spin_lock(&prtd->lock);
#if 1
	dst = dma->dma_addr;
	s3c_idma_getpos(&src);
	res = src - runtime->dma_addr;
#else
	res = prtd->pos - prtd->start;
#endif
	spin_unlock(&prtd->lock);


	/* By Jung */
	if (res >= snd_pcm_lib_buffer_bytes(substream)) {
		if (res == snd_pcm_lib_buffer_bytes(substream))
			res = 0;
	}

	return bytes_to_frames(substream->runtime, res);
}

static int s3c_idma_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long size, offset;
	int ret;

	pr_debug("Entered %s\n", __func__);

	/* From snd_pcm_lib_mmap_iomem */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_IO;
	size = vma->vm_end - vma->vm_start;
	offset = vma->vm_pgoff << PAGE_SHIFT;
	ret = io_remap_pfn_range(vma, vma->vm_start,
			(runtime->dma_addr + offset) >> PAGE_SHIFT,
			size, vma->vm_page_prot);

	return ret;
}

static irqreturn_t s3c_iis_irq(int irqno, void *dev_id)
{
	u32 iiscon, iisahb, val;
	u32 iisfic;

	/* dump_i2s(); */
	iisahb  = readl(s3c_idma.regs + S5P_IISAHB);
	iiscon  = readl(s3c_idma.regs + S3C2412_IISCON);

	if (iiscon & S5P_IISCON_FTXSURSTAT) {
		iiscon |= S5P_IISCON_FTXURSTATUS;
		writel(iiscon, s3c_idma.regs + S3C2412_IISCON);
		pr_debug("TX_S underrun interrupt IISCON = 0x%08x\n", readl(s3c_idma.regs + S3C2412_IISCON));
	}

	//TX FIFO underrun
	if (iiscon & S5P_IISCON_FTXURSTATUS) {
		/*
		iiscon &= ~S5P_IISCON_FTXURINTEN;
		iiscon |= S5P_IISCON_FTXURSTATUS;
		writel(iiscon, s3c_idma.regs + S3C2412_IISCON);
		pr_debug("TX_P underrun interrupt IISCON = 0x%08x\n", readl(s3c_idma.regs + S3C2412_IISCON));
		printk("TX_P underrun interrupt IISCON = 0x%08x\n", readl(s3c_idma.regs + S3C2412_IISCON));
		*/
		iisfic = readl(s3c_idma.regs + S3C2412_IISFIC);
		iiscon = readl(s3c_idma.regs + S3C2412_IISCON);

		iisfic |=  S3C2412_IISFIC_TXFLUSH; //flush FIFO try solve DMA freeze
		iiscon |=  S3C2412_IISCON_TXDMA_PAUSE;
		iiscon |=  S3C2412_IISCON_TXCH_PAUSE;
		iiscon &= ~S3C2412_IISCON_TXDMA_ACTIVE;

		writel(iisfic, s3c_idma.regs + S3C2412_IISFIC);
		writel(iiscon, s3c_idma.regs + S3C2412_IISCON);

		udelay(1);
		printk("TX irq\n");

		iisfic = readl(s3c_idma.regs + S3C2412_IISFIC);
		iiscon = readl(s3c_idma.regs + S3C2412_IISCON);
		iisfic &=  ~S3C2412_IISFIC_TXFLUSH; //flush FIFO try solve DMA freeze
		iiscon |= S3C2412_IISCON_TXDMA_ACTIVE | S3C2412_IISCON_IIS_ACTIVE;
		iiscon &= ~S3C2412_IISCON_TXDMA_PAUSE;
		iiscon &= ~S3C2412_IISCON_TXCH_PAUSE;
		iiscon |= (1<<17); //clear the inerrupt

		writel(iisfic, s3c_idma.regs + S3C2412_IISFIC);
		writel(iiscon, s3c_idma.regs + S3C2412_IISCON);
	}

	//RX FIFO full
	if (iiscon & S5P_IISCON_FRXOFSTAT) {
		iisfic = readl(s3c_idma.regs + S3C2412_IISFIC);
		iiscon &= ~S3C2412_IISCON_RXDMA_ACTIVE;
		iiscon |=  S3C2412_IISCON_RXDMA_PAUSE;
		iiscon |=  S3C2412_IISCON_RXCH_PAUSE;
		writel(iisfic, s3c_idma.regs + S3C2412_IISFIC);
		writel(iiscon, s3c_idma.regs + S3C2412_IISCON);
		udelay(1);
		printk("RX irq\n");
		iisfic = readl(s3c_idma.regs + S3C2412_IISFIC);
		iiscon = readl(s3c_idma.regs + S3C2412_IISCON);
		iiscon |= S3C2412_IISCON_RXDMA_ACTIVE | S3C2412_IISCON_IIS_ACTIVE;
		iiscon &= ~S3C2412_IISCON_RXDMA_PAUSE;
		iiscon &= ~S3C2412_IISCON_RXCH_PAUSE;
		iiscon |= S5P_IISCON_FRXOFSTAT; //clear the inerrupt
		writel(iisfic, s3c_idma.regs + S3C2412_IISFIC);
		writel(iiscon, s3c_idma.regs + S3C2412_IISCON);
	}
	/* Check internal DMA level interrupt. */
	if (iisahb & S5P_IISAHB_LVL0INT) {
		val = S5P_IISAHB_CLRLVL0;
		val |= S5P_IISAHB_INTENLVL3;
	}
	else if (iisahb & S5P_IISAHB_LVL1INT)
		val = S5P_IISAHB_CLRLVL1;
	else if (iisahb & S5P_IISAHB_LVL2INT)
		val = S5P_IISAHB_CLRLVL2;
	else if (iisahb & S5P_IISAHB_LVL3INT)
		val = S5P_IISAHB_CLRLVL3;
	else
		val = 0;

	if (val) {
		iisahb |= val;
		writel(iisahb, s3c_idma.regs + S5P_IISAHB);

		/* Finished dma transfer ? */
		if (iisahb & S5P_IISLVLINTMASK) {
			if (s3c_idma.cb) {
				s3c_idma.cb(s3c_idma.token, s3c_idma.dma_prd);
			}
		}
	}

	return IRQ_HANDLED;
}

static int s3c_idma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpam_i2s_pdata *prtd;
	int ret;

	pr_debug("Entered %s\n", __func__);

	snd_soc_set_runtime_hwparams(substream, &s3c_idma_hardware);

	prtd = kzalloc(sizeof(struct lpam_i2s_pdata), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	ret = request_irq(IRQ_I2S0, s3c_iis_irq, 0, "s3c-i2s", prtd);
	if (ret < 0) {
		printk("fail to claim i2s irq , ret = %d\n", ret);
		kfree(prtd);
		return ret;
	}

	spin_lock_init(&prtd->lock);

	runtime->private_data = prtd;

	return 0;
}

static int s3c_idma_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpam_i2s_pdata *prtd = runtime->private_data;

	pr_debug("Entered %s, prtd = %p\n", __func__, prtd);

	free_irq(IRQ_I2S0, prtd);

	if (prtd)
		kfree(prtd);

	return 0;
}

static struct snd_pcm_ops s3c_idma_ops = {
	.open		= s3c_idma_open,
	.close		= s3c_idma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.trigger	= s3c_idma_trigger,
	.pointer	= s3c_idma_pointer,
	.mmap		= s3c_idma_mmap,
	.hw_params	= s3c_idma_hw_params,
	.hw_free	= s3c_idma_hw_free,
	.prepare	= s3c_idma_prepare,
};

static void s3c_idma_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;

	pr_debug("Entered %s\n", __func__);

	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (!substream)
		return;

	buf = &substream->dma_buffer;
	if (!buf->area)
		return;;

	iounmap(buf->area);

	buf->area = NULL;
	buf->addr = 0;
}

static int s3c_idma_preallocate_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	pr_debug("Entered %s\n", __func__);

	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	/* Assign PCM buffer pointers */
	buf->dev.type = SNDRV_DMA_TYPE_CONTINUOUS;
	buf->addr = LP_TXBUFF_ADDR;
	buf->bytes = s3c_idma_hardware.buffer_bytes_max;
	buf->area = (unsigned char *)ioremap(buf->addr, buf->bytes);
	printk("%s:  VA-%p  PA-%X  %ubytes\n",
			__func__, buf->area, buf->addr, buf->bytes);

	return 0;
}

static u64 s3c_idma_mask = DMA_BIT_MASK(32);

static int s3c_idma_pcm_new(struct snd_card *card,
	struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;

	pr_debug("Entered %s\n", __func__);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &s3c_idma_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->playback.channels_min)
		ret = s3c_idma_preallocate_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);

	return ret;
}

struct snd_soc_platform idma_soc_platform = {
	.name = "s5p-lp-audio",
	.pcm_ops = &s3c_idma_ops,
	.pcm_new = s3c_idma_pcm_new,
	.pcm_free = s3c_idma_pcm_free,
};
EXPORT_SYMBOL_GPL(idma_soc_platform);

void s5p_idma_init(void *regs)
{
	spin_lock_init(&s3c_idma.lock);
	s3c_idma.regs = regs;
}

static int __init s5p_soc_platform_init(void)
{
	return snd_soc_register_platform(&idma_soc_platform);
}
module_init(s5p_soc_platform_init);

static void __exit s5p_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&idma_soc_platform);
}
module_exit(s5p_soc_platform_exit);

MODULE_AUTHOR("Jaswinder Singh, jassi.brar@samsung.com");
MODULE_DESCRIPTION("Samsung S5P LP-Audio DMA module");
MODULE_LICENSE("GPL");
