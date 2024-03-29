/* drivers/rtc/rtc-s3c.c
 *
 * Copyright (c) 2004,2006 Simtec Electronics
 *	Ben Dooks, <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C/S5P - Internal RTC Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/rtc.h>
//#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/log2.h>

#include <mach/hardware.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/mach/time.h>

#include <plat/regs-rtc.h>

/* I have yet to find an S3C implementation with more than one
 * of these rtc blocks in */

static struct resource *s3c_rtc_mem;

static void __iomem *s3c_rtc_base;
static int s3c_rtc_alarmno = NO_IRQ;
static int s3c_rtc_tickno  = NO_IRQ;
static int s3c_rtc_freq    = 1;

static DEFINE_SPINLOCK(s3c_rtc_pie_lock);
static unsigned int tick_count;

/*
 * bcd2bin/bin2bcd paramater type is char.
 * So, It does not support 3 digits.
 * s5p_bcd2bin/s3c_bin2bcd supports 3 digits.
 */
static unsigned long s5p_bcd2bin(unsigned long val)
{
	return ((val & 0x00f) + ((val & 0x0f0) >> 4) * 10 +
			((val & 0xf00) >> 8) * 100);
}

static unsigned long s3c_bin2bcd(unsigned long val)
{
	return (((val / 100) << 8) + (((val % 100) / 10) << 4)
			+ (val % 10));
}

/* IRQ Handlers */

static irqreturn_t s3c_rtc_alarmirq(int irq, void *id)
{
	struct rtc_device *rdev = id;

	rtc_update_irq(rdev, 1, RTC_AF | RTC_IRQF);

	writeb(S3C_INTP_ALM, s3c_rtc_base + S3C_INTP);

	return IRQ_HANDLED;
}

static irqreturn_t s3c_rtc_tickirq(int irq, void *id)
{
	struct rtc_device *rdev = id;

	rtc_update_irq(rdev, 1, RTC_PF | RTC_IRQF);

	writeb(S3C_INTP_TIC, s3c_rtc_base + S3C_INTP);

	return IRQ_HANDLED;
}

/* Update control registers */
static void s3c_rtc_setaie(int to)
{
	unsigned int tmp;

	pr_debug("%s: aie=%d\n", __func__, to);

	tmp = readb(s3c_rtc_base + S3C2410_RTCALM) & 
		~S3C2410_RTCALM_ALMEN;

	if (to)
		tmp |= S3C2410_RTCALM_ALMEN;

	writeb(tmp, s3c_rtc_base + S3C2410_RTCALM);
}

static int s3c_rtc_setpie(struct device *dev, int enabled)
{
	unsigned int tmp;

	pr_debug("%s: pie=%d\n", __func__, enabled);

	spin_lock_irq(&s3c_rtc_pie_lock);
	tmp = readw(s3c_rtc_base + S3C2410_RTCCON) &
		~S3C_RTCCON_TICEN;

	if (enabled)
		tmp |= S3C_RTCCON_TICEN;

	writew(tmp, s3c_rtc_base + S3C2410_RTCCON);
	spin_unlock_irq(&s3c_rtc_pie_lock);

	return 0;
}

static int s3c_rtc_setfreq(struct device *dev, int freq)
{
	unsigned int tmp;

	if (!is_power_of_2(freq))
		return -EINVAL;

	spin_lock_irq(&s3c_rtc_pie_lock);

	tmp = readw(s3c_rtc_base + S3C2410_RTCCON) &
		(S3C_RTCCON_TICEN | S3C2410_RTCCON_RTCEN);
	writew(tmp, s3c_rtc_base + S3C2410_RTCCON);
	tmp |= (32768 / freq)-1;

	writel(tmp, s3c_rtc_base + S3C2410_TICNT);
	spin_unlock_irq(&s3c_rtc_pie_lock);

	return 0;
}

/* Time read/write */

static int s3c_rtc_gettime(struct device *dev, 
		struct rtc_time *rtc_tm)
{
	unsigned int have_retried = 0;
	void __iomem *base = s3c_rtc_base;
	int year_110;

retry_get_time:
	rtc_tm->tm_sec  = readb(base + S3C2410_RTCSEC);
	rtc_tm->tm_min  = readb(base + S3C2410_RTCMIN);
	rtc_tm->tm_hour = readb(base + S3C2410_RTCHOUR);
	rtc_tm->tm_mday = readb(base + S3C2410_RTCDATE);
	rtc_tm->tm_mon  = readb(base + S3C2410_RTCMON);

#if defined(CONFIG_CPU_S5PV210)
	year_110 = readl(base + S3C2410_RTCYEAR);
	rtc_tm->tm_year = (0x00000fff & year_110);
#else
	rtc_tm->tm_year = readb(base + S3C2410_RTCYEAR);
#endif


	/* the only way to work out wether the system was mid-update
	 * when we read it is to check the second counter, and if it
	 * is zero, then we re-try the entire read
	 */

	if (rtc_tm->tm_sec == 0 && !have_retried) {
		have_retried = 1;
		goto retry_get_time;
	}

	pr_debug("read time %02x.%02x.%02x %02x/%02x/%02x\n",
			rtc_tm->tm_year, rtc_tm->tm_mon,
			rtc_tm->tm_mday,
			rtc_tm->tm_hour, rtc_tm->tm_min,
			rtc_tm->tm_sec);

	rtc_tm->tm_sec = s5p_bcd2bin(rtc_tm->tm_sec);
	rtc_tm->tm_min = s5p_bcd2bin(rtc_tm->tm_min);
	rtc_tm->tm_hour = s5p_bcd2bin(rtc_tm->tm_hour);
	rtc_tm->tm_mday = s5p_bcd2bin(rtc_tm->tm_mday);
	rtc_tm->tm_mon = s5p_bcd2bin(rtc_tm->tm_mon);
	rtc_tm->tm_year = s5p_bcd2bin(rtc_tm->tm_year);

#if defined (CONFIG_CPU_S5PV210)
	/* Base year is in 1600 at s5pc110.
	 * Because, leap year generator.
	 * Linux base year is in 1900.
	 * Both base values are different 300.
	 * So, We add or subtract 300 at SFR value.
	 * If you want more information about LEAP YEAR GRNERATOR.
	 * Please refer RTC manual
	 */
	rtc_tm->tm_year -= 300;
#else
	/* Base year is in 2000 at before s5pc100.
	 * Because, leap year generator.
	 * Linux base year is in 1900.
	 * Both base values are different 100.
	 * So, We add or subtract 100 at SFR value.
	 * If you want more information about LEAP YEAR GRNERATOR.
	 * Please refer RTC manual
	 */
	rtc_tm->tm_year += 100;
#endif


	rtc_tm->tm_mon -= 1;

//urbetter+
	if (rtc_tm->tm_year < (2000 - 1900) || rtc_tm->tm_year > (2030 - 1900)) {
		rtc_tm->tm_hour = 8;
		rtc_tm->tm_mday = 1;
		rtc_tm->tm_mon = 1 - 1;
		rtc_tm->tm_year = 2011 - 1900;
		printk("s3c-rtc: invalid date, set to 2011-1-1 8:00 am\n");
	}
//urbetter+ end
	return 0;
}

static int s3c_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	void __iomem *base = s3c_rtc_base;
	unsigned int tmp;

#if defined (CONFIG_CPU_S5PV210)
	/* Base year is in 1600 at s5pc110.
	 * Because, leap year generator.
	 * Linux base year is in 1900.
	 * Both base values are different 300.
	 * So, We add or subtract 300 at SFR value.
	 * If you want more information about LEAP YEAR GRNERATOR.
	 * Please refer RTC manual
	 */
	int year = tm->tm_year + 300;
#else
	/* Base year is in 2000 at before s5pc100.
	 * Because, leap year generator.
	 * Linux base year is in 1900.
	 * Both base values are different 100.
	 * So, We add or subtract 100 at SFR value.
	 * If you want more information about LEAP YEAR GRNERATOR.
	 * Please refer RTC manual
	 */
	int year = tm->tm_year - 100;
#endif

	pr_debug("set time %02d.%02d.%02d %02d/%02d/%02d\n",
			tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* we get around y2k by simply not supporting it */

#if defined (CONFIG_CPU_S5PV210)
	if (year < 0 || year >= 1000 ) {
		dev_err(dev, "rtc only supports 1000 years\n");
		return -EINVAL;
	}
#else
	if (year < 0 || year >= 100) {
		dev_err(dev, "rtc only supports 100 years\n");
		return -EINVAL;
	}
#endif

	local_irq_disable();

	tmp = readw(base + S3C2410_RTCCON);
	writew(tmp|S3C2410_RTCCON_RTCEN, base+S3C2410_RTCCON);

	writeb(s3c_bin2bcd(tm->tm_sec),  base + S3C2410_RTCSEC);
	writeb(s3c_bin2bcd(tm->tm_min),  base + S3C2410_RTCMIN);
	writeb(s3c_bin2bcd(tm->tm_hour), base + S3C2410_RTCHOUR);
	writeb(s3c_bin2bcd(tm->tm_mday), base + S3C2410_RTCDATE);
	writeb(s3c_bin2bcd(tm->tm_mon + 1), base + S3C2410_RTCMON);

#if defined (CONFIG_CPU_S5PV210)
	year = (0x00000fff & year);
	writel(s3c_bin2bcd(year), base + S3C2410_RTCYEAR);
#else
	writeb(s3c_bin2bcd(year), base + S3C2410_RTCYEAR);
#endif
	writew(tmp, base+S3C2410_RTCCON);

	local_irq_enable();

	return 0;
}

static int s3c_rtc_getalarm(struct device *dev, 
		struct rtc_wkalrm *alrm)
{
	struct rtc_time *alm_tm = &alrm->time;
	void __iomem *base = s3c_rtc_base;
	unsigned int alm_en;

	alm_tm->tm_sec  = readb(base + S3C2410_ALMSEC);
	alm_tm->tm_min  = readb(base + S3C2410_ALMMIN);
	alm_tm->tm_hour = readb(base + S3C2410_ALMHOUR);
	alm_tm->tm_mon  = readb(base + S3C2410_ALMMON);
	alm_tm->tm_mday = readb(base + S3C2410_ALMDATE);
#if defined(CONFIG_CPU_S5PV210)
	alm_tm->tm_year = readl(base + S3C2410_ALMYEAR);
	alm_tm->tm_year = (0x00000fff & alm_tm->tm_year);
#else
	alm_tm->tm_year = readb(base + S3C2410_ALMYEAR);
#endif


	alm_en = readb(base + S3C2410_RTCALM);

	alrm->enabled = (alm_en & S3C2410_RTCALM_ALMEN) ? 1 : 0;

#if defined (CONFIG_CPU_S5PV210)
	pr_debug("read alarm %02x %03x.%02x.%02x %03x/%02x/%02x\n",
			alm_en,
			alm_tm->tm_year, alm_tm->tm_mon,
			alm_tm->tm_mday,
			alm_tm->tm_hour, alm_tm->tm_min,
			alm_tm->tm_sec);
#else
	pr_debug("read alarm %02x %02x.%02x.%02x %02x/%02x/%02x\n",
			alm_en,
			alm_tm->tm_year, alm_tm->tm_mon,
			alm_tm->tm_mday,
			alm_tm->tm_hour, alm_tm->tm_min,
			alm_tm->tm_sec);
#endif


	/* decode the alarm enable field */

	if (alm_en & S3C2410_RTCALM_SECEN)
		alm_tm->tm_sec = s5p_bcd2bin(alm_tm->tm_sec);
	else
		alm_tm->tm_sec = 0xff;

	if (alm_en & S3C2410_RTCALM_MINEN)
		alm_tm->tm_min = s5p_bcd2bin(alm_tm->tm_min);
	else
		alm_tm->tm_min = 0xff;

	if (alm_en & S3C2410_RTCALM_HOUREN)
		alm_tm->tm_hour = s5p_bcd2bin(alm_tm->tm_hour);
	else
		alm_tm->tm_hour = 0xff;

	if (alm_en & S3C2410_RTCALM_DAYEN)
		alm_tm->tm_mday = s5p_bcd2bin(alm_tm->tm_mday);
	else
		alm_tm->tm_mday = 0xff;

	if (alm_en & S3C2410_RTCALM_MONEN) {
		alm_tm->tm_mon = s5p_bcd2bin(alm_tm->tm_mon);
		alm_tm->tm_mon -= 1;
	} else {
		alm_tm->tm_mon = 0xff;
	}

	if (alm_en & S3C2410_RTCALM_YEAREN) {
#if defined (CONFIG_CPU_S5PV210)
		alm_tm->tm_year = s5p_bcd2bin(alm_tm->tm_year);
		alm_tm->tm_year -= 300;
#else
		alm_tm->tm_year = s5p_bcd2bin(alm_tm->tm_year);
		alm_tm->tm_year += 100;
#endif
	} else {
		alm_tm->tm_year = 0xffff;
	}

	return 0;
}

static int s3c_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;
	void __iomem *base = s3c_rtc_base;
	unsigned int alrm_en;

#if defined (CONFIG_CPU_S5PV210)
	/* Base year is in 1600 at s5pc110.
	 * Because, leap year generator.
	 * Linux base year is in 1900.
	 * Both base values are different 300.
	 * So, We add or subtract 300 at SFR value.
	 * If you want more information about LEAP YEAR GRNERATOR.
	 * Please refer RTC manual
	 */
	int year = tm->tm_year + 300;
#else
	/* Base year is in 2000 at before s5pc100.
	 * Because, leap year generator.
	 * Linux base year is in 1900.
	 * Both base values are different 100.
	 * So, We add or subtract 100 at SFR value.
	 * If you want more information about LEAP YEAR GRNERATOR.
	 * Please refer RTC manual
	 */
	int year = tm->tm_year - 100;
#endif

	pr_debug("s3c_rtc_setalarm: %d, %02x/%02x/%02x %02x.%02x.%02x\n",
			alrm->enabled,
			tm->tm_mday & 0xff, tm->tm_mon & 0xff,
			tm->tm_year & 0xff,
			tm->tm_hour & 0xff, tm->tm_min & 0xff,
			tm->tm_sec);


	alrm_en = readb(base + S3C2410_RTCALM) & S3C2410_RTCALM_ALMEN;
	writeb(0x00, base + S3C2410_RTCALM);

	if (tm->tm_sec < 60 && tm->tm_sec >= 0) {
		alrm_en |= S3C2410_RTCALM_SECEN;
		writeb(s3c_bin2bcd(tm->tm_sec), base + S3C2410_ALMSEC);
	}

	if (tm->tm_min < 60 && tm->tm_min >= 0) {
		alrm_en |= S3C2410_RTCALM_MINEN;
		writeb(s3c_bin2bcd(tm->tm_min), base + S3C2410_ALMMIN);
	}

	if (tm->tm_hour < 24 && tm->tm_hour >= 0) {
		alrm_en |= S3C2410_RTCALM_HOUREN;
		writeb(s3c_bin2bcd(tm->tm_hour), base + S3C2410_ALMHOUR);
	}

	if (tm->tm_mday >= 0) {
		alrm_en |= S3C2410_RTCALM_DAYEN;
		writeb(s3c_bin2bcd(tm->tm_mday), base + S3C2410_ALMDATE);
	}

	if (tm->tm_mon < 13 && tm->tm_mon >= 0) {
		alrm_en |= S3C2410_RTCALM_MONEN;
		writeb(s3c_bin2bcd(tm->tm_mon + 1), base + S3C2410_ALMMON);
	}

#if defined (CONFIG_CPU_S5PV210)
	if (year < 1000 && year >= 0) {
		alrm_en |= S3C2410_RTCALM_YEAREN;
		year = (0x00000fff & year);
		writel(s3c_bin2bcd(year), base + S3C2410_ALMYEAR);
	}
#else
	if (year < 100 && year >= 0) {
		alrm_en |= S3C2410_RTCALM_YEAREN;
		writeb(s3c_bin2bcd(year), base + S3C2410_ALMYEAR);
	}
#endif

	pr_debug("setting S3C2410_RTCALM to %08x\n", alrm_en);

	writeb(alrm_en, base + S3C2410_RTCALM);

	s3c_rtc_setaie(alrm->enabled);

	return 0;
}

static int s3c_rtc_ioctl(struct device *dev,
		unsigned int cmd, unsigned long arg)
{
	unsigned int ret = -ENOIOCTLCMD;

	switch (cmd) {
		case RTC_AIE_OFF:
		case RTC_AIE_ON:
			s3c_rtc_setaie((cmd == RTC_AIE_ON) ? 1 : 0);
			ret = 0;
			break;

		case RTC_PIE_OFF:
		case RTC_PIE_ON:
			tick_count = 0;
			s3c_rtc_setpie(dev,(cmd == RTC_PIE_ON) ? 1 : 0);
			ret = 0;
			break;

		case RTC_IRQP_READ:
			ret = put_user(s3c_rtc_freq, 
					(unsigned long __user *)arg);
			break;

		case RTC_IRQP_SET:
			/* check for power of 2 */

			if ((arg & (arg-1)) != 0 || arg < 1) {
				ret = -EINVAL;
				goto exit;
			}

			pr_debug(
			"s3c2410_rtc: setting frequency %ld\n", arg);

			s3c_rtc_setfreq(dev, arg);
			ret = 0;
			break;

		case RTC_UIE_ON:
		case RTC_UIE_OFF:
			ret = -EINVAL;
	}

exit:
	return ret;
}
static int s3c_rtc_proc(struct device *dev, struct seq_file *seq)
{
	unsigned int ticnt = readw(s3c_rtc_base + S3C2410_TICNT);

	seq_printf(seq, "periodic_IRQ\t: %s\n",
			(ticnt & S3C2410_TICNT_ENABLE) ? "yes" : "no");
	return 0;
}

static int s3c_rtc_open(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc_dev = platform_get_drvdata(pdev);
	int ret;

	ret = request_irq(s3c_rtc_alarmno, s3c_rtc_alarmirq,
			IRQF_DISABLED,  "s3c2410-rtc alarm", rtc_dev);

	if (ret) {
		dev_err(dev, "IRQ%d error %d\n", s3c_rtc_alarmno, ret);
		return ret;
	}

	ret = request_irq(s3c_rtc_tickno, s3c_rtc_tickirq,
			IRQF_DISABLED,  "s3c2410-rtc tick", rtc_dev);

	if (ret) {
		dev_err(dev, "IRQ%d error %d\n", s3c_rtc_tickno, ret);
		goto tick_err;
	}

	return ret;

tick_err:
	free_irq(s3c_rtc_alarmno, rtc_dev);
	return ret;
}

static void s3c_rtc_release(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtc_device *rtc_dev = platform_get_drvdata(pdev);

	/* do not clear AIE here, it may be needed for wake */

	s3c_rtc_setpie(dev, 0);
	free_irq(s3c_rtc_alarmno, rtc_dev);
	free_irq(s3c_rtc_tickno, rtc_dev);
}

static const struct rtc_class_ops s3c_rtcops = {
	.open		= s3c_rtc_open,
	.release	= s3c_rtc_release,
	.ioctl		= s3c_rtc_ioctl,
	.read_time	= s3c_rtc_gettime,
	.set_time	= s3c_rtc_settime,
	.read_alarm	= s3c_rtc_getalarm,
	.set_alarm	= s3c_rtc_setalarm,
	.irq_set_freq	= s3c_rtc_setfreq,
	.irq_set_state	= s3c_rtc_setpie,
	.proc	        = s3c_rtc_proc,
};

static void s3c_rtc_enable(struct platform_device *pdev, int en)
{
	void __iomem *base = s3c_rtc_base;
	unsigned int tmp;

	if (s3c_rtc_base == NULL)
		return;

	if (!en) {
		tmp = readw(base + S3C2410_RTCCON);
#ifdef CONFIG_HRT_RTC
		writew(tmp & ~ (S3C2410_RTCCON_RTCEN), base + S3C2410_RTCCON);
#else
		writew(tmp & ~ (S3C2410_RTCCON_RTCEN |
			S3C_RTCCON_TICEN), base + S3C2410_RTCCON);
#endif
	} else {
		/* re-enable the device, and check it is ok */

		if ((readw(base+S3C2410_RTCCON) &
					S3C2410_RTCCON_RTCEN) == 0){
			dev_dbg(&pdev->dev, "rtc disabled, re-enabling\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp|S3C2410_RTCCON_RTCEN,
					base+S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CNTSEL)){
			dev_dbg(&pdev->dev, "removing RTCCON_CNTSEL\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp& ~S3C2410_RTCCON_CNTSEL,
					base+S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CLKRST)){
			dev_dbg(&pdev->dev, "removing RTCCON_CLKRST\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp & ~S3C2410_RTCCON_CLKRST,
					base+S3C2410_RTCCON);
		}
	}
}

static int __devexit s3c_rtc_remove(struct platform_device *dev)
{
	struct rtc_device *rtc = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);
	rtc_device_unregister(rtc);

	s3c_rtc_setpie(&dev->dev, 0);
	s3c_rtc_setaie(0);

	iounmap(s3c_rtc_base);
	release_resource(s3c_rtc_mem);
	kfree(s3c_rtc_mem);

	return 0;
}

static int __devinit s3c_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	struct resource *res;
	unsigned char bcd_tmp,bcd_loop;
	int ret;

	pr_debug("%s: probe=%p\n", __func__, pdev);

	/* find the IRQs */

	s3c_rtc_tickno = platform_get_irq(pdev, 1);
	if (s3c_rtc_tickno < 0) {
		dev_err(&pdev->dev, "no irq for rtc tick\n");
		return -ENOENT;
	}

	s3c_rtc_alarmno = platform_get_irq(pdev, 0);
	if (s3c_rtc_alarmno < 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		return -ENOENT;
	}

	pr_debug("s3c2410_rtc: tick irq %d, alarm irq %d\n",
			s3c_rtc_tickno, s3c_rtc_alarmno);

	/* get the memory region */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}

	s3c_rtc_mem = request_mem_region(res->start,
			res->end-res->start+1,
			pdev->name);

	if (s3c_rtc_mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_nores;
	}

	s3c_rtc_base = ioremap(res->start, res->end - res->start + 1);
	if (s3c_rtc_base == NULL) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -EINVAL;
		goto err_nomap;
	}

	/* check to see if everything is setup correctly */

	s3c_rtc_enable(pdev, 1);

	pr_debug("s3c2410_rtc: RTCCON=%02x\n",
			readw(s3c_rtc_base + S3C2410_RTCCON));

#ifdef CONFIG_PM
	s3c_rtc_setfreq(&pdev->dev, 0);
#else
	s3c_rtc_setfreq(&pdev->dev, 1);
#endif

	device_init_wakeup(&pdev->dev, 1);

	/* register RTC and exit */

	rtc = rtc_device_register("s3c", &pdev->dev, &s3c_rtcops,
			THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);
		goto err_nortc;
	}

	rtc->max_user_freq = S3C_MAX_CNT;

	/* check rtc time */
	for (bcd_loop = S3C2410_RTCSEC ; bcd_loop <= S3C2410_RTCYEAR ; bcd_loop +=0x4)
	{
		bcd_tmp = readb(s3c_rtc_base + bcd_loop);
		if (((bcd_tmp & 0xf) > 0x9) || ((bcd_tmp & 0xf0) > 0x90))
			writeb(0, s3c_rtc_base + bcd_loop);
	}

	platform_set_drvdata(pdev, rtc);
	return 0;

err_nortc:
	s3c_rtc_enable(pdev, 0);
	iounmap(s3c_rtc_base);

err_nomap:
	release_resource(s3c_rtc_mem);

err_nores:
	return ret;
}

#ifdef CONFIG_PM

/* RTC Power management control */

static struct timespec s3c_rtc_delta;
static int ticnt_save;

static int s3c_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_time tm;
	struct timespec time;

	time.tv_nsec = 0;
	/* save TICNT for anyone using periodic interrupts */
	ticnt_save = readb(s3c_rtc_base + S3C2410_TICNT);

	s3c_rtc_gettime(&pdev->dev, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	save_time_delta(&s3c_rtc_delta, &time);

	s3c_rtc_enable(pdev, 0);
	return 0;
}

static int s3c_rtc_resume(struct platform_device *pdev)
{
	struct rtc_time tm;
	struct timespec time;

	time.tv_nsec = 0;

	s3c_rtc_enable(pdev, 1);
	s3c_rtc_gettime(&pdev->dev, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	restore_time_delta(&s3c_rtc_delta, &time);
	writeb(ticnt_save, s3c_rtc_base + S3C2410_TICNT);
	return 0;
}
#else
#define s3c_rtc_suspend NULL
#define s3c_rtc_resume  NULL
#endif

static struct platform_driver s3c2410_rtc_driver = {
	.probe		= s3c_rtc_probe,
	.remove		= __devexit_p(s3c_rtc_remove),
	.suspend	= s3c_rtc_suspend,
	.resume		= s3c_rtc_resume,
	.driver		= {
		.name	= "s3c2410-rtc",
		.owner	= THIS_MODULE,
	},
};

static char __initdata banner[] =
	"S3C24XX RTC, (c) 2004,2006 Simtec Electronics\n";

static int __init s3c_rtc_init(void)
{
	printk(banner);
	return platform_driver_register(&s3c2410_rtc_driver);
}

static void __exit s3c_rtc_exit(void)
{
	platform_driver_unregister(&s3c2410_rtc_driver);
}

module_init(s3c_rtc_init);
module_exit(s3c_rtc_exit);

MODULE_DESCRIPTION("Samsung S3C RTC Driver");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s3c2410-rtc");
