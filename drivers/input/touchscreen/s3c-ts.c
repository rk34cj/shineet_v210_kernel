/* linux/drivers/input/touchscreen/s3c-ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Potzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c24xx (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2006-09-05: Ryu Euiyoul <ryu.real@gmail.com>
 *      - added power management suspend and resume code
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#include <mach/regs-adc.h>
#include <mach/ts.h>
#include <mach/irqs.h>

#include <linux/proc_fs.h>

#define CONFIG_TOUCHSCREEN_S3C_DEBUG
//#undef CONFIG_TOUCHSCREEN_S3C_DEBUG

#define        X_COOR_MIN      0x40
#define        X_COOR_MAX      0xfbf
#define        X_COOR_FUZZ     32
#define        Y_COOR_MIN      0x18c
#define        Y_COOR_MAX      0xf49
#define        Y_COOR_FUZZ     32

static int  a0_tmp=1;
static int  a1_tmp=0;
static int  a2_tmp=0;
static int  a3_tmp=0;
static int  a4_tmp=1;
static int  a5_tmp=0;
static int  a6_tmp=1;
#define PROC_NAME		"touch"


/* For ts->dev.id.version */
#define S3C_TSVERSION	0x0101

#define WAIT4INT(x)  (((x)<<8) | \
		     S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | S3C_ADCTSC_XP_SEN | \
		     S3C_ADCTSC_XY_PST(3))

#define AUTOPST	     (S3C_ADCTSC_YM_SEN | S3C_ADCTSC_YP_SEN | S3C_ADCTSC_XP_SEN | \
		     S3C_ADCTSC_AUTO_PST | S3C_ADCTSC_XY_PST(0))


#define DEBUG_LVL    KERN_DEBUG

#ifdef CONFIG_HAS_EARLYSUSPEND
void ts_early_suspend(struct early_suspend *h);
void ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

/* Touchscreen default configuration */
struct s3c_ts_mach_info s3c_ts_default_cfg __initdata = 
{
		.delay =		10000,
		.presc = 		49,
		.oversampling_shift = 	2,
		.resol_bit = 		10
};

/*
 * Definitions & global arrays.
 */
static char *s3c_ts_name = "S5P TouchScreen";
static void __iomem 		*ts_base;
static void __iomem 		*ts_base0;
static struct resource		*ts_mem;
static struct resource		*ts_mem0;
static struct resource		*ts_irq;
static struct clk		*ts_clock;
static struct s3c_ts_info 	*ts;

#define TOUCH_SCREEN1 0x1
#define TSSEL 17

int a0,a1,a2,a3,a4,a5,a6;
static void touch_timer_fire(unsigned long data)
{
	unsigned long data0;
	unsigned long data1;
	int updown;
	int x,y;

	a0=a0_tmp;
	a1=a1_tmp;
	a2=a2_tmp;
	a3=a3_tmp;
	a4=a4_tmp;
	a5=a5_tmp;
	a6=a6_tmp;

	data0 = readl(ts_base+S3C_ADCDAT0);
	data1 = readl(ts_base+S3C_ADCDAT1);

	updown = (!(data0 & S3C_ADCDAT0_UPDOWN)) && (!(data1 & S3C_ADCDAT1_UPDOWN));

	if (updown) 
	{
		if (ts->count) 
		{

#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
			{
				struct timeval tv;
				do_gettimeofday(&tv);
				printk(KERN_INFO "T: %06d, X: %03ld, Y: %03ld\n", (int)tv.tv_usec, ts->xp, ts->yp);
			}
#endif

			x=(int) ts->xp;
			y=(int) ts->yp;

			x = ((int)ts->xp - X_COOR_MIN) * 800 / (X_COOR_MAX - X_COOR_MIN);
			y = 480 - ((int)ts->yp - Y_COOR_MIN) * 480 / (Y_COOR_MAX- Y_COOR_MIN);

			ts->xp =x;
			ts->yp =y;


			ts->xp=(long) ((a2+(a0*x)+(a1*y))/a6) ;
			ts->yp=(long) ((a5+(a3*x)+(a4*y))/a6) ;
			
			input_report_abs(ts->dev, ABS_MT_TOUCH_MAJOR,1000);
		
			input_report_abs(ts->dev, ABS_MT_POSITION_X, ts->xp);
			input_report_abs(ts->dev, ABS_MT_POSITION_Y, ts->yp );
			input_report_abs(ts->dev, ABS_X, ts->xp);
			input_report_abs(ts->dev, ABS_Y, ts->yp);
			input_report_abs(ts->dev, ABS_PRESSURE,0xff);
			input_report_abs(ts->dev, ABS_MT_WIDTH_MAJOR, 1);

                           					
			input_mt_sync(ts->dev);

			input_sync(ts->dev);
			//printk("utv210-X = 0x%x,Y = 0x%x ====\n",(ts->xp),(ts->yp));
			ts->xp_old=ts->xp;
			ts->yp_old=ts->yp;


		}

		ts->xp = 0;
		ts->yp = 0;
		ts->count = 0;

		writel(S3C_ADCTSC_PULL_UP_DISABLE | AUTOPST, ts_base+S3C_ADCTSC);
		writel(readl(ts_base+S3C_ADCCON) | S3C_ADCCON_ENABLE_START, ts_base+S3C_ADCCON);
#ifdef CONFIG_TOUCHSCREEN_ADC1
		writel(readl(ts_base0) | (TOUCH_SCREEN1 << TSSEL), ts_base0 + S3C_ADCCON);
#endif
	} 
	else 
	{
		ts->count = 0;
		input_report_abs(ts->dev, ABS_MT_TOUCH_MAJOR, 0);
		//input_report_abs(ts.dev, ABS_MT_WIDTH_MAJOR, 500+press);
		input_report_abs(ts->dev, ABS_MT_POSITION_X, ts->xp_old );
		input_report_abs(ts->dev, ABS_MT_POSITION_Y, ts->yp_old );
		input_report_abs(ts->dev, ABS_X, ts->xp_old);
		input_report_abs(ts->dev, ABS_Y, ts->yp_old);
		input_report_abs(ts->dev, ABS_PRESSURE,0x0);
		input_mt_sync(ts->dev);

		input_sync(ts->dev);
		input_report_key(ts->dev, BTN_TOUCH, 0);
		writel(WAIT4INT(0), ts_base+S3C_ADCTSC);
	}
}

static struct timer_list touch_timer =
		TIMER_INITIALIZER(touch_timer_fire, 0, 0);

static irqreturn_t stylus_updown(int irqno, void *param)
{
	unsigned long data0;
	unsigned long data1;
	int updown;

	data0 = readl(ts_base+S3C_ADCDAT0);
	data1 = readl(ts_base+S3C_ADCDAT1);

	updown = (!(data0 & S3C_ADCDAT0_UPDOWN)) && (!(data1 & S3C_ADCDAT1_UPDOWN));

#ifdef CONFIG_TOUCHSCREEN_ADC1
	writel(readl(ts_base0) | (TOUCH_SCREEN1 << TSSEL), ts_base0 + S3C_ADCCON);
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
       printk(KERN_INFO "   %c\n",	updown ? 'D' : 'U');
#endif

	/* TODO we should never get an interrupt with updown set while
	 * the timer is running, but maybe we ought to verify that the
	 * timer isn't running anyways. */

	if (updown)
		touch_timer_fire(0);

	if (ts->s3c_adc_con == ADC_TYPE_2) {
		__raw_writel(0x0, ts_base+S3C_ADCCLRWK);
		__raw_writel(0x0, ts_base+S3C_ADCCLRINT);
	}

	return IRQ_HANDLED;
}

static irqreturn_t stylus_action(int irqno, void *param)
{
	unsigned long data0;
	unsigned long data1;

	data0 = readl(ts_base+S3C_ADCDAT0);
	data1 = readl(ts_base+S3C_ADCDAT1);

	if (ts->resol_bit == 12)
	{

		//ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT;
		//ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;

		ts->xp = data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT;
		ts->yp = data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;

	}
	else 
	{
#if defined(CONFIG_TOUCHSCREEN_NEW)
		ts->yp += S3C_ADCDAT0_XPDATA_MASK - (data0 & S3C_ADCDAT0_XPDATA_MASK);
		ts->xp += S3C_ADCDAT1_YPDATA_MASK - (data1 & S3C_ADCDAT1_YPDATA_MASK);
#else /* CONFIG_TOUCHSCREEN_NEW */
		ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK;
		ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK;
#endif /* CONFIG_TOUCHSCREEN_NEW */
	}

	ts->count++;

	if (ts->count < (1<<ts->shift))
	{
		writel(S3C_ADCTSC_PULL_UP_DISABLE | AUTOPST, ts_base+S3C_ADCTSC);
		writel(readl(ts_base+S3C_ADCCON) | S3C_ADCCON_ENABLE_START, ts_base+S3C_ADCCON);
	} 
	else 
	{
		mod_timer(&touch_timer, jiffies+1);
		writel(WAIT4INT(1), ts_base+S3C_ADCTSC);
	}

	if (ts->s3c_adc_con == ADC_TYPE_2) 
	{
		__raw_writel(0x0, ts_base+S3C_ADCCLRWK);
		__raw_writel(0x0, ts_base+S3C_ADCCLRINT);
	}
	return IRQ_HANDLED;
}


static struct s3c_ts_mach_info *s3c_ts_get_platdata(struct device *dev)
{
	if (dev->platform_data != NULL)
		return (struct s3c_ts_mach_info *)dev->platform_data;

	return &s3c_ts_default_cfg;
}

/*
 * The functions for inserting/removing us as a module.
 */
static int __init s3c_ts_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev;
	struct input_dev *input_dev;
	struct s3c_ts_mach_info *s3c_ts_cfg;
	int ret, size;
	int irq_flags = 0;

	dev = &pdev->dev;

       printk("s3c_ts_probe-------2010-11-03\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;
	ts_mem = request_mem_region(res->start, size, pdev->name);
	if (ts_mem == NULL) {
		dev_err(dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	ts_base = ioremap(res->start, size);
#ifdef CONFIG_TOUCHSCREEN_ADC1
	//Set TSADCCON0 bit 17
	writel(readl(ts_base) | (TOUCH_SCREEN1 << TSSEL), ts_base);
	ts_base0 = ts_base;
	ts_mem0 = ts_mem;

	//remap ts_base to 0xE1701000
//	iounmap(ts_base);
//	release_resource(ts_mem);
	ts_mem = request_mem_region(res->start + 0x1000, size, pdev->name);
	ts_base = ioremap(res->start + 0x1000, size);
#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
	printk("remap to 0xE1701000. done\n");
#endif
#endif

	if (ts_base == NULL) {
		dev_err(dev, "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_map;
	}

	ts_clock = clk_get(&pdev->dev, "adc");
	if (IS_ERR(ts_clock)) {
		dev_err(dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(ts_clock);
		goto err_clk;
	}

	clk_enable(ts_clock);

	s3c_ts_cfg = s3c_ts_get_platdata(&pdev->dev);
	if ((s3c_ts_cfg->presc&0xff) > 0)
		writel(S3C_ADCCON_PRSCEN | S3C_ADCCON_PRSCVL(s3c_ts_cfg->presc&0xFF),\
				ts_base+S3C_ADCCON);
	else
		writel(0, ts_base+S3C_ADCCON);

	/* Initialise registers */
	if ((s3c_ts_cfg->delay&0xffff) > 0)
		writel(s3c_ts_cfg->delay & 0xffff, ts_base+S3C_ADCDLY);


	if (s3c_ts_cfg->resol_bit == 12) 
	{
		switch (s3c_ts_cfg->s3c_adc_con) 
		{
			case ADC_TYPE_2:
				writel(readl(ts_base+S3C_ADCCON)|S3C_ADCCON_RESSEL_12BIT, ts_base+S3C_ADCCON);
				break;

			case ADC_TYPE_1:
				writel(readl(ts_base+S3C_ADCCON)|S3C_ADCCON_RESSEL_12BIT_1, ts_base+S3C_ADCCON);
				break;

			default:
				dev_err(dev, "Touchscreen over this type of AP isn't supported !\n");
				break;
		}
	}

	writel(WAIT4INT(0), ts_base+S3C_ADCTSC);

	ts = kzalloc(sizeof(struct s3c_ts_info), GFP_KERNEL);

	input_dev = input_allocate_device();
	if (!input_dev) 
	{
		ret = -ENOMEM;
		goto err_alloc;
	}

	ts->dev = input_dev;

	ts->dev->evbit[0] = ts->dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	if (s3c_ts_cfg->resol_bit==12) 
	{
		
	       set_bit(ABS_MT_TOUCH_MAJOR, ts->dev->absbit);
		set_bit(ABS_MT_POSITION_X, ts->dev->absbit);
		set_bit(ABS_MT_POSITION_Y, ts->dev->absbit);
		set_bit(ABS_MT_WIDTH_MAJOR, ts->dev->absbit);

		input_set_abs_params(ts->dev, ABS_MT_TOUCH_MAJOR, 0, 1000, 0, 0);
		//input_set_abs_params(codec_ts_input, ABS_MT_WIDTH_MAJOR, 0, 1000, 0, 0);
		input_set_abs_params(ts->dev, ABS_MT_POSITION_X, 0, 800, 0, 0);
		input_set_abs_params(ts->dev, ABS_MT_POSITION_Y,  0,  480, 0, 0);
		input_set_abs_params(input_dev,  ABS_MT_WIDTH_MAJOR, 0, 1000, 0, 0); 
		//ret = input_register_device(ts->dev);
		input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
		input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
	}
	else 
	{
	
		input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
		input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
	}

	input_set_abs_params(ts->dev, ABS_PRESSURE, 0, 1, 0, 0);

	sprintf(ts->phys, "input(ts)");

	ts->dev->name = s3c_ts_name;
	ts->dev->phys = ts->phys;
	ts->dev->id.bustype = BUS_RS232;
	ts->dev->id.vendor = 0xDEAD;
	ts->dev->id.product = 0xBEEF;
	ts->dev->id.version = S3C_TSVERSION;

	ts->shift = s3c_ts_cfg->oversampling_shift;
	ts->resol_bit = s3c_ts_cfg->resol_bit;
	ts->s3c_adc_con = s3c_ts_cfg->s3c_adc_con;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ts_early_suspend;
	ts->early_suspend.resume =ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	/* For IRQ_PENDUP */
	ts_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (ts_irq == NULL)
	{
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_irq;
	}

	ret = request_irq(ts_irq->start, stylus_updown, irq_flags, "s3c_updown", ts);
	if (ret != 0) 
	{
		dev_err(dev, "s3c_ts.c: Could not allocate ts IRQ_PENDN !\n");
		ret = -EIO;
		goto err_irq;
	}

	/* For IRQ_ADC */
	ts_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (ts_irq == NULL) 
	{
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_irq;
	}

	ret = request_irq(ts_irq->start, stylus_action, irq_flags, "s3c_action", ts);
	if (ret != 0)
	{
		dev_err(dev, "s3c_ts.c: Could not allocate ts IRQ_ADC !\n");
		ret =  -EIO;
		goto err_irq;
	}

	printk(KERN_INFO "%s got loaded successfully : %d bits\n", s3c_ts_name, s3c_ts_cfg->resol_bit);

	/* All went ok, so register to the input system */
	ret = input_register_device(ts->dev);
	if (ret) 
	{
		dev_err(dev, "s3c_ts.c: Could not register input device(touchscreen)!\n");
		ret = -EIO;
		goto fail;
	}

	return 0;

fail:
	free_irq(ts_irq->start, ts->dev);
	free_irq(ts_irq->end, ts->dev);

err_irq:
	input_free_device(input_dev);
	kfree(ts);

err_alloc:
	clk_disable(ts_clock);
	clk_put(ts_clock);

err_clk:
	iounmap(ts_base);

err_map:
	release_resource(ts_mem);
	kfree(ts_mem);

err_req:
	return ret;
}

static int s3c_ts_remove(struct platform_device *dev)
{
	printk(KERN_INFO "s3c_ts_remove() of TS called !\n");

	disable_irq(IRQ_ADC);
	disable_irq(IRQ_PENDN);

	free_irq(IRQ_PENDN, ts->dev);
	free_irq(IRQ_ADC, ts->dev);

	if (ts_clock) {
		clk_disable(ts_clock);
		clk_put(ts_clock);
		ts_clock = NULL;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
     unregister_early_suspend(&ts->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	input_unregister_device(ts->dev);
	iounmap(ts_base);

	return 0;
}

#ifdef CONFIG_PM
static unsigned int adccon, adctsc, adcdly;

static int s3c_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	adccon = readl(ts_base+S3C_ADCCON);
	adctsc = readl(ts_base+S3C_ADCTSC);
	adcdly = readl(ts_base+S3C_ADCDLY);

	disable_irq(IRQ_ADC);
	disable_irq(IRQ_PENDN);

	clk_disable(ts_clock);

	return 0;
}

static int s3c_ts_resume(struct platform_device *pdev)
{
	clk_enable(ts_clock);

	writel(adccon, ts_base+S3C_ADCCON);
	writel(adctsc, ts_base+S3C_ADCTSC);
	writel(adcdly, ts_base+S3C_ADCDLY);
	writel(WAIT4INT(0), ts_base+S3C_ADCTSC);

	enable_irq(IRQ_ADC);
	enable_irq(IRQ_PENDN);

	return 0;
}
#else
#define s3c_ts_suspend NULL
#define s3c_ts_resume  NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void ts_early_suspend(struct early_suspend *h)
{
	struct s3c_ts_info *ts;
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	s3c_ts_suspend(NULL, PMSG_SUSPEND); // platform_device is now used
}

void ts_late_resume(struct early_suspend *h)
{
	struct s3c_ts_info *ts;
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	s3c_ts_resume(NULL); // platform_device is now used
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static struct platform_driver s3c_ts_driver = {
       .probe          = s3c_ts_probe,
       .remove         = s3c_ts_remove,
       .suspend        = s3c_ts_suspend,
       .resume         = s3c_ts_resume,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-ts",
	},
};



//denis_wei add for calibration 2010-09-22
int GetDigitalFromChars(const char *s, int **arrBuf, int arrCount)
{
    if (NULL == s || NULL == arrBuf || arrCount <=0 )
        return 0;
	
    int i = 0;
	char *p = s;
	int num = 0;
	bool neg = 0; 

	while (*p && i < arrCount)
	{
		if ((*p) == ' ' || (*p) == '\0')
		{		
			arrBuf[i] = (neg)?(-(num)):num;	
			neg = 0;
			num = 0;
			i++;
			
			if ((*p) == '\0')
				break;

			p++;			
			continue;
		}

		if ((*p) < 48 || (*p) >57)
		{
			// is a negative number.
			if ((*p) == 45)
			{			
				neg = 1;
				p++;
			}
			else 
			{
				break;
			}
		} 
		else
		{
			num *= 10;
			num += (*p) - 48;
			p++;
		}
		
	}// end while

	if (num != 0)
	   arrBuf[i] = (neg)?(-(num)):num;	

    return 1;
}
static int touch_calibrate_proc_write(struct file *file, const char *buffer, 
                           unsigned long count, void *data) 
{ 
       int i=0,j=0;
	char *buf0,*buf1,*buf2,*buf3,*buf4,*buf5,*buf6;
	static int buf[7] ={0};
	int value; 
	value = 0; 
	//sscanf(buffer, "%d", &value);
       GetDigitalFromChars(buffer, &buf, 7);

	a0_tmp = buf[0];
	a1_tmp = buf[1];
	a2_tmp = buf[2];
	a3_tmp = buf[3];
	a4_tmp = buf[4];
	a5_tmp = buf[5];
	a6_tmp = buf[6];
	//printk("\n touch_calibrate_proc_write\n");

	     
	printk("\n A0:%d,A1:%d,A2:%d,A3:%d,A4:%d,A5:%d,A6:%d\n",a0_tmp,a1_tmp,a2_tmp,a3_tmp,a4_tmp,a5_tmp,a6_tmp);

	
	
    return count; 
} 

static int touch_calibrate_proc_read(char *page, char **start, off_t off, 
			  int count, int *eof, void *data) 
{
	int len;
	int value;
	//value = read_power_item_value(stringToIndex((const char*)data));
	len = sprintf( page, "%d\n", value);
	printk("\n  touch_calibrate_proc_read--\n");
	printk("\n A0:%d,A1:%d,A2:%d,A3:%d,A4:%d,A5:%d,A6:%d\n",a0_tmp,a1_tmp,a2_tmp,a3_tmp,a4_tmp,a5_tmp,a6_tmp);

	
	return len;
}
//add end 2010-09-22
static char banner[] __initdata = KERN_INFO "S5P Touchscreen driver, (c) 2008 Samsung Electronics\n";
extern struct proc_dir_entry proc_root;

static int __init s3c_ts_init(void)
{
	printk(banner);
	//denis_wei add for calibration 2010-09-22
	struct proc_dir_entry *root_entry;
	struct proc_dir_entry *entry;
	int ret;

	ret = 0;
	root_entry = proc_mkdir(PROC_NAME, &proc_root);

	if(root_entry)
	{
			//try = create_proc_entry(s_nods[i], 0666, root_entry);
			entry = create_proc_entry("touch_calibrate" ,0666, root_entry);
			if(entry)
			{
				entry->write_proc = touch_calibrate_proc_write;
				entry->read_proc =  touch_calibrate_proc_read;
				//try->data = (void*)s_nods[i];	
				entry->data = (void*)0;	
			}
	}
	//add end 2010-09-22
	return platform_driver_register(&s3c_ts_driver);
}

static void __exit s3c_ts_exit(void)
{
	platform_driver_unregister(&s3c_ts_driver);
}

module_init(s3c_ts_init);
module_exit(s3c_ts_exit);

MODULE_AUTHOR("Samsung AP");
MODULE_DESCRIPTION("S5P touchscreen driver");
MODULE_LICENSE("GPL");
