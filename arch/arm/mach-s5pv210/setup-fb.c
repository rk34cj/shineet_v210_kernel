/* linux/arch/arm/mach-s5pv210/setup-fb.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Base FIMD controller configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <linux/io.h>
#include <mach/map.h>
#include <mach/pd.h>
#include <mach/gpio-bank.h>

struct platform_device; /* don't need the contents */

int s3cfb_clk_on(struct platform_device *pdev, struct clk **s3cfb_clk)
{
	struct clk *sclk = NULL;
	struct clk *mout_fimd = NULL, *mout_mpll = NULL;
	u32 rate = 0;
	int ret;

	sclk = clk_get(&pdev->dev, "sclk_fimd");
	if (IS_ERR(sclk)) {
		dev_err(&pdev->dev, "failed to get sclk for fimd\n");
		goto err_clk1;
	}

	mout_mpll = clk_get(&pdev->dev, "mout_mpll");
	if (IS_ERR(mout_mpll)) {
		dev_err(&pdev->dev, "failed to get mout_mpll\n");
		goto err_clk1;
	}

	mout_fimd = clk_get(&pdev->dev, "mout_fimd");
	if (IS_ERR(mout_fimd)) {
		dev_err(&pdev->dev,
				"failed to get mout_fimd\n");
		goto err_clk2;
	}

	clk_set_parent(mout_fimd, mout_mpll);

	rate = clk_round_rate(sclk, 133400000);
	dev_dbg(&pdev->dev, "set fimd sclk rate to %d\n", rate);

	if (!rate)
		rate = 133400000;

	clk_set_rate(sclk, rate);
	dev_dbg(&pdev->dev, "set fimd sclk rate to %d\n", rate);

	clk_put(mout_mpll);
	clk_put(mout_fimd);

	ret = s5pv210_pd_enable("fimd_pd");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable fimd power domain\n");
		goto err_clk2;
	}

	clk_enable(sclk);

	*s3cfb_clk = sclk;

	return 0;

err_clk2:
	clk_put(mout_mpll);

err_clk1:
	clk_put(sclk);

	return -EINVAL;
}

int s3cfb_clk_off(struct platform_device *pdev, struct clk **clk)
{
	int ret;

	clk_disable(*clk);
	clk_put(*clk);

	*clk = NULL;
	ret = s5pv210_pd_disable("fimd_pd");
	if (ret < 0)
		dev_err(&pdev->dev, "failed to disable fimd power domain\n");

	return 0;
}

void s3cfb_get_clk_name(char *clk_name)
{
	strcpy(clk_name, "sclk_fimd");
}

#if defined(CONFIG_FB_S3C_LTE480WV) || defined(CONFIG_FB_S3C_UT10GM) || defined(CONFIG_FB_S3C_UT7GM) || defined(CONFIG_FB_URBETTER_SERIAL)
void s3cfb_cfg_gpio(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 4; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
	}

	/* mDNIe SEL: why we shall write 0x2 ? */
	writel(0x2, S5P_MDNIE_SEL);

	/* drive strength to max */
	writel(0xffffffff, S5PV210_GPF0_BASE + 0xc);
	writel(0xffffffff, S5PV210_GPF1_BASE + 0xc);
	writel(0xffffffff, S5PV210_GPF2_BASE + 0xc);
	writel(0x000000ff, S5PV210_GPF3_BASE + 0xc);
}

int s3cfb_backlight_onoff(struct platform_device *pdev, int onoff)
{
#if 0 //urbetter: nobody used.
	int err;

	err = gpio_request(S5PV210_GPD0(3), "GPD0");

	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
			"lcd backlight control\n");
		return err;
	}

	if (onoff) {
		gpio_direction_output(S5PV210_GPD0(3), 1);
		/* 2009.12.28 by icarus : added for PWM backlight */
		s3c_gpio_cfgpin(S5PV210_GPD0(3), S5PV210_GPD_0_3_TOUT_3);

	}
	else {
		gpio_direction_output(S5PV210_GPD0(3), 0);
	}
	gpio_free(S5PV210_GPD0(3));
#endif
	return 0;
}

int s3cfb_backlight_on(struct platform_device *pdev)
{
#if 0 //urbetter
	int err;

	err = gpio_request(S5PV210_GPD0(3), "GPD0");

	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
			"lcd backlight control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPD0(3), 1);

	s3c_gpio_cfgpin(S5PV210_GPD0(3), S5PV210_GPD_0_3_TOUT_3);

	gpio_free(S5PV210_GPD0(3));
#endif
	return 0;
}

int s3cfb_backlight_off(struct platform_device *pdev)
{
#if 0
#if defined(CONFIG_BACKLIGHT_PWM)
	int err;

	err = gpio_request(S5PV210_GPD0(3), "GPD0");

	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
				"lcd backlight control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPD0(3), 0);
	gpio_free(S5PV210_GPD0(3));
#endif
#endif
	return 0;
}

int s3cfb_lcd_on(struct platform_device *pdev)
{
#if 0	//urbetter
	int err;

	err = gpio_request(S5PV210_GPH0(6), "GPH0");
	if (err) {
		printk(KERN_ERR "failed to request GPH0 for "
			"lcd reset control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPH0(6), 1);
	mdelay(100);

	gpio_set_value(S5PV210_GPH0(6), 0);
	mdelay(10);

	gpio_set_value(S5PV210_GPH0(6), 1);
	mdelay(10);

	gpio_free(S5PV210_GPH0(6));
#endif
	return 0;
}

int s3cfb_lcd_off(struct platform_device *pdev)
{
	return 0;
}
#elif defined(CONFIG_FB_S3C_HT101HD1)
void s3cfb_cfg_gpio(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 4; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
	}

	/* mDNIe SEL: why we shall write 0x2 ? */
	writel(0x2, S5P_MDNIE_SEL);

	/* drive strength to max */
	writel(0xffffffff, S5PV210_GPF0_BASE + 0xc);
	writel(0xffffffff, S5PV210_GPF1_BASE + 0xc);
	writel(0xffffffff, S5PV210_GPF2_BASE + 0xc);
	writel(0x000000ff, S5PV210_GPF3_BASE + 0xc);
}

int s3cfb_backlight_onoff(struct platform_device *pdev, int onoff)
{
	return 0;
}

int s3cfb_backlight_on(struct platform_device *pdev)
{
	int err;

	err = gpio_request(S5PV210_GPD0(0), "GPD0");
	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
				"lcd backlight control\n");
		return err;
	}

	err = gpio_request(S5PV210_GPB(2), "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPB for "
				"lcd LED_EN control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPD0(0), 1); /* BL pwm High */

	gpio_direction_output(S5PV210_GPB(2), 1); /* LED_EN (SPI1_MOSI) */

	gpio_free(S5PV210_GPD0(0));
	gpio_free(S5PV210_GPB(2));

	return 0;
}

int s3cfb_backlight_off(struct platform_device *pdev)
{
#if !defined(CONFIG_BACKLIGHT_PWM)
	int err;

	err = gpio_request(S5PV210_GPD0(0), "GPD0");

	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
				"lcd backlight control\n");
		return err;
	}

	err = gpio_request(S5PV210_GPB(2), "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPB for "
				"lcd LED_EN control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPD0(3), 0);
	gpio_direction_output(S5PV210_GPB(2), 0);

	gpio_free(S5PV210_GPD0(0));
	gpio_free(S5PV210_GPB(2));
#endif
	return 0;
}

int s3cfb_lcd_on(struct platform_device *pdev)
{
	int err;

	err = gpio_request(S5PV210_GPH0(1), "GPH0");
	if (err) {
		printk(KERN_ERR "failed to request GPH0 for "
			"lcd reset control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPH0(1), 1);

	gpio_set_value(S5PV210_GPH0(1), 0);

	gpio_set_value(S5PV210_GPH0(1), 1);

	gpio_free(S5PV210_GPH0(1));

	return 0;
}

int s3cfb_lcd_off(struct platform_device *pdev)
{
	return 0;
}
#elif defined(CONFIG_FB_S3C_TL2796)
void s3cfb_cfg_gpio(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 4; i++) {
		s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
	}

	/* mDNIe SEL: why we shall write 0x2 ? */
	writel(0x2, S5P_MDNIE_SEL);

	/* drive strength to max */
	writel(0xffffffff, S5P_VA_GPIO + 0x12c);
	writel(0xffffffff, S5P_VA_GPIO + 0x14c);
	writel(0xffffffff, S5P_VA_GPIO + 0x16c);
	writel(0x000000ff, S5P_VA_GPIO + 0x18c);

	s3c_gpio_cfgpin(S5PV210_GPB(4), S3C_GPIO_SFN(1));
	s3c_gpio_cfgpin(S5PV210_GPB(5), S3C_GPIO_SFN(1));
	s3c_gpio_cfgpin(S5PV210_GPB(6), S3C_GPIO_SFN(1));
	s3c_gpio_cfgpin(S5PV210_GPB(7), S3C_GPIO_SFN(1));

	s3c_gpio_setpull(S5PV210_GPB(4), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5PV210_GPB(5), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5PV210_GPB(6), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5PV210_GPB(7), S3C_GPIO_PULL_NONE);

	gpio_request(S5PV210_GPH0(5), "GPH0");
	gpio_direction_output(S5PV210_GPH0(5), 1);
}

int s3cfb_backlight_on(struct platform_device *pdev)
{

	int err;

	err = gpio_request(S5PV210_GPD0(3), "GPD0");

	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
			"lcd backlight control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPD0(3), 1);
	gpio_free(S5PV210_GPD0(3));

	return 0;
}

int s3cfb_backlight_off(struct platform_device *pdev)
{
	return 0;
}

int s3cfb_backlight_onoff(struct platform_device *pdev, int onoff)
{
	return 0;
}

int s3cfb_lcd_on(struct platform_device *pdev)
{

	int err;

	err = gpio_request(S5PV210_GPH0(6), "GPH0");
	if (err) {
		printk(KERN_ERR "failed to request GPH0 for "
			"lcd reset control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPH0(6), 1);
	mdelay(100);

	gpio_set_value(S5PV210_GPH0(6), 0);
	mdelay(10);

	gpio_set_value(S5PV210_GPH0(6), 1);
	mdelay(10);

	gpio_free(S5PV210_GPH0(6));


	return 0;
}

int s3cfb_lcd_off(struct platform_device *pdev)
{
	return 0;
}

#else
void s3cfb_cfg_gpio(struct platform_device *pdev)
{
	return 0;
}

int s3cfb_backlight_onoff(struct platform_device *pdev, int onoff)
{
	return 0;
}

int s3cfb_backlight_on(struct platform_device *pdev)
{
	return 0;
}

int s3cfb_backlight_off(struct platform_device *pdev)
{
	return 0;
}

int s3cfb_lcd_on(struct platform_device *pdev)
{
	return 0;
}

int s3cfb_lcd_off(struct platform_device *pdev)
{
	return 0;
}
#endif
