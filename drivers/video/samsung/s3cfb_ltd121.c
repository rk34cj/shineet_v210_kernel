/* linux/drivers/video/samsung/s3cfb_lte480wv.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * LTE480 4.8" WVGA Landscape LCD module driver for the SMDK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "s3cfb.h"

static struct s3cfb_lcd lte480wv = {
	.width = 1024,
	.height = 768,
	.bpp = 24,
	.freq = 60,

	.timing = {
		.h_fp = 210,
		.h_bp = 46,
		.h_sw = 20,
		.v_fp = 12,
		.v_fpe = 1,
		.v_bp = 23,
		.v_bpe = 1,
		.v_sw = 10,
	},

	.polarity = {
		.rise_vclk = 0,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	lte480wv.init_ldi = NULL;
	ctrl->lcd = &lte480wv;
}
