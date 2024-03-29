/* linux/drivers/input/keyboard/s3c-keypad.h
 *
 * Driver header for Samsung SoC keypad.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef _S3C_KEYPAD_H_
#define _S3C_KEYPAD_H_

static void __iomem *key_base;

#if defined(CONFIG_KEYPAD_S3C_MSM)
#define KEYPAD_COLUMNS 	8
#define KEYPAD_ROWS		14
#define MAX_KEYPAD_NR	112 /* 8*14 */
#define MAX_KEYMASK_NR	56
#else
#define KEYPAD_COLUMNS  8
#define KEYPAD_ROWS 8
#define MAX_KEYPAD_NR   64  /* 8*8 */
#define MAX_KEYMASK_NR  32
#endif
#endif

#if defined(CONFIG_KEYPAD_S3C_MSM)
//eleven add for keybd 20110626

int keypad_keycode[] = {
		KEY_1,KEY_FN,KEY_2,KEY_4,KEY_3,KEY_7,KEY_F10,KEY_8,KEY_9,KEY_0,KEY_O,1,1,1,
		KEY_ESC,KEY_F5,KEY_BACKSLASH,KEY_G,KEY_F4,KEY_H,2,KEY_F6,3,KEY_COMMA,KEY_DOWN,1,1,1,
		KEY_TAB,KEY_LEFTSHIFT,KEY_CAPSLOCK,KEY_T,KEY_F3,KEY_Y,KEY_END,KEY_RIGHTBRACE,KEY_F7,KEY_LEFTBRACE,KEY_SPACE,1,1,1,
		KEY_Q,KEY_PAUSE,KEY_W,KEY_R,KEY_E,KEY_U,KEY_SYSRQ,KEY_I,KEY_O,KEY_P,KEY_NUMLOCK,1,1,1,
		KEY_Z,KEY_BACKSPACE,KEY_X,KEY_V,KEY_C,KEY_M,4,KEY_COMMA,KEY_DOT,KEY_BACKSLASH,KEY_LEFT,1,1,1,
		4,5,KEY_ENTER,KEY_B,KEY_F9,KEY_N,KEY_LEFTALT,5,KEY_REWIND,KEY_SLASH,KEY_RIGHT,1,1,1,
		KEY_HOME,KEY_F1,KEY_F2,KEY_5,KEY_LEFTCTRL,KEY_6,KEY_INSERT,KEY_EQUAL,KEY_F8,KEY_MINUS,KEY_DELETE,1,1,1,
		KEY_A,KEY_RIGHTSHIFT,KEY_S,KEY_F,KEY_D,KEY_J,5,KEY_K,KEY_L,KEY_SEMICOLON,KEY_UP,1,1,1,
};
#if 0
int keypad_keycode[] = {
	KEY_UP,KEY_RIGHT,KEY_BACK,KEY_ENTER,5,6,7,8,
	KEY_LEFT,KEY_DOWN,KEY_END,KEY_MENU,13,14,15,16,
		17,18,19,20,21,22,23,24,
		25,26,27,28,29,30,31,32,
		33,34,35,36,37,38,39,40,
		41,42,43,44,45,46,47,48,
		49,50,51,52,53,54,55,56,
		57,58,59,60,61,62,63,64
};
#endif
#else
#ifdef CONFIG_ANDROID
#if 0
int keypad_keycode[] = {
		KEY_1,KEY_FN,KEY_2,KEY_4,KEY_3,KEY_7,KEY_F10,KEY_8,KEY_9,KEY_0,KEY_O,1,1,1,
		KEY_ESC,KEY_F5,KEY_BACKSLASH,KEY_G,KEY_F4,KEY_H,2,KEY_F6,3,KEY_COMMA,KEY_DOWN,1,1,1,
		KEY_TAB,KEY_LEFTSHIFT,KEY_CAPSLOCK,KEY_T,KEY_F3,KEY_Y,KEY_END,KEY_RIGHTBRACE,KEY_F7,KEY_LEFTBRACE,KEY_SPACE,1,1,1,
		KEY_Q,KEY_PAUSE,KEY_W,KEY_R,KEY_E,KEY_U,KEY_SYSRQ,KEY_I,KEY_O,KEY_P,KEY_NUMLOCK,1,1,1,
		KEY_Z,KEY_BACKSPACE,KEY_X,KEY_V,KEY_C,KEY_M,4,KEY_COMMA,KEY_DOT,KEY_BACKSLASH,KEY_LEFT,1,1,1,
		4,5,KEY_ENTER,KEY_B,KEY_F9,KEY_N,KEY_LEFTALT,5,KEY_REWIND,KEY_SLASH,KEY_RIGHT,1,1,1,
		KEY_SPACE,KEY_F1,KEY_F2,KEY_5,KEY_LEFTCTRL,KEY_6,KEY_INSERT,KEY_EQUAL,KEY_F8,KEY_MINUS,KEY_DELETE,1,1,1,
		KEY_A,KEY_RIGHTSHIFT,KEY_S,KEY_F,KEY_D,KEY_J,5,KEY_K,KEY_L,KEY_SEMICOLON,KEY_UP,1,1,1,
};
#endif
int keypad_keycode[] = {
	KEY_UP,KEY_RIGHT,KEY_BACK,KEY_ENTER,5,6,7,8,
	KEY_LEFT,KEY_DOWN,KEY_END,KEY_MENU,13,14,15,16,
		17,18,19,20,21,22,23,24,
		25,26,27,28,29,30,31,32,
		33,34,35,36,37,38,39,40,
		41,42,43,44,45,46,47,48,
		49,50,51,52,53,54,55,56,
		57,58,59,60,61,62,63,64
};
#else
int keypad_keycode[] = {
	1,				2,		KEY_1,	KEY_Q,	KEY_A,	6,			7,			KEY_LEFT,
	9,				10,		KEY_2,	KEY_W,	KEY_S,	KEY_Z,		KEY_RIGHT,	16,
	17,				18,		KEY_3,	KEY_E,	KEY_D,	KEY_X,		23,			KEY_UP,
	25,				26,		KEY_4,	KEY_R,	KEY_F,	KEY_C,		31,			32,
	33,				KEY_O,	KEY_5,	KEY_T,	KEY_G,	KEY_V,		KEY_DOWN,	KEY_BACKSPACE,
	KEY_P,			KEY_0,	KEY_6,	KEY_Y,	KEY_H,	KEY_SPACE,	47,			48,
	KEY_M,			KEY_L,	KEY_7,	KEY_U,	KEY_J,	KEY_N,		55,			KEY_ENTER,
	KEY_LEFTSHIFT,	KEY_9,	KEY_8,	KEY_I,	KEY_K,	KEY_B,		63,			KEY_COMMA,
};
#endif
#endif

#if defined(CONFIG_CPU_S3C6410)
#define KEYPAD_DELAY		(50)
#elif defined(CONFIG_CPU_S5PC100)
#define KEYPAD_DELAY		(600)
#elif defined(CONFIG_CPU_S5PV210)
#define KEYPAD_DELAY		(900)
#elif defined(CONFIG_CPU_S5P6442)
#define KEYPAD_DELAY		(50)
#endif

#define	KEYIFCOL_CLEAR		(readl(key_base+S3C_KEYIFCOL) & ~0xffff)
#define	KEYIFCON_CLEAR		(readl(key_base+S3C_KEYIFCON) & ~0x1f)
#define	KEYIFFC_DIV			(readl(key_base+S3C_KEYIFFC) | 0x1)

struct s3c_keypad {
	struct input_dev *dev;
	int nr_rows;
	int no_cols;
	int total_keys;
	int keycodes[MAX_KEYPAD_NR];
};

#if defined(CONFIG_KEYPAD_S3C_MSM)
extern void s3c_setup_keypad_cfg_gpio(void);
#else
extern void s3c_setup_keypad_cfg_gpio(int rows, int columns);
#endif				/* _S3C_KEYIF_H_ */
