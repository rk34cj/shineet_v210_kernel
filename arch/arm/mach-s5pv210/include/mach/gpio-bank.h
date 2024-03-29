/* linux/arch/arm/mach-s5pv210/include/mach/gpio-bank.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 - GPIO Bank definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_GPIO_BANK_H
#define __ASM_ARCH_GPIO_BANK_H __FILE__

#define S5P_GPIO_CONMASK(__gpio)	(0xf << ((__gpio) * 4))
#define S5P_GPIO_INPUT(__gpio)		(0x0 << ((__gpio) * 4))
#define S5P_GPIO_OUTPUT(__gpio)		(0x1 << ((__gpio) * 4))

#define S5PV210_GPA0CON			(S5PV210_GPA0_BASE + 0x00)
#define S5PV210_GPA0DAT			(S5PV210_GPA0_BASE + 0x04)
#define S5PV210_GPA0PUD			(S5PV210_GPA0_BASE + 0x08)
#define S5PV210_GPA0DRV			(S5PV210_GPA0_BASE + 0x0c)
#define S5PV210_GPA0CONPDN		(S5PV210_GPA0_BASE + 0x10)
#define S5PV210_GPA0PUDPDN		(S5PV210_GPA0_BASE + 0x14)

#define S5PV210_GPA1CON			(S5PV210_GPA1_BASE + 0x00)
#define S5PV210_GPA1DAT			(S5PV210_GPA1_BASE + 0x04)
#define S5PV210_GPA1PUD			(S5PV210_GPA1_BASE + 0x08)
#define S5PV210_GPA1DRV			(S5PV210_GPA1_BASE + 0x0c)
#define S5PV210_GPA1CONPDN		(S5PV210_GPA1_BASE + 0x10)
#define S5PV210_GPA1PUDPDN		(S5PV210_GPA1_BASE + 0x14)

#define S5PV210_GPBCON			(S5PV210_GPB_BASE + 0x00)
#define S5PV210_GPBDAT			(S5PV210_GPB_BASE + 0x04)
#define S5PV210_GPBPUD			(S5PV210_GPB_BASE + 0x08)
#define S5PV210_GPBDRV			(S5PV210_GPB_BASE + 0x0c)
#define S5PV210_GPBCONPDN		(S5PV210_GPB_BASE + 0x10)
#define S5PV210_GPBPUDPDN		(S5PV210_GPB_BASE + 0x14)

#define S5PV210_GPCCON			(S5PV210_GPC_BASE + 0x00)
#define S5PV210_GPCDAT			(S5PV210_GPC_BASE + 0x04)
#define S5PV210_GPCPUD			(S5PV210_GPC_BASE + 0x08)
#define S5PV210_GPCDRV			(S5PV210_GPC_BASE + 0x0c)
#define S5PV210_GPCCONPDN		(S5PV210_GPC_BASE + 0x10)
#define S5PV210_GPCPUDPDN		(S5PV210_GPC_BASE + 0x14)

#define S5PV210_GPC0CON			(S5PV210_GPC0_BASE + 0x00)
#define S5PV210_GPC0DAT			(S5PV210_GPC0_BASE + 0x04)
#define S5PV210_GPC0PUD			(S5PV210_GPC0_BASE + 0x08)
#define S5PV210_GPC0DRV			(S5PV210_GPC0_BASE + 0x0c)
#define S5PV210_GPC0CONPDN		(S5PV210_GPC0_BASE + 0x10)
#define S5PV210_GPC0PUDPDN		(S5PV210_GPC0_BASE + 0x14)

#define S5PV210_GPC1CON			(S5PV210_GPC1_BASE + 0x00)
#define S5PV210_GPC1DAT			(S5PV210_GPC1_BASE + 0x04)
#define S5PV210_GPC1PUD			(S5PV210_GPC1_BASE + 0x08)
#define S5PV210_GPC1DRV			(S5PV210_GPC1_BASE + 0x0c)
#define S5PV210_GPC1CONPDN		(S5PV210_GPC1_BASE + 0x10)
#define S5PV210_GPC1PUDPDN		(S5PV210_GPC1_BASE + 0x14)

#define S5PV210_GPDCON			(S5PV210_GPD_BASE + 0x00)
#define S5PV210_GPDDAT			(S5PV210_GPD_BASE + 0x04)
#define S5PV210_GPDPUD			(S5PV210_GPD_BASE + 0x08)
#define S5PV210_GPDDRV			(S5PV210_GPD_BASE + 0x0c)
#define S5PV210_GPDCONPDN		(S5PV210_GPD_BASE + 0x10)
#define S5PV210_GPDPUDPDN		(S5PV210_GPD_BASE + 0x14)

#define S5PV210_GPD0CON			(S5PV210_GPD0_BASE + 0x00)
#define S5PV210_GPD0DAT			(S5PV210_GPD0_BASE + 0x04)
#define S5PV210_GPD0PUD			(S5PV210_GPD0_BASE + 0x08)
#define S5PV210_GPD0DRV			(S5PV210_GPD0_BASE + 0x0c)
#define S5PV210_GPD0CONPDN		(S5PV210_GPD0_BASE + 0x10)
#define S5PV210_GPD0PUDPDN		(S5PV210_GPD0_BASE + 0x14)

#define S5PV210_GPD1CON			(S5PV210_GPD1_BASE + 0x00)
#define S5PV210_GPD1DAT			(S5PV210_GPD1_BASE + 0x04)
#define S5PV210_GPD1PUD			(S5PV210_GPD1_BASE + 0x08)
#define S5PV210_GPD1DRV			(S5PV210_GPD1_BASE + 0x0c)
#define S5PV210_GPD1CONPDN		(S5PV210_GPD1_BASE + 0x10)
#define S5PV210_GPD1PUDPDN		(S5PV210_GPD1_BASE + 0x14)

#define S5PV210_GPE0CON			(S5PV210_GPE0_BASE + 0x00)
#define S5PV210_GPE0DAT			(S5PV210_GPE0_BASE + 0x04)
#define S5PV210_GPE0PUD			(S5PV210_GPE0_BASE + 0x08)
#define S5PV210_GPE0DRV			(S5PV210_GPE0_BASE + 0x0c)
#define S5PV210_GPE0CONPDN		(S5PV210_GPE0_BASE + 0x10)
#define S5PV210_GPE0PUDPDN		(S5PV210_GPE0_BASE + 0x14)

#define S5PV210_GPE1CON			(S5PV210_GPE1_BASE + 0x00)
#define S5PV210_GPE1DAT			(S5PV210_GPE1_BASE + 0x04)
#define S5PV210_GPE1PUD			(S5PV210_GPE1_BASE + 0x08)
#define S5PV210_GPE1DRV			(S5PV210_GPE1_BASE + 0x0c)
#define S5PV210_GPE1CONPDN		(S5PV210_GPE1_BASE + 0x10)
#define S5PV210_GPE1PUDPDN		(S5PV210_GPE1_BASE + 0x14)

#define S5PV210_GPF0CON			(S5PV210_GPF0_BASE + 0x00)
#define S5PV210_GPF0DAT			(S5PV210_GPF0_BASE + 0x04)
#define S5PV210_GPF0PUD			(S5PV210_GPF0_BASE + 0x08)
#define S5PV210_GPF0DRV			(S5PV210_GPF0_BASE + 0x0c)
#define S5PV210_GPF0CONPDN		(S5PV210_GPF0_BASE + 0x10)
#define S5PV210_GPF0PUDPDN		(S5PV210_GPF0_BASE + 0x14)

#define S5PV210_GPF1CON			(S5PV210_GPF1_BASE + 0x00)
#define S5PV210_GPF1DAT			(S5PV210_GPF1_BASE + 0x04)
#define S5PV210_GPF1PUD			(S5PV210_GPF1_BASE + 0x08)
#define S5PV210_GPF1DRV			(S5PV210_GPF1_BASE + 0x0c)
#define S5PV210_GPF1CONPDN		(S5PV210_GPF1_BASE + 0x10)
#define S5PV210_GPF1PUDPDN		(S5PV210_GPF1_BASE + 0x14)

#define S5PV210_GPF2CON			(S5PV210_GPF2_BASE + 0x00)
#define S5PV210_GPF2DAT			(S5PV210_GPF2_BASE + 0x04)
#define S5PV210_GPF2PUD			(S5PV210_GPF2_BASE + 0x08)
#define S5PV210_GPF2DRV			(S5PV210_GPF2_BASE + 0x0c)
#define S5PV210_GPF2CONPDN		(S5PV210_GPF2_BASE + 0x10)
#define S5PV210_GPF2PUDPDN		(S5PV210_GPF2_BASE + 0x14)

#define S5PV210_GPF3CON			(S5PV210_GPF3_BASE + 0x00)
#define S5PV210_GPF3DAT			(S5PV210_GPF3_BASE + 0x04)
#define S5PV210_GPF3PUD			(S5PV210_GPF3_BASE + 0x08)
#define S5PV210_GPF3DRV			(S5PV210_GPF3_BASE + 0x0c)
#define S5PV210_GPF3CONPDN		(S5PV210_GPF3_BASE + 0x10)
#define S5PV210_GPF3PUDPDN		(S5PV210_GPF3_BASE + 0x14)

#define S5PV210_GPG0CON			(S5PV210_GPG0_BASE + 0x00)
#define S5PV210_GPG0DAT			(S5PV210_GPG0_BASE + 0x04)
#define S5PV210_GPG0PUD			(S5PV210_GPG0_BASE + 0x08)
#define S5PV210_GPG0DRV			(S5PV210_GPG0_BASE + 0x0c)
#define S5PV210_GPG0CONPDN		(S5PV210_GPG0_BASE + 0x10)
#define S5PV210_GPG0PUDPDN		(S5PV210_GPG0_BASE + 0x14)

#define S5PV210_GPG1CON			(S5PV210_GPG1_BASE + 0x00)
#define S5PV210_GPG1DAT			(S5PV210_GPG1_BASE + 0x04)
#define S5PV210_GPG1PUD			(S5PV210_GPG1_BASE + 0x08)
#define S5PV210_GPG1DRV			(S5PV210_GPG1_BASE + 0x0c)
#define S5PV210_GPG1CONPDN		(S5PV210_GPG1_BASE + 0x10)
#define S5PV210_GPG1PUDPDN		(S5PV210_GPG1_BASE + 0x14)

#define S5PV210_GPG2CON			(S5PV210_GPG2_BASE + 0x00)
#define S5PV210_GPG2DAT			(S5PV210_GPG2_BASE + 0x04)
#define S5PV210_GPG2PUD			(S5PV210_GPG2_BASE + 0x08)
#define S5PV210_GPG2DRV			(S5PV210_GPG2_BASE + 0x0c)
#define S5PV210_GPG2CONPDN		(S5PV210_GPG2_BASE + 0x10)
#define S5PV210_GPG2PUDPDN		(S5PV210_GPG2_BASE + 0x14)

#define S5PV210_GPG3CON			(S5PV210_GPG3_BASE + 0x00)
#define S5PV210_GPG3DAT			(S5PV210_GPG3_BASE + 0x04)
#define S5PV210_GPG3PUD			(S5PV210_GPG3_BASE + 0x08)
#define S5PV210_GPG3DRV			(S5PV210_GPG3_BASE + 0x0c)
#define S5PV210_GPG3CONPDN		(S5PV210_GPG3_BASE + 0x10)
#define S5PV210_GPG3PUDPDN		(S5PV210_GPG3_BASE + 0x14)

#define S5PV210_GPH0CON			(S5PV210_GPH0_BASE + 0x00)
#define S5PV210_GPH0DAT			(S5PV210_GPH0_BASE + 0x04)
#define S5PV210_GPH0PUD			(S5PV210_GPH0_BASE + 0x08)
#define S5PV210_GPH0DRV			(S5PV210_GPH0_BASE + 0x0c)
#define S5PV210_GPH0CONPDN		(S5PV210_GPH0_BASE + 0x10)
#define S5PV210_GPH0PUDPDN		(S5PV210_GPH0_BASE + 0x14)

#define S5PV210_GPH1CON			(S5PV210_GPH1_BASE + 0x00)
#define S5PV210_GPH1DAT			(S5PV210_GPH1_BASE + 0x04)
#define S5PV210_GPH1PUD			(S5PV210_GPH1_BASE + 0x08)
#define S5PV210_GPH1DRV			(S5PV210_GPH1_BASE + 0x0c)
#define S5PV210_GPH1CONPDN		(S5PV210_GPH1_BASE + 0x10)
#define S5PV210_GPH1PUDPDN		(S5PV210_GPH1_BASE + 0x14)

#define S5PV210_GPH2CON			(S5PV210_GPH2_BASE + 0x00)
#define S5PV210_GPH2DAT			(S5PV210_GPH2_BASE + 0x04)
#define S5PV210_GPH2PUD			(S5PV210_GPH2_BASE + 0x08)
#define S5PV210_GPH2DRV			(S5PV210_GPH2_BASE + 0x0c)
#define S5PV210_GPH2CONPDN		(S5PV210_GPH2_BASE + 0x10)
#define S5PV210_GPH2PUDPDN		(S5PV210_GPH2_BASE + 0x14)

#define S5PV210_GPH3CON			(S5PV210_GPH3_BASE + 0x00)
#define S5PV210_GPH3DAT			(S5PV210_GPH3_BASE + 0x04)
#define S5PV210_GPH3PUD			(S5PV210_GPH3_BASE + 0x08)
#define S5PV210_GPH3DRV			(S5PV210_GPH3_BASE + 0x0c)
#define S5PV210_GPH3CONPDN		(S5PV210_GPH3_BASE + 0x10)
#define S5PV210_GPH3PUDPDN		(S5PV210_GPH3_BASE + 0x14)

#define S5PV210_GPICON			(S5PV210_GPI_BASE + 0x00)
#define S5PV210_GPIDAT			(S5PV210_GPI_BASE + 0x04)
#define S5PV210_GPIPUD			(S5PV210_GPI_BASE + 0x08)
#define S5PV210_GPIDRV			(S5PV210_GPI_BASE + 0x0c)
#define S5PV210_GPICONPDN		(S5PV210_GPI_BASE + 0x10)
#define S5PV210_GPIPUDPDN		(S5PV210_GPI_BASE + 0x14)

#define S5PV210_GPJ0CON			(S5PV210_GPJ0_BASE + 0x00)
#define S5PV210_GPJ0DAT			(S5PV210_GPJ0_BASE + 0x04)
#define S5PV210_GPJ0PUD			(S5PV210_GPJ0_BASE + 0x08)
#define S5PV210_GPJ0DRV			(S5PV210_GPJ0_BASE + 0x0c)
#define S5PV210_GPJ0CONPDN		(S5PV210_GPJ0_BASE + 0x10)
#define S5PV210_GPJ0PUDPDN		(S5PV210_GPJ0_BASE + 0x14)

#define S5PV210_GPJ1CON			(S5PV210_GPJ1_BASE + 0x00)
#define S5PV210_GPJ1DAT			(S5PV210_GPJ1_BASE + 0x04)
#define S5PV210_GPJ1PUD			(S5PV210_GPJ1_BASE + 0x08)
#define S5PV210_GPJ1DRV			(S5PV210_GPJ1_BASE + 0x0c)
#define S5PV210_GPJ1CONPDN		(S5PV210_GPJ1_BASE + 0x10)
#define S5PV210_GPJ1PUDPDN		(S5PV210_GPJ1_BASE + 0x14)

#define S5PV210_GPJ2CON			(S5PV210_GPJ2_BASE + 0x00)
#define S5PV210_GPJ2DAT			(S5PV210_GPJ2_BASE + 0x04)
#define S5PV210_GPJ2PUD			(S5PV210_GPJ2_BASE + 0x08)
#define S5PV210_GPJ2DRV			(S5PV210_GPJ2_BASE + 0x0c)
#define S5PV210_GPJ2CONPDN		(S5PV210_GPJ2_BASE + 0x10)
#define S5PV210_GPJ2PUDPDN		(S5PV210_GPJ2_BASE + 0x14)

#define S5PV210_GPJ3CON			(S5PV210_GPJ3_BASE + 0x00)
#define S5PV210_GPJ3DAT			(S5PV210_GPJ3_BASE + 0x04)
#define S5PV210_GPJ3PUD			(S5PV210_GPJ3_BASE + 0x08)
#define S5PV210_GPJ3DRV			(S5PV210_GPJ3_BASE + 0x0c)
#define S5PV210_GPJ3CONPDN		(S5PV210_GPJ3_BASE + 0x10)
#define S5PV210_GPJ3PUDPDN		(S5PV210_GPJ3_BASE + 0x14)

#define S5PV210_GPJ4CON			(S5PV210_GPJ4_BASE + 0x00)
#define S5PV210_GPJ4DAT			(S5PV210_GPJ4_BASE + 0x04)
#define S5PV210_GPJ4PUD			(S5PV210_GPJ4_BASE + 0x08)
#define S5PV210_GPJ4DRV			(S5PV210_GPJ4_BASE + 0x0c)
#define S5PV210_GPJ4CONPDN		(S5PV210_GPJ4_BASE + 0x10)
#define S5PV210_GPJ4PUDPDN		(S5PV210_GPJ4_BASE + 0x14)

#define S5PV210_GPK0CON			(S5PV210_GPK0_BASE + 0x00)
#define S5PV210_GPK0DAT			(S5PV210_GPK0_BASE + 0x04)
#define S5PV210_GPK0PUD			(S5PV210_GPK0_BASE + 0x08)
#define S5PV210_GPK0DRV			(S5PV210_GPK0_BASE + 0x0c)
#define S5PV210_GPK0CONPDN		(S5PV210_GPK0_BASE + 0x10)
#define S5PV210_GPK0PUDPDN		(S5PV210_GPK0_BASE + 0x14)

#define S5PV210_GPK1CON			(S5PV210_GPK1_BASE + 0x00)
#define S5PV210_GPK1DAT			(S5PV210_GPK1_BASE + 0x04)
#define S5PV210_GPK1PUD			(S5PV210_GPK1_BASE + 0x08)
#define S5PV210_GPK1DRV			(S5PV210_GPK1_BASE + 0x0c)
#define S5PV210_GPK1CONPDN		(S5PV210_GPK1_BASE + 0x10)
#define S5PV210_GPK1PUDPDN		(S5PV210_GPK1_BASE + 0x14)

#define S5PV210_GPK2CON			(S5PV210_GPK2_BASE + 0x00)
#define S5PV210_GPK2DAT			(S5PV210_GPK2_BASE + 0x04)
#define S5PV210_GPK2PUD			(S5PV210_GPK2_BASE + 0x08)
#define S5PV210_GPK2DRV			(S5PV210_GPK2_BASE + 0x0c)
#define S5PV210_GPK2CONPDN		(S5PV210_GPK2_BASE + 0x10)
#define S5PV210_GPK2PUDPDN		(S5PV210_GPK2_BASE + 0x14)

#define S5PV210_GPK3CON			(S5PV210_GPK3_BASE + 0x00)
#define S5PV210_GPK3DAT			(S5PV210_GPK3_BASE + 0x04)
#define S5PV210_GPK3PUD			(S5PV210_GPK3_BASE + 0x08)
#define S5PV210_GPK3DRV			(S5PV210_GPK3_BASE + 0x0c)
#define S5PV210_GPK3CONPDN		(S5PV210_GPK3_BASE + 0x10)
#define S5PV210_GPK3PUDPDN		(S5PV210_GPK3_BASE + 0x14)

#define S5PV210_MP00CON			(S5PV210_MP00_BASE + 0x00)
#define S5PV210_MP00DAT			(S5PV210_MP00_BASE + 0x04)
#define S5PV210_MP00PUD			(S5PV210_MP00_BASE + 0x08)
#define S5PV210_MP00DRV			(S5PV210_MP00_BASE + 0x0c)
#define S5PV210_MP00CONPDN		(S5PV210_MP00_BASE + 0x10)
#define S5PV210_MP00PUDPDN		(S5PV210_MP00_BASE + 0x14)

#define S5PV210_MP01CON			(S5PV210_MP01_BASE + 0x00)
#define S5PV210_MP01DAT			(S5PV210_MP01_BASE + 0x04)
#define S5PV210_MP01PUD			(S5PV210_MP01_BASE + 0x08)
#define S5PV210_MP01DRV			(S5PV210_MP01_BASE + 0x0c)
#define S5PV210_MP01CONPDN		(S5PV210_MP01_BASE + 0x10)
#define S5PV210_MP01PUDPDN		(S5PV210_MP01_BASE + 0x14)

#define S5PV210_MP02CON			(S5PV210_MP02_BASE + 0x00)
#define S5PV210_MP02DAT			(S5PV210_MP02_BASE + 0x04)
#define S5PV210_MP02PUD			(S5PV210_MP02_BASE + 0x08)
#define S5PV210_MP02DRV			(S5PV210_MP02_BASE + 0x0c)
#define S5PV210_MP02CONPDN		(S5PV210_MP02_BASE + 0x10)
#define S5PV210_MP02PUDPDN		(S5PV210_MP02_BASE + 0x14)

#define S5PV210_MP03CON			(S5PV210_MP03_BASE + 0x00)
#define S5PV210_MP03DAT			(S5PV210_MP03_BASE + 0x04)
#define S5PV210_MP03PUD			(S5PV210_MP03_BASE + 0x08)
#define S5PV210_MP03DRV			(S5PV210_MP03_BASE + 0x0c)
#define S5PV210_MP03CONPDN		(S5PV210_MP03_BASE + 0x10)
#define S5PV210_MP03PUDPDN		(S5PV210_MP03_BASE + 0x14)

#define S5PV210_MP04CON			(S5PV210_MP04_BASE + 0x00)
#define S5PV210_MP04DAT			(S5PV210_MP04_BASE + 0x04)
#define S5PV210_MP04PUD			(S5PV210_MP04_BASE + 0x08)
#define S5PV210_MP04DRV			(S5PV210_MP04_BASE + 0x0c)
#define S5PV210_MP04CONPDN		(S5PV210_MP04_BASE + 0x10)
#define S5PV210_MP04PUDPDN		(S5PV210_MP04_BASE + 0x14)

#define S5PV210_MP05CON			(S5PV210_MP05_BASE + 0x00)
#define S5PV210_MP05DAT			(S5PV210_MP05_BASE + 0x04)
#define S5PV210_MP05PUD			(S5PV210_MP05_BASE + 0x08)
#define S5PV210_MP05DRV			(S5PV210_MP05_BASE + 0x0c)
#define S5PV210_MP05CONPDN		(S5PV210_MP05_BASE + 0x10)
#define S5PV210_MP05PUDPDN		(S5PV210_MP05_BASE + 0x14)

#define S5PV210_MP06CON			(S5PV210_MP06_BASE + 0x00)
#define S5PV210_MP06DAT			(S5PV210_MP06_BASE + 0x04)
#define S5PV210_MP06PUD			(S5PV210_MP06_BASE + 0x08)
#define S5PV210_MP06DRV			(S5PV210_MP06_BASE + 0x0c)
#define S5PV210_MP06CONPDN		(S5PV210_MP06_BASE + 0x10)
#define S5PV210_MP06PUDPDN		(S5PV210_MP06_BASE + 0x14)

#define S5PV210_MP07CON			(S5PV210_MP07_BASE + 0x00)
#define S5PV210_MP07DAT			(S5PV210_MP07_BASE + 0x04)
#define S5PV210_MP07PUD			(S5PV210_MP07_BASE + 0x08)
#define S5PV210_MP07DRV			(S5PV210_MP07_BASE + 0x0c)
#define S5PV210_MP07CONPDN		(S5PV210_MP07_BASE + 0x10)
#define S5PV210_MP07PUDPDN		(S5PV210_MP07_BASE + 0x14)

#define S5PV210_MP10CON			(S5PV210_MP10_BASE + 0x00)
#define S5PV210_MP10DAT			(S5PV210_MP10_BASE + 0x04)
#define S5PV210_MP10PUD			(S5PV210_MP10_BASE + 0x08)
#define S5PV210_MP10DRV			(S5PV210_MP10_BASE + 0x0c)
#define S5PV210_MP10CONPDN		(S5PV210_MP10_BASE + 0x10)
#define S5PV210_MP10PUDPDN		(S5PV210_MP10_BASE + 0x14)

#define S5PV210_MP11CON			(S5PV210_MP11_BASE + 0x00)
#define S5PV210_MP11DAT			(S5PV210_MP11_BASE + 0x04)
#define S5PV210_MP11PUD			(S5PV210_MP11_BASE + 0x08)
#define S5PV210_MP11DRV			(S5PV210_MP11_BASE + 0x0c)
#define S5PV210_MP11CONPDN		(S5PV210_MP11_BASE + 0x10)
#define S5PV210_MP11PUDPDN		(S5PV210_MP11_BASE + 0x14)

#define S5PV210_MP12CON			(S5PV210_MP12_BASE + 0x00)
#define S5PV210_MP12DAT			(S5PV210_MP12_BASE + 0x04)
#define S5PV210_MP12PUD			(S5PV210_MP12_BASE + 0x08)
#define S5PV210_MP12DRV			(S5PV210_MP12_BASE + 0x0c)
#define S5PV210_MP12CONPDN		(S5PV210_MP12_BASE + 0x10)
#define S5PV210_MP12PUDPDN		(S5PV210_MP12_BASE + 0x14)

#define S5PV210_MP13CON			(S5PV210_MP13_BASE + 0x00)
#define S5PV210_MP13DAT			(S5PV210_MP13_BASE + 0x04)
#define S5PV210_MP13PUD			(S5PV210_MP13_BASE + 0x08)
#define S5PV210_MP13DRV			(S5PV210_MP13_BASE + 0x0c)
#define S5PV210_MP13CONPDN		(S5PV210_MP13_BASE + 0x10)
#define S5PV210_MP13PUDPDN		(S5PV210_MP13_BASE + 0x14)

#define S5PV210_MP14CON			(S5PV210_MP14_BASE + 0x00)
#define S5PV210_MP14DAT			(S5PV210_MP14_BASE + 0x04)
#define S5PV210_MP14PUD			(S5PV210_MP14_BASE + 0x08)
#define S5PV210_MP14DRV			(S5PV210_MP14_BASE + 0x0c)
#define S5PV210_MP14CONPDN		(S5PV210_MP14_BASE + 0x10)
#define S5PV210_MP14PUDPDN		(S5PV210_MP14_BASE + 0x14)

#define S5PV210_MP15CON			(S5PV210_MP15_BASE + 0x00)
#define S5PV210_MP15DAT			(S5PV210_MP15_BASE + 0x04)
#define S5PV210_MP15PUD			(S5PV210_MP15_BASE + 0x08)
#define S5PV210_MP15DRV			(S5PV210_MP15_BASE + 0x0c)
#define S5PV210_MP15CONPDN		(S5PV210_MP15_BASE + 0x10)
#define S5PV210_MP15PUDPDN		(S5PV210_MP15_BASE + 0x14)

#define S5PV210_MP16CON			(S5PV210_MP16_BASE + 0x00)
#define S5PV210_MP16DAT			(S5PV210_MP16_BASE + 0x04)
#define S5PV210_MP16PUD			(S5PV210_MP16_BASE + 0x08)
#define S5PV210_MP16DRV			(S5PV210_MP16_BASE + 0x0c)
#define S5PV210_MP16CONPDN		(S5PV210_MP16_BASE + 0x10)
#define S5PV210_MP16PUDPDN		(S5PV210_MP16_BASE + 0x14)

#define S5PV210_MP17CON			(S5PV210_MP17_BASE + 0x00)
#define S5PV210_MP17DAT			(S5PV210_MP17_BASE + 0x04)
#define S5PV210_MP17PUD			(S5PV210_MP17_BASE + 0x08)
#define S5PV210_MP17DRV			(S5PV210_MP17_BASE + 0x0c)
#define S5PV210_MP17CONPDN		(S5PV210_MP17_BASE + 0x10)
#define S5PV210_MP17PUDPDN		(S5PV210_MP17_BASE + 0x14)

#define S5PV210_MP18CON			(S5PV210_MP18_BASE + 0x00)
#define S5PV210_MP18DAT			(S5PV210_MP18_BASE + 0x04)
#define S5PV210_MP18PUD			(S5PV210_MP18_BASE + 0x08)
#define S5PV210_MP18DRV			(S5PV210_MP18_BASE + 0x0c)
#define S5PV210_MP18CONPDN		(S5PV210_MP18_BASE + 0x10)
#define S5PV210_MP18PUDPDN		(S5PV210_MP18_BASE + 0x14)

#define S5PV210_MP20CON			(S5PV210_MP20_BASE + 0x00)
#define S5PV210_MP20DAT			(S5PV210_MP20_BASE + 0x04)
#define S5PV210_MP20PUD			(S5PV210_MP20_BASE + 0x08)
#define S5PV210_MP20DRV			(S5PV210_MP20_BASE + 0x0c)
#define S5PV210_MP20CONPDN		(S5PV210_MP20_BASE + 0x10)
#define S5PV210_MP20PUDPDN		(S5PV210_MP20_BASE + 0x14)

#define S5PV210_MP21CON			(S5PV210_MP21_BASE + 0x00)
#define S5PV210_MP21DAT			(S5PV210_MP21_BASE + 0x04)
#define S5PV210_MP21PUD			(S5PV210_MP21_BASE + 0x08)
#define S5PV210_MP21DRV			(S5PV210_MP21_BASE + 0x0c)
#define S5PV210_MP21CONPDN		(S5PV210_MP21_BASE + 0x10)
#define S5PV210_MP21PUDPDN		(S5PV210_MP21_BASE + 0x14)

#define S5PV210_MP22CON			(S5PV210_MP22_BASE + 0x00)
#define S5PV210_MP22DAT			(S5PV210_MP22_BASE + 0x04)
#define S5PV210_MP22PUD			(S5PV210_MP22_BASE + 0x08)
#define S5PV210_MP22DRV			(S5PV210_MP22_BASE + 0x0c)
#define S5PV210_MP22CONPDN		(S5PV210_MP22_BASE + 0x10)
#define S5PV210_MP22PUDPDN		(S5PV210_MP22_BASE + 0x14)

#define S5PV210_MP23CON			(S5PV210_MP23_BASE + 0x00)
#define S5PV210_MP23DAT			(S5PV210_MP23_BASE + 0x04)
#define S5PV210_MP23PUD			(S5PV210_MP23_BASE + 0x08)
#define S5PV210_MP23DRV			(S5PV210_MP23_BASE + 0x0c)
#define S5PV210_MP23CONPDN		(S5PV210_MP23_BASE + 0x10)
#define S5PV210_MP23PUDPDN		(S5PV210_MP23_BASE + 0x14)

#define S5PV210_MP24CON			(S5PV210_MP24_BASE + 0x00)
#define S5PV210_MP24DAT			(S5PV210_MP24_BASE + 0x04)
#define S5PV210_MP24PUD			(S5PV210_MP24_BASE + 0x08)
#define S5PV210_MP24DRV			(S5PV210_MP24_BASE + 0x0c)
#define S5PV210_MP24CONPDN		(S5PV210_MP24_BASE + 0x10)
#define S5PV210_MP24PUDPDN		(S5PV210_MP24_BASE + 0x14)

#define S5PV210_MP25CON			(S5PV210_MP25_BASE + 0x00)
#define S5PV210_MP25DAT			(S5PV210_MP25_BASE + 0x04)
#define S5PV210_MP25PUD			(S5PV210_MP25_BASE + 0x08)
#define S5PV210_MP25DRV			(S5PV210_MP25_BASE + 0x0c)
#define S5PV210_MP25CONPDN		(S5PV210_MP25_BASE + 0x10)
#define S5PV210_MP25PUDPDN		(S5PV210_MP25_BASE + 0x14)

#define S5PV210_MP26CON			(S5PV210_MP26_BASE + 0x00)
#define S5PV210_MP26DAT			(S5PV210_MP26_BASE + 0x04)
#define S5PV210_MP26PUD			(S5PV210_MP26_BASE + 0x08)
#define S5PV210_MP26DRV			(S5PV210_MP26_BASE + 0x0c)
#define S5PV210_MP26CONPDN		(S5PV210_MP26_BASE + 0x10)
#define S5PV210_MP26PUDPDN		(S5PV210_MP26_BASE + 0x14)

#define S5PV210_MP27CON			(S5PV210_MP27_BASE + 0x00)
#define S5PV210_MP27DAT			(S5PV210_MP27_BASE + 0x04)
#define S5PV210_MP27PUD			(S5PV210_MP27_BASE + 0x08)
#define S5PV210_MP27DRV			(S5PV210_MP27_BASE + 0x0c)
#define S5PV210_MP27CONPDN		(S5PV210_MP27_BASE + 0x10)
#define S5PV210_MP27PUDPDN		(S5PV210_MP27_BASE + 0x14)

#define S5PV210_MP28CON			(S5PV210_MP28_BASE + 0x00)
#define S5PV210_MP28DAT			(S5PV210_MP28_BASE + 0x04)
#define S5PV210_MP28PUD			(S5PV210_MP28_BASE + 0x08)
#define S5PV210_MP28DRV			(S5PV210_MP28_BASE + 0x0c)
#define S5PV210_MP28CONPDN		(S5PV210_MP28_BASE + 0x10)
#define S5PV210_MP28PUDPDN		(S5PV210_MP28_BASE + 0x14)

#define S5PV210_ETC0PUD			(S5PV210_ETC0_BASE + 0x08)
#define S5PV210_ETC0DRV			(S5PV210_ETC0_BASE + 0x0c)

#define S5PV210_ETC1PUD			(S5PV210_ETC1_BASE + 0x08)
#define S5PV210_ETC1DRV			(S5PV210_ETC1_BASE + 0x0c)

#define S5PV210_ETC2PUD			(S5PV210_ETC2_BASE + 0x08)
#define S5PV210_ETC2DRV			(S5PV210_ETC2_BASE + 0x0c)

#define S5PV210_ETC4PUD			(S5PV210_ETC4_BASE + 0x08)
#define S5PV210_ETC4DRV			(S5PV210_ETC4_BASE + 0x0c)


#define S5PV210_GPD_0_0_TOUT_0  (0x2 << 0)
#define S5PV210_GPD_0_1_TOUT_1  (0x2 << 4)
#define S5PV210_GPD_0_2_TOUT_2  (0x2 << 8)
#define S5PV210_GPD_0_3_TOUT_3  (0x2 << 12)

#define S5PV210_GPH0_0_EXT_INT0_0         (0xf<<0)
#define S5PV210_GPH0_1_EXT_INT1_1         (0xf<<4)
#define S5PV210_GPH0_2_EXT_INT2_2         (0xf<<8)
#define S5PV210_GPH0_3_EXT_INT3_3         (0xf<<12)
#define S5PV210_GPH0_4_EXT_INT4_4         (0xf<<16)
#define S5PV210_GPH0_5_EXT_INT5_5         (0xf<<20)
#define S5PV210_GPH0_6_EXT_INT6_6         (0xf<<24)
#define S5PV210_GPH0_7_EXT_INT7_7         (0xf<<28)

#define S5PV210_GPH0_0_INTPUT         (0<<0)
#define S5PV210_GPH0_1_INTPUT         (0<<4)
#define S5PV210_GPH0_2_INTPUT         (0<<8)
#define S5PV210_GPH0_3_INTPUT         (0<<12)
#define S5PV210_GPH0_4_INTPUT         (0<<16)
#define S5PV210_GPH0_5_INTPUT         (0<<20)
#define S5PV210_GPH0_6_INTPUT         (0<<24)
#define S5PV210_GPH0_7_INTPUT         (0<<28)

#define S5PV210_GPH1_4_HDMI_CEC		(0x4 << 16)
#define S5PV210_GPH1_4_EXT_INT31_4	(0xf << 16)

#define S5PV210_GPH1_5_HDMI_HPD		(0x4 << 20)
#define S5PV210_GPH1_5_EXT_INT31_5	(0xf << 20)


#endif /* __ASM_ARCH_GPIO_BANK_H */
