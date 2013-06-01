/* linux/arch/arm/mach-msm/board-bliss.h
 * Copyright (C) 2007-2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_BLISS_H
#define __ARCH_ARM_MACH_MSM_BOARD_BLISS_H

#include <mach/board.h>

#define BLISS_GPIO_UART2_RX 	51
#define BLISS_GPIO_UART2_TX 	52

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x04400000
#define MSM_LINUX_SIZE1		0x0BC00000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x09A00000	/* 159MB -> 157MB -> 154MB*/
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_BASE  	0x29A00000
#define MSM_PMEM_ADSP_SIZE	0x03A00000	/* 64MB -> 55MB -> 58MB */
#define PMEM_KERNEL_EBI1_BASE   0x2D400000
#define PMEM_KERNEL_EBI1_SIZE   0x00900000	/* 7MB -> 9MB*/

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_SF_BASE	0x2DD00000
#define MSM_PMEM_SF_SIZE	0x01B40000

#define MSM_PMEM_ADSP2_BASE      0x2F840000
#define MSM_PMEM_ADSP2_SIZE      0x002C0000

#define MSM_FB_BASE		0x2FB00000
#define MSM_FB_SIZE		0x00500000

#define BLISS_GPIO_WIFI_IRQ             147
#define BLISS_GPIO_WIFI_SHUTDOWN_N       39

#define BLISS_GPIO_KEYPAD_POWER_KEY		46
/*
#define BLISS_GPIO_KEYPAD_MENU_KEY		113
#define BLISS_GPIO_KEYPAD_HOME_KEY		18
#define BLISS_GPIO_KEYPAD_BACK_KEY		19

#define BLISS_GPIO_FLASH_EN			97//PMIC GPIO15
*/
#define BLISS_GPIO_TORCH_EN			98
/*
#define BLISS_GPIO_UP_RESET_N		43//PMIC GPIO36
#define BLISS_GPIO_UP_INT_N			142//PMIC GPIO23
#define BLISS_GPIO_COMPASS_INT		42//PMIC GPIO37
*/
#define BLISS_GPIO_CHG_INT	180

#define BLISS_LAYOUTS			{ \
		{ { 0,  1, 0}, { -1, 0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { -1, 0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}
#define BLISS_MDDI_TE			(30)
#define BLISS_LCD_RSTz		(126)
#define BLISS_LCD_ID1			(128)
#define BLISS_LCD_ID0			(129)
/*
#define BLISS_LCD_PCLK               (90)
#define BLISS_LCD_DE                 (91)
#define BLISS_LCD_VSYNC              (92)
#define BLISS_LCD_HSYNC              (93)
#define BLISS_LCD_G2                 (94)
#define BLISS_LCD_G3                 (95)
#define BLISS_LCD_G4                 (96)
#define BLISS_LCD_G5                 (97)
#define BLISS_LCD_G6                 (98)
#define BLISS_LCD_G7                 (99)
#define BLISS_LCD_B3                 (100)
#define BLISS_LCD_B4                 (101)
#define BLISS_LCD_B5                 (102)
#define BLISS_LCD_B6                 (103)
#define BLISS_LCD_B7                 (104)
#define BLISS_LCD_R3                 (105)
#define BLISS_LCD_R4                 (106)
#define BLISS_LCD_R5                 (107)
#define BLISS_LCD_R6                 (108)
#define BLISS_LCD_R7                 (109)
*/
/* BT */
#define BLISS_GPIO_BT_UART1_RTS      (134)
#define BLISS_GPIO_BT_UART1_CTS      (135)
#define BLISS_GPIO_BT_UART1_RX       (136)
#define BLISS_GPIO_BT_UART1_TX       (137)
#define BLISS_GPIO_BT_PCM_OUT        (138)
#define BLISS_GPIO_BT_PCM_IN         (139)
#define BLISS_GPIO_BT_PCM_SYNC       (140)
#define BLISS_GPIO_BT_PCM_CLK        (141)
#define BLISS_GPIO_BT_EN             (38)

/* USB */
#define BLISS_GPIO_USB_ID_PIN			(49)
#define BLISS_GPIO_USB_ID1_PIN			(145)
#define BLISS_AUDIOz_UART_SW			(95)
#define BLISS_USBz_AUDIO_SW				(96)

/*
#define BLISS_SPI_CS2                (87)
#define BLISS_SPI_DO                 (47)
#define BLISS_SPI_DI                 (48)
#define BLISS_SPI_CLK                (45)
*/
#define BLISS_GPIO_PS_HOLD	(29)

/*#define BLISS_SLIDING_INTZ	(18)//Bliss doesn't have sliding keyboard */

#define BLISS_AUD_CODEC_EN          (36)
#define BLISS_AUD_MICPATH_SEL		(127)

/* 35mm headset */
#define BLISS_GPIO_35MM_HEADSET_DET	(26)

/* EMMC */
#define BLISS_GPIO_EMMC_RST			  (88)

/* Touch */
#define BLISS_GPIO_TP_ATT_N			(20)

/* BT Dock */
#define BLISS_GPIO_BT_DOCK		(44)

/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define BLISS_AUD_MIC_PATH_SEL		PMGPIO(4)
#define BLISS_AUD_HP_PATH_SEL1		PMGPIO(5)
#define BLISS_AUD_HP_PATH_SEL2		PMGPIO(6)
#define BLISS_GPIO_GSENSOR_INT		PMGPIO(7)
#define BLISS_AUD_REMO_PRES		PMGPIO(8)
#define BLISS_AUD_SPK_SD			PMGPIO(12)
#define BLISS_GPIO_FLASH_EN		PMGPIO(15)
#define BLISS_VOL_UP				PMGPIO(16)
#define BLISS_VOL_DN				PMGPIO(17)
#define BLISS_AUD_AMP_EN			PMGPIO(26)
/*#define BLISS_AUD_HANDSET_ENO		PMGPIO(19)//Bliss doesn't have this pin, sync mecha audio.*/
#define BLISS_GPIO_PS_EN			PMGPIO(20)
#define BLISS_TP_RSTz				PMGPIO(21)
#define BLISS_GPIO_PS_INT_N		PMGPIO(22)
#define BLISS_GPIO_UP_INT_N		PMGPIO(23)
#define BLISS_GREEN_LED				PMGPIO(24)
#define BLISS_AMBER_LED			PMGPIO(25)
/*#define BLISS_KEYPAD_LED			PMGPIO(26)//Bliss doesn't have sliding keyboard*/
#define BLISS_GPIO_SDMC_CD_N		PMGPIO(34)
#define BLISS_GPIO_LS_EN			PMGPIO(35)
#define BLISS_GPIO_uP_RST 			PMGPIO(36)
#define BLISS_GPIO_COMPASS_INT_N 	PMGPIO(37)
#define BLISS_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)

/* Camera */
#define CAM1_PWD			(35)
#define CAM1_VCM_PWD	(34)
#define CAM2_PWD			(146)
#define CAM2_RST			(31)
#define CLK_SWITCH		(144)

/*display*/
extern struct platform_device msm_device_mdp;
extern struct platform_device msm_device_mddi0;
extern int panel_type;

int bliss_init_mmc(unsigned int sys_rev);
void __init bliss_audio_init(void);
int bliss_init_keypad(void);
int __init bliss_wifi_init(void);

/*bliss and blissw will share one panel code */
int __init bliss_init_panel(void);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_BLISS_H */
