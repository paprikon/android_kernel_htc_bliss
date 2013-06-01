/* linux/arch/arm/mach-msm/board-bliss-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Author: Jay Tu <jay_tu@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb-7x30.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/msm_panel.h>
#include <mach/panel_id.h>


#include "../board-bliss.h"
#include "../devices.h"
#include "../proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif
#define LCM_GPIO_CFG(gpio, func) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
struct vreg  *vreg_ldo20, *V_LCMIO_1V8;
struct mddi_cmd {
	unsigned char cmd;
	unsigned delay;
	unsigned char *vals;
	unsigned len;
};
static struct clk *axi_clk;
#define prm_size 20
#define LCM_CMD(_cmd, _delay, ...)                              \
{                                                               \
	.cmd = _cmd,                                            \
	.delay = _delay,                                        \
	.vals = (u8 []){__VA_ARGS__},                           \
	.len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}
#define DEFAULT_BRIGHTNESS 255
#define PWM_USER_DEF	 		143
#define PWM_USER_MIN			30
#define PWM_USER_DIM			 9
#define PWM_USER_MAX			255

#define PWM_HITACHI_DEF                        174
#define PWM_HITACHI_MIN                         10
#define PWM_HITACHI_MAX                        255

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
	CABC_STATE,
};
static struct mddi_cmd tear[] = {
	LCM_CMD(0x44, 0, 0x01, 0x00, 0x00, 0x00,
		0x90, 0x00, 0x00, 0x00)
};

static struct mddi_cmd bliss_renesas_cmd[] = {
	LCM_CMD(0x2A, 0, 0x00, 0x00, 0x01, 0xDF),
	LCM_CMD(0x2B, 0, 0x00, 0x00, 0x03, 0x1F),
	LCM_CMD(0x36, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x3A, 0, 0x55, 0x00, 0x00, 0x00),/*set_pixel_format 0x66 for 18bit/pixel, 0x77 for 24bit/pixel*/
};
static struct mddi_cmd bliss_renesas_backlight_blank_cmd[] = {
	LCM_CMD(0xB9, 0, 0x00, 0x00, 0x00, 0x00,
			 0xff, 0x00, 0x00, 0x00,
			 0x04, 0x00, 0x00, 0x00,/*adjust PWM frequency to 10.91k .*/
			 0x08, 0x00, 0x00, 0x00),
};
static struct mddi_cmd gama[] = {
	LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
	LCM_CMD(0xC1, 0, 0x43, 0x00, 0x00, 0x00,
		0x31, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x21, 0x00, 0x00, 0x00,
		0x21, 0x00, 0x00, 0x00,
		0x32, 0x00, 0x00, 0x00,
		0x12, 0x00, 0x00, 0x00,
		0x28, 0x00, 0x00, 0x00,
		0x4A, 0x00, 0x00, 0x00,
		0x1E, 0x00, 0x00, 0x00,
		0xA5, 0x00, 0x00, 0x00,
		0x0F, 0x00, 0x00, 0x00,
		0x58, 0x00, 0x00, 0x00,
		0x21, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00
	),
	LCM_CMD(0xC8, 0, 0x2D, 0x00, 0x00, 0x00,
		0x2F, 0x00, 0x00, 0x00,
		0x31, 0x00, 0x00, 0x00,
		0x36, 0x00, 0x00, 0x00,
		0x3E, 0x00, 0x00, 0x00,
		0x51, 0x00, 0x00, 0x00,
		0x36, 0x00, 0x00, 0x00,
		0x23, 0x00, 0x00, 0x00,
		0x16, 0x00, 0x00, 0x00,
		0x0B, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00,
		0x2D, 0x00, 0x00, 0x00,
		0x2F, 0x00, 0x00, 0x00,
		0x31, 0x00, 0x00, 0x00,
		0x36, 0x00, 0x00, 0x00,
		0x3E, 0x00, 0x00, 0x00,
		0x51, 0x00, 0x00, 0x00,
		0x36, 0x00, 0x00, 0x00,
		0x23, 0x00, 0x00, 0x00,
		0x16, 0x00, 0x00, 0x00,
		0x0B, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00
	),
	LCM_CMD(0xC9, 0, 0x00, 0x00, 0x00, 0x00,
		0x0F, 0x00, 0x00, 0x00,
		0x18, 0x00, 0x00, 0x00,
		0x25, 0x00, 0x00, 0x00,
		0x33, 0x00, 0x00, 0x00,
		0x4D, 0x00, 0x00, 0x00,
		0x38, 0x00, 0x00, 0x00,
		0x25, 0x00, 0x00, 0x00,
		0x18, 0x00, 0x00, 0x00,
		0x11, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x0F, 0x00, 0x00, 0x00,
		0x18, 0x00, 0x00, 0x00,
		0x25, 0x00, 0x00, 0x00,
		0x33, 0x00, 0x00, 0x00,
		0x4D, 0x00, 0x00, 0x00,
		0x38, 0x00, 0x00, 0x00,
		0x25, 0x00, 0x00, 0x00,
		0x18, 0x00, 0x00, 0x00,
		0x11, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00
	),
	LCM_CMD(0xCA, 0, 0x27, 0x00, 0x00, 0x00,
		0x2A, 0x00, 0x00, 0x00,
		0x2E, 0x00, 0x00, 0x00,
		0x34, 0x00, 0x00, 0x00,
		0x3C, 0x00, 0x00, 0x00,
		0x51, 0x00, 0x00, 0x00,
		0x36, 0x00, 0x00, 0x00,
		0x24, 0x00, 0x00, 0x00,
		0x16, 0x00, 0x00, 0x00,
		0x0C, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00,
		0x27, 0x00, 0x00, 0x00,
		0x2A, 0x00, 0x00, 0x00,
		0x2E, 0x00, 0x00, 0x00,
		0x34, 0x00, 0x00, 0x00,
		0x3C, 0x00, 0x00, 0x00,
		0x51, 0x00, 0x00, 0x00,
		0x36, 0x00, 0x00, 0x00,
		0x24, 0x00, 0x00, 0x00,
		0x16, 0x00, 0x00, 0x00,
		0x0C, 0x00, 0x00, 0x00,
		0x02, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00),
	LCM_CMD(0xD5, 0, 0x14, 0x00, 0x00, 0x00,
		0x14, 0x00, 0x00, 0x00
	),
	LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
};

static inline int is_hitachi_panel(void){
	return (panel_type == PANEL_ID_SAG_HITACHI)? 1 : 0;
}

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static void
do_renesas_cmd(struct msm_mddi_client_data *client_data, struct mddi_cmd *cmd_table, ssize_t size)
{
	struct mddi_cmd *pcmd = NULL;
	for (pcmd = cmd_table; pcmd < cmd_table + size; pcmd++) {
		client_data->remote_write_vals(client_data, pcmd->vals,
			pcmd->cmd, pcmd->len);
		if (pcmd->delay)
			hr_msleep(pcmd->delay);
	}
}
enum led_brightness brightness_value = DEFAULT_BRIGHTNESS;

/* use one flag to have better backlight on/off performance */
static int bliss_set_dim = 1;


static int
bliss_shrink_pwm(int brightness, int user_def,
		int user_min, int user_max, int panel_def,
		int panel_min, int panel_max)
{
	if (brightness < PWM_USER_DIM)
		return 0;

	if (brightness < user_min)
		return panel_min;

	if (brightness > user_def) {
		brightness = (panel_max - panel_def) *
			(brightness - user_def) /
			(user_max - user_def) +
			panel_def;
	} else {
			brightness = (panel_def - panel_min) *
			(brightness - user_min) /
			(user_def - user_min) +
			panel_min;
	}

	return brightness;
}

static void
bliss_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;
	struct mddi_cmd *pcmd = bliss_renesas_backlight_blank_cmd;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	shrink_br = bliss_shrink_pwm(val, PWM_USER_DEF,
				PWM_USER_MIN, PWM_USER_MAX, PWM_HITACHI_DEF,
				PWM_HITACHI_MIN, PWM_HITACHI_MAX);
	pcmd->vals[4] = shrink_br;

	if (!client) {
		pr_info("null mddi client");
		return;
	}

	mutex_lock(&cabc.lock);
	if (panel_type == PANEL_ID_BLS_HITACHI) {
		pcmd->vals[4] = shrink_br;
		client->remote_write(client, 0x04, 0xB0);
		client->remote_write_vals(client, pcmd->vals, pcmd->cmd, pcmd->len);
		client->remote_write(client, 0x03, 0xB0);
	} else {
		if (bliss_set_dim == 1) {
			client->remote_write(client, 0x2C, 0x5300);
			/* we dont need set dim again */
			bliss_set_dim = 0;
		}
		client->remote_write(client, 0x00, 0x5500);
		client->remote_write(client, shrink_br, 0x5100);
	}
	brightness_value = val;
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
bliss_get_brightness(struct led_classdev *led_cdev)
{

	return brightness_value;
}


static void
bliss_backlight_switch(struct msm_mddi_client_data *client_data, int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;
		/* LED core uses get_brightness for default value
		  If the physical layer is not ready, we should
		not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		bliss_set_brightness(&cabc.lcd_backlight, val);
		 /*set next backlight value with dim */
		bliss_set_dim = 1;
	} else {
		do_renesas_cmd(client_data, bliss_renesas_backlight_blank_cmd, ARRAY_SIZE(bliss_renesas_backlight_blank_cmd));
		bliss_set_brightness(&cabc.lcd_backlight, 0);
		clear_bit(GATE_ON, &cabc.status);
	}
}

static int
bliss_cabc_switch(int on)
{
	struct msm_mddi_client_data *client = cabc.client_data;

	if (test_bit(CABC_STATE, &cabc.status) == on)
		return 1;

	if (on) {
		printk(KERN_DEBUG "turn on CABC\n");
		set_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x01, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	} else {
		printk(KERN_DEBUG "turn off CABC\n");
		clear_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x00, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	}
	return 1;
}

static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count);
#define CABC_ATTR(name) __ATTR(name, 0644, auto_backlight_show, auto_backlight_store)
static struct device_attribute auto_attr = CABC_ATTR(auto);

static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - 1, "%d\n",
				test_bit(CABC_STATE, &cabc.status));
	return i;
}

static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned long res;

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		printk(KERN_ERR "invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	if (bliss_cabc_switch(!!res))
		count = -EIO;

err_out:
	return count;
}

static int
bliss_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);

	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = bliss_set_brightness;
	cabc.lcd_backlight.brightness_get = bliss_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	err = device_create_file(cabc.lcd_backlight.dev, &auto_attr);
	if (err)
		goto err_out;

	return 0;

err_out:
		device_remove_file(&pdev->dev, &auto_attr);

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

/* ------------------------------------------------------------------- */

#define REG_WAIT (0xffff)
struct nov_regs {
	unsigned reg;
	unsigned val;
};

static struct nov_regs sony_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 125},
	{0xF000, 0x55},
	{0xF001, 0xAA},
	{0xF002, 0x52},
	{0xF003, 0x08},
	{0xF004, 0x01},
	{0xCE00, 0x00},
	{0xCE01, 0x00},
	{0xCE02, 0x00},
	{0xCE03, 0x00},
	{0xCE04, 0x00},
	{0xCE05, 0x00},
	{0xCE06, 0x00},
	{0xFF00, 0xAA},
	{0xFF01, 0x55},
	{0xFF02, 0x25},
	{0xFF03, 0x01},
	{REG_WAIT, 100},
	{0xF813, 0x23},
	{0x2900, 0x00},
	{REG_WAIT, 40},
	{0x4400, 0x02},
	{0x4401, 0x58},
	{0x5500, 0x00},
	{0x5E00, 0x00},
	{0x5300, 0x24},
	{0x3500, 0x00},
};

static int
bliss_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size = 0;
	unsigned reg, val;
	struct nov_regs *init_seq = NULL;
	if (panel_type == PANEL_ID_BLS_HITACHI)
		do_renesas_cmd(client_data, bliss_renesas_cmd, ARRAY_SIZE(bliss_renesas_cmd));
	else {
			if (panel_type == PANEL_ID_BLS_SONY) {
				init_seq = sony_init_seq;
				array_size = ARRAY_SIZE(sony_init_seq);
				for (i = 0; i < array_size; i++) {
					reg = cpu_to_le32(init_seq[i].reg);
					val = cpu_to_le32(init_seq[i].val);
					if (reg == REG_WAIT)
						hr_msleep(val);
					else {
						client_data->remote_write(client_data, val, reg);
						if (reg == 0x1100)
							client_data->send_powerdown(client_data);
					}
				}
			}
	}
	if (axi_clk)
		clk_set_rate(axi_clk, 0);
	return 0;
}

static int
bliss_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	if (panel_type == PANEL_ID_BLS_HITACHI) {
		client_data->auto_hibernate(client_data, 0);
		client_data->remote_write(client_data, 0x0, 0x10);
		hr_msleep(72);
	}
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
bliss_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	client_data->auto_hibernate(client_data, 0);

	if (panel_type == PANEL_ID_BLS_HITACHI) {
		client_data->remote_write(client_data, 0x04, 0xB0);
		client_data->remote_write(client_data, 0x0, 0x28);
		bliss_backlight_switch(client_data, LED_OFF);
		client_data->remote_write(client_data, 0x0, 0xB8);
		client_data->remote_write(client_data, 0x03, 0xB0);
		hr_msleep(72);
	} else {
		client_data->remote_write(client_data, 0x0, 0x5300);
		bliss_backlight_switch(client_data, LED_OFF);
		client_data->remote_write(client_data, 0, 0x2800);
		client_data->remote_write(client_data, 0, 0x1000);
	}

	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
bliss_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s +\n", __func__);

	client_data->auto_hibernate(client_data, 0);

       if (panel_type == PANEL_ID_BLS_HITACHI) {
		client_data->remote_write(client_data, 0x04, 0xB0);
		client_data->remote_write(client_data, 0x0, 0x11);
		hr_msleep(125);
		do_renesas_cmd(client_data, gama, ARRAY_SIZE(gama));
		bliss_backlight_switch(client_data, LED_FULL);
		client_data->remote_write(client_data, 0x0, 0x29);
		do_renesas_cmd(client_data, tear, ARRAY_SIZE(tear));
		client_data->remote_write(client_data, 0x0, 0x35);
		client_data->remote_write(client_data, 0x03, 0xB0);
	} else {
		client_data->remote_write(client_data, 0x00, 0x3600);
		client_data->remote_write(client_data, 0x24, 0x5300);
		client_data->remote_write(client_data, 0x0A, 0x22C0);
		hr_msleep(30);
		bliss_backlight_switch(client_data, LED_FULL);
	}
	client_data->auto_hibernate(client_data, 1);

	B(KERN_DEBUG "%s -\n", __func__);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = bliss_mddi_init,
	.uninit = bliss_mddi_uninit,
	.blank = bliss_panel_blank,
	.unblank = bliss_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 48,
		.height = 80,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
		.vsync_gpio = 30,
	},
};

static struct msm_mddi_bridge_platform_data renesas_client_data = {
	.init = bliss_mddi_init,
	.uninit = bliss_mddi_uninit,
	.blank = bliss_panel_blank,
	.unblank = bliss_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 48,
		.height = 80,
		.output_format = 0,
	},
};

static void
mddi_power(struct msm_mddi_client_data *client_data, int on)
{
	int rc;
	unsigned config;
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);

	if (on) {
		if (axi_clk)
			clk_set_rate(axi_clk, 192000000);

		config = PCOM_GPIO_CFG(BLISS_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(BLISS_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(BLISS_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

		vreg_enable(V_LCMIO_1V8);
		if (panel_type == PANEL_ID_BLS_HITACHI) {
			vreg_enable(vreg_ldo20);
			hr_msleep(1);

			gpio_set_value(BLISS_LCD_RSTz, 1);
			hr_msleep(5);
			gpio_set_value(BLISS_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(BLISS_LCD_RSTz, 1);
			hr_msleep(5);
		} else {
			hr_msleep(3);
			vreg_enable(vreg_ldo20);
			hr_msleep(5);

			gpio_set_value(BLISS_LCD_RSTz, 1);
			hr_msleep(1);
			gpio_set_value(BLISS_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(BLISS_LCD_RSTz, 1);
			hr_msleep(15);
		}
	} else {
		if (panel_type == PANEL_ID_BLS_HITACHI) {
			gpio_set_value(BLISS_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(vreg_ldo20);
		} else {
			hr_msleep(80);
			gpio_set_value(BLISS_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(vreg_ldo20);
		}
		vreg_disable(V_LCMIO_1V8);

		config = PCOM_GPIO_CFG(BLISS_MDDI_TE, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(BLISS_LCD_ID1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(BLISS_LCD_ID0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	}
}

static void
panel_mddi_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	printk(KERN_DEBUG "mddi fixup\n");
	if (panel_type == PANEL_ID_BLS_SONY) {
		*mfr_name = 0xb9f6;
		*product_code = 0x5560;
	} else {
		*mfr_name = 0xb9f6;
		*product_code = 0x1408;
	}
}


static struct msm_mddi_platform_data mddi_pdata = {
	.fixup = panel_mddi_fixup,
	.power_client = mddi_power,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5560),
			.name = "mddi_c_b9f6_5560",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			.product_id = (0xb9f6 << 16 | 0x1408),
			.name = "mddi_renesas_b9f6_61408",
			.id = 0,
			.client_data = &renesas_client_data,
			.clk_rate = 1,
		},
	},
};
static struct platform_driver bliss_backlight_driver = {
	.probe = bliss_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};

static struct msm_mdp_platform_data mdp_pdata_hitachi = {
	.overrides = 0,
#ifdef CONFIG_MDP4_HW_VSYNC
	.xres = 480,
	.yres = 800,
	.back_porch = 2,
	.front_porch = 48,
	.pulse_width = 2,
#endif
};

static struct msm_mdp_platform_data mdp_pdata_sony = {
	.overrides = MSM_MDP_PANEL_FLIP_UD | MSM_MDP_PANEL_FLIP_LR,
	.color_format = MSM_MDP_OUT_IF_FMT_RGB888,
#ifdef CONFIG_MDP4_HW_VSYNC
	.xres = 480,
	.yres = 800,
	.back_porch = 4,
	.front_porch = 2,
	.pulse_width = 4,
#endif
};

int __init bliss_init_panel(void)
{
	int rc = 0;

	B(KERN_INFO "%s(%d): enter. panel_type 0x%08x\n", __func__, __LINE__, panel_type);

	if (panel_type == PANEL_ID_BLS_HITACHI)
		msm_device_mdp.dev.platform_data = &mdp_pdata_hitachi;
	else
		msm_device_mdp.dev.platform_data = &mdp_pdata_sony;



	/* lcd panel power */
	/* 2.85V -- LDO20 */
	vreg_ldo20 = vreg_get(NULL, "gp13");

	if (IS_ERR(vreg_ldo20)) {
		pr_err("%s: gp13 vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_ldo20));
		return -1;
	}

	rc = vreg_set_level(vreg_ldo20, 2850);
	if (rc) {
		pr_err("%s: vreg LDO20 set level failed (%d)\n",
			__func__, rc);
		return -1;
	}

	V_LCMIO_1V8 = vreg_get(NULL, "lvsw0");

	if (IS_ERR(V_LCMIO_1V8)) {
		pr_err("%s: LCMIO_1V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_1V8));
		return -1;
	}
	/* 1.80V -- LCMIO */
	rc = vreg_set_level(V_LCMIO_1V8, 1800);
	if (rc) {
		pr_err("%s: vreg LCMIO_1V8 set level failed (%d)\n",
			__func__, rc);
		return -1;
	}

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	if (panel_type == PANEL_ID_BLS_HITACHI)
		mddi_pdata.clk_rate = 384000000;
	else
		mddi_pdata.clk_rate = 384000000;

	mddi_pdata.type = MSM_MDP_MDDI_TYPE_II;

	axi_clk = clk_get(NULL, "ebi1_mddi_clk");
	if (IS_ERR(axi_clk)) {
		pr_err("%s: failed to get axi clock\n", __func__);
		return PTR_ERR(axi_clk);
	}

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);

	if (panel_type == PANEL_ID_BLS_HITACHI)
		bliss_backlight_driver.driver.name = "renesas_backlight";

	if (rc)
		return rc;
	rc = platform_driver_register(&bliss_backlight_driver);
	if (rc)
		return rc;

	return 0;
}
