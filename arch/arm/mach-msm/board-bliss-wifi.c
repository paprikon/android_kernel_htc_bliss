/* linux/arch/arm/mach-msm/board-bliss-wifi.c
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wifi_tiwlan.h>
#include <mach/htc_fast_clk.h>

#include "board-bliss.h"
#include "devices.h"
#include "proc_comm.h"
#include <mach/vreg.h>

extern int bliss_wifi_power(int on);

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

/* HTC_CSP_START */
#include <linux/wl12xx.h>

static struct wl12xx_platform_data bliss_wlan_data __initdata = {
       .irq = MSM_GPIO_TO_INT(BLISS_GPIO_WIFI_IRQ),
       .board_ref_clock = WL12XX_REFCLOCK_26,
       .board_tcxo_clock = 1,
};
/* HTC_CSP_END */



static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_12MA), /* CMD */
	PCOM_GPIO_CFG(110, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_12MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_12MA), /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(116, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_12MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_12MA), /* CMD */
	PCOM_GPIO_CFG(110, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_12MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_12MA), /* WLAN IRQ */
};



#define ID_WIFI	0
#define ID_BT	1
#define CLK_OFF	0
#define CLK_ON	1

int bliss_sleep_clk_state_wifi = CLK_OFF;
int bliss_sleep_clk_state_bt = CLK_OFF;

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

int ti_wifi_power(int on)
{
	printk(KERN_INFO "[WLAN]%s: %d\n", __func__, on);

	if (on) {
	   config_gpio_table(wifi_on_gpio_table,
				ARRAY_SIZE(wifi_on_gpio_table));
	   /* control osc */
	   htc_wifi_bt_fast_clk_ctl(CLK_ON, ID_WIFI);
	   msleep(100);
	   gpio_set_value(BLISS_GPIO_WIFI_SHUTDOWN_N, 1); /* WIFI_SHUTDOWN */
	   msleep(15);
	   gpio_set_value(BLISS_GPIO_WIFI_SHUTDOWN_N, 0);
	   msleep(1);
	   gpio_set_value(BLISS_GPIO_WIFI_SHUTDOWN_N, 1);
	   msleep(70);
	} else {
   	  gpio_set_value(BLISS_GPIO_WIFI_SHUTDOWN_N,0);
   	  msleep(1);
	  config_gpio_table(wifi_off_gpio_table,
				ARRAY_SIZE(wifi_off_gpio_table));
	  msleep(10);
      /* control osc */
	  htc_wifi_bt_fast_clk_ctl(CLK_OFF, ID_WIFI);
	}
	msleep(250);
	printk(KERN_INFO "[WLAN]%s: ---\n", __func__);
	return 0;
}
EXPORT_SYMBOL(ti_wifi_power);


int __init bliss_wifi_init(void)
{
	int ret=0;

	printk(KERN_INFO "%s: start\n", __func__);
/* HTC_CSP_START */
    //ruby_wifi_power(1);
	if(wl12xx_set_platform_data(&bliss_wlan_data))
		pr_err("Error setting wl12xx_data\n");
/* HTC_CSP_END */
	return ret;
}
