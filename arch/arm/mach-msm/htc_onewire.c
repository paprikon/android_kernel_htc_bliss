/* arch/arm/mach-msm/htc_onewire.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <mach/msm_fast_timer.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <mach/htc_onewire.h>

#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/switch.h>

static struct switch_dev dock_switch = {
	.name = "bt_dock",
};

static int vbus_present;
static int dock_charging_type;
static int dock_gpio;
static bool onewire_init_ok;
static int test_mode;

struct dock_state {
	struct mutex lock;
	u32 t;
	u32 last_edge_t[2];
	u32 last_edge_i[2];
	bool level;
	bool dock_connected_unknown;
};

static struct workqueue_struct *dock_wq;
static struct work_struct dock_work;
static struct delayed_work test_work;
static struct wake_lock dock_work_wake_lock;
static struct dock_state ds = {
	.lock = __MUTEX_INITIALIZER(ds.lock),
};

#define dock_out(n) gpio_direction_output(dock_gpio, n)
#define dock_out2(n) gpio_set_value(dock_gpio, n)
#define dock_in() gpio_direction_input(dock_gpio)
#define dock_read() gpio_get_value(dock_gpio)

#define MFM_DELAY_NS 10000

static DEFINE_MUTEX(owe_notify_sem);
static void send_owe_charging_notify(int charging_type)
{
	static struct t_owe_charging_notifier *notifier;

	mutex_lock(&owe_notify_sem);
	charging_type = charging_type ? CONNECT_TYPE_AC : CONNECT_TYPE_USB;
	OWE_I("%s: charging_type = %d\n", __func__, charging_type);

	list_for_each_entry(notifier,
		&g_lh_owe_charging_notifier_list,
		owe_charging_notifier_link) {
			if (notifier->func != NULL) {
				OWE_I("Send to: %s, type %d\n",
						notifier->name, charging_type);
				notifier->func(charging_type);
			}
		}
	mutex_unlock(&owe_notify_sem);
}

int owe_charging_register_notifier(struct t_owe_charging_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&owe_notify_sem);
	list_add(&notifier->owe_charging_notifier_link,
		&g_lh_owe_charging_notifier_list);
	mutex_unlock(&owe_notify_sem);
	return 0;
}

static int dock_get_edge(struct dock_state *s, u32 timeout, u32 tmin, u32 tmax)
{
	bool lin;
	bool in = s->level;
	u32 t;
	do {
		lin = in;
		in = dock_read();
		t = msm_read_fast_timer();
		if (in != lin) {
			s->last_edge_t[in] = t;
			s->last_edge_i[in] = 0;
			s->level = in;
			if ((s32)(t - tmin) < 0 || (s32)(t - tmax) > 0)
				return -1;
			return 1;
		}
	} while ((s32)(t - timeout) < 0);
	return 0;
}

static bool dock_sync(struct dock_state *s, u32 timeout)
{
	u32 t;

	s->level = dock_read();
	t = msm_read_fast_timer();

	if (!dock_get_edge(s, t + timeout, 0, 0))
		return false;
	s->last_edge_i[s->level] = 2;

	return !!dock_get_edge(s,
			s->last_edge_t[s->level] + MFM_DELAY_NS * 4, 0, 0);
}

static int dock_get_next_bit(struct dock_state *s)
{
	u32 i = s->last_edge_i[!s->level] + ++s->last_edge_i[s->level];
	u32 target = s->last_edge_t[!s->level] + MFM_DELAY_NS * i;
	u32 timeout = target + MFM_DELAY_NS / 2;
	u32 tmin = target - MFM_DELAY_NS / 4;
	u32 tmax = target + MFM_DELAY_NS / 4;
	return dock_get_edge(s, timeout, tmin, tmax);
}

static u32 dock_get_bits(struct dock_state *s, int count, int *errp)
{
	u32 data = 0;
	u32 m = 1;
	int ret;
	int err = 0;
	while (count--) {
		ret = dock_get_next_bit(s);
		if (ret)
			data |= m;
		if (ret < 0)
			err++;
		m <<= 1;
	}
	if (errp)
		*errp = err;
	return data;
}

static void dock_delay(u32 timeout)
{
	timeout += msm_read_fast_timer();
	while (((s32)(msm_read_fast_timer() - timeout)) < 0)
		;
}

static int dock_send_bits(struct dock_state *s, u32 data, int count, int period)
{
	u32 t, t0, to;

	dock_out2(s->level);
	t = to = 0;
	t0 = msm_read_fast_timer();

	while (count--) {
		if (data & 1)
			dock_out2((s->level = !s->level));

		t = msm_read_fast_timer() - t0;
		if (t - to > period / 2) {
			OWE_WARN("dock: to = %d, t = %d\n", to, t);
			return -EIO;
		}

		to += MFM_DELAY_NS;
		do {
			t = msm_read_fast_timer() - t0;
		} while (t < to);
		if (t - to > period / 4) {
			OWE_WARN("dock: to = %d, t = %d\n", to, t);
			return -EIO;
		}
		data >>= 1;
	}
	return 0;
}

static u32 mfm_encode(u16 data, int count, bool p)
{
	u32 mask;
	u32 mfm = 0;
	u32 clock = ~data & ~(data << 1 | !!p);
	for (mask = 1UL << (count - 1); mask; mask >>= 1) {
		mfm |= (data & mask);
		mfm <<= 1;
		mfm |= (clock & mask);
	}
	return mfm;
}

static u32 mfm_decode(u32 mfm)
{
	u32 data = 0;
	u32 clock = 0;
	u32 mask = 1;
	while (mfm) {
		if (mfm & 1)
			clock |= mask;
		mfm >>= 1;
		if (mfm & 1)
			data |= mask;
		mfm >>= 1;
		mask <<= 1;
	}
	return data;
}

static int dock_command(struct dock_state *s, u16 cmd, int len, int retlen)
{
	u32 mfm;
	int count;
	u32 data = cmd;
	int ret;
	int err = -1;
	unsigned long flags;

	data = data << 2 | 3; /* add 0101 mfm data*/
	mfm = mfm_encode(data, len, false);
	count = len * 2 + 1;

	msm_enable_fast_timer();
	local_irq_save(flags);
	ret = dock_send_bits(s, mfm, count, MFM_DELAY_NS);
	if (!ret) {
		dock_in();
		if (dock_sync(s, MFM_DELAY_NS * 5))
			ret = dock_get_bits(s, retlen * 2, &err);
		else
			ret = -1;
		dock_out(s->level);
	}
	local_irq_restore(flags);

	dock_delay((ret < 0) ? MFM_DELAY_NS * 6 : MFM_DELAY_NS * 2);
	msm_disable_fast_timer();
	if (ret < 0) {
		OWE_WARN("dock_command: %x: no response\n", cmd);
		return ret;
	}
	data = mfm_decode(ret);
	mfm = mfm_encode(data, retlen, true);
	if (mfm != ret || err) {
		OWE_WARN("dock_command: %x: bad response, "
			   "data %x, mfm %x %x, err %d\n",
			   cmd, data, mfm, ret, err);
		return -EIO;
	}
	return data;
}

static int dock_command_retry(struct dock_state *s, u16 cmd, size_t len, size_t retlen)
{
	int retry = 20;
	int ret;
	while (retry--) {
		ret = dock_command(s, cmd, len, retlen);
		if (ret >= 0)
			return ret;
		if (retry != 19)
			msleep(10);
	}
	s->dock_connected_unknown = true;
	return -EIO;
}

static int dock_read_single(struct dock_state *s, int addr)
{
	int ret = -1;
	int retry = 20;
	while (retry--) {
		ret = dock_command_retry(s, addr << 1, 6, 8);
		if (ret >= 0)
			return ret;
	}
	return -EIO;
}

static int dock_read_multi(struct dock_state *s, int addr, u8 *data, size_t len)
{
	int ret;
	int i;
	u8 suml, sumr = -1;
	int retry = 20;
	while (retry--) {
		suml = 0;
		for (i = 0; i <= len; i++) {
			ret = dock_command_retry(s, (addr + i) << 1, 6, 8);
			if (ret < 0)
				return ret;
			if (i < len) {
				data[i] = ret;
				suml += ret;
			} else
				sumr = ret;
		}
		if (sumr == suml)
			return 0;

		OWE_WARN("dock_read_multi(%x): bad checksum, %x != %x\n",
			   addr, sumr, suml);
	}
	return -EIO;
}

static int dock_write_byte(struct dock_state *s, int addr, u8 data)
{
	return dock_command_retry(s, 1 | addr << 1 | data << 4, 6 + 8, 1);
}

static int dock_write_multi(struct dock_state *s, int addr, u8 *data, size_t len)
{
	int ret;
	int i;
	u8 sum;
	int retry = 2;
	while (retry--) {
		sum = 0;
		for (i = 0; i < len; i++) {
			sum += data[i];
			ret = dock_write_byte(s, addr + i, data[i]);
			if (ret < 0)
				return ret;
		}
		ret = dock_write_byte(s, addr + len, sum);
		if (ret <= 0)
			return ret;
	}
	return -EIO;
}

static int dock_acquire(struct dock_state *s)
{
	mutex_lock(&s->lock);
	dock_in();
	if (dock_read()) {
		/* Allow some time for the dock pull-down resistor to discharge
		 * the capasitor.
		 */
		msleep(20);
		if (dock_read()) {
			mutex_unlock(&s->lock);
			return -ENOENT;
		}
	}
	dock_out(0);
	s->level = false;
	return 0;
}

static void dock_release(struct dock_state *s)
{
	dock_in();
	mutex_unlock(&s->lock);
}

enum {
	DOCK_TYPE = 0x0,
	DOCK_BT_ADDR = 0x1, /* - 0x7 */

	DOCK_PIN_CODE = 0x0,
};

static ssize_t bt_addr_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int ret;
	u8 bt_addr[6];

	ret = dock_acquire(&ds);
	if (ret < 0)
		return ret;
	ret = dock_read_multi(&ds, DOCK_BT_ADDR, bt_addr, 6);
	dock_release(&ds);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		bt_addr[0], bt_addr[1], bt_addr[2],
		bt_addr[3], bt_addr[4], bt_addr[5]);
}
static DEVICE_ATTR(bt_addr, S_IRUGO | S_IWUSR, bt_addr_show, NULL);

static ssize_t bt_pin_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int ret, i;
	u8 pin[4];

	if (size < 4)
		return -EINVAL;

	for (i = 0; i < sizeof(pin); i++) {
		if ((pin[i] = buf[i] - '0') > 10)
			return -EINVAL;
	}

	ret = dock_acquire(&ds);
	if (ret < 0)
		return ret;
	ret = dock_write_multi(&ds, DOCK_PIN_CODE, pin, 4);
	dock_release(&ds);
	if (ret < 0)
		return ret;

	return size;
}
static DEVICE_ATTR(bt_pin, S_IRUGO | S_IWUSR, NULL, bt_pin_store);

static ssize_t owe_test_mode(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	sscanf(buf, "%d", &test_mode);
	OWE_I("%s test_mode = %d\n", __func__, test_mode);
	return size;
}
static DEVICE_ATTR(test, S_IRUGO | S_IWUSR, NULL, owe_test_mode);

static void dock_test_proc(struct work_struct *work)
{
	int ret, dockid;
	u8 bt_addr[6];
	u8 pin[4] = {1, 2, 3, 4};

	if (!vbus_present || dock_acquire(&ds)) {
		OWE_I("%s no dock, vbus %d\n", __func__, vbus_present);
		return;
	}
	dockid = dock_read_single(&ds, DOCK_TYPE);
	OWE_I("%s Detected dock with ID %02x\n", __func__, dockid);

	ret = dock_read_multi(&ds, DOCK_BT_ADDR, bt_addr, 6);
	if (ret < 0)
		OWE_ERR("%s read bt_addr fail\n", __func__);
	OWE_I("%s BT_ADDR = %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
			bt_addr[0], bt_addr[1], bt_addr[2],
			bt_addr[3], bt_addr[4], bt_addr[5]);

	ret = dock_write_multi(&ds, DOCK_PIN_CODE, pin, 4);
	if (ret < 0)
		OWE_ERR("%s write bt_pin fail\n", __func__);
	else
		OWE_I("%s write bt_pin success\n", __func__);

	dock_release(&ds);

	if (test_mode)
		queue_delayed_work(dock_wq, &test_work, HZ/2);
	return;
}

static void dock_work_proc(struct work_struct *work)
{
	int dockid;

	if (!vbus_present || dock_acquire(&ds))
		goto no_dock;

	if (ds.dock_connected_unknown) {
		/* force a new dock notification if a command failed */
		switch_set_state(&dock_switch, 0);
		OWE_I("%s Set dock switch state 0\n", __func__);
		ds.dock_connected_unknown = false;
	}
	dockid = dock_read_single(&ds, DOCK_TYPE);
	dock_release(&ds);

	OWE_I("%s Detected dock with ID %02x\n", __func__, dockid);
	if (dockid >= 0) {
		dock_charging_type = !!(dockid & 0x80);
		switch_set_state(&dock_switch, (dockid & 1) ? 2 : 1);
		OWE_I("%s Set dock switch state %d\n",
				__func__, (dockid & 1) ? 2 : 1);
		send_owe_charging_notify(dock_charging_type);
		goto done;
	}
no_dock:
	OWE_I("%s no dock, vbus %d\n",
		__func__, vbus_present);
	dock_charging_type = 0;
	switch_set_state(&dock_switch, 0);
	OWE_I("%s Set dock switch state 0\n", __func__);
done:
	if (onewire_init_ok == 0)
		onewire_init_ok = 1;
	wake_unlock(&dock_work_wake_lock);
}

void onewire_detect_start(int status)
{
	OWE_D("dock detect start\n");
	status = !!status;
	vbus_present = status;
	OWE_I("dock vbus_present %d \n",  vbus_present);
	if (onewire_init_ok == 0) {
		OWE_I("driver not ready\n");
		return;
	}
	wake_lock_timeout(&dock_work_wake_lock, HZ*5);
	if (test_mode) {
		if (!vbus_present)
			test_mode--;
		else
			queue_delayed_work(dock_wq, &test_work, 0);
		return;
	}
	queue_work(dock_wq, &dock_work);
}

static int htc_onewire_probe(struct platform_device *pdev)
{
	struct msm_onewire_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	OWE_I("%s in\n", __func__);
	INIT_WORK(&dock_work, dock_work_proc);
	INIT_DELAYED_WORK(&test_work, dock_test_proc);
	dock_gpio = pdata->gpio_one_wire;
	OWE_I("dock gpio = %d\n", dock_gpio);
	gpio_request(dock_gpio, "onewire");

	if (pdata->onewire_gpio_config)
		pdata->onewire_gpio_config();
	dock_in();
	msleep(20);

	wake_lock_init(&dock_work_wake_lock, WAKE_LOCK_SUSPEND, "onewire");

	if (switch_dev_register(&dock_switch) == 0) {
		ret = device_create_file(dock_switch.dev, &dev_attr_bt_addr);
		WARN_ON(ret);
		ret = device_create_file(dock_switch.dev, &dev_attr_bt_pin);
		WARN_ON(ret);
		ret = device_create_file(dock_switch.dev, &dev_attr_test);
		WARN_ON(ret);
	}

	dock_wq = create_singlethread_workqueue("dock");

	wake_lock_timeout(&dock_work_wake_lock, HZ*5);
	queue_work(dock_wq, &dock_work);

	return 0;
}

static struct platform_driver htc_onewire_driver = {
	.probe	= htc_onewire_probe,
	.driver	= {
		.name	= ONEWIRE_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init htc_onewire_init(void)
{
	platform_driver_register(&htc_onewire_driver);

	return 0;
}

module_init(htc_onewire_init);
MODULE_DESCRIPTION("HTC One Wire Driver");
MODULE_LICENSE("GPL");

