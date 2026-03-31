// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * apml-common.c - Common registration system for APML devices
 *
 * Copyright (C) 2025 Advanced Micro Devices, Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/wait.h>

#include "apml_common.h"

/* Global list of registered APML devices */
LIST_HEAD(apml_devices);
EXPORT_SYMBOL_GPL(apml_devices);

/*
 * Global Lock: Protects apml_devices list structure during registration,
 * unregistration, and iteration.
 */
DEFINE_MUTEX(apml_devices_lock);
EXPORT_SYMBOL_GPL(apml_devices_lock);

/* Release function for device node reference counting */
static void apml_device_node_release(struct kref *ref)
{
	struct apml_device_node *node = container_of(ref, struct apml_device_node, refcount);

	wake_up_all(&node->release_wait);
	kfree(node);
}

/* Get a reference to a device node - increases reference count */
struct apml_device_node *apml_get_device_node(struct apml_device_node *node)
{
	if (node && kref_get_unless_zero(&node->refcount))
		return node;
	return NULL;
}
EXPORT_SYMBOL_GPL(apml_get_device_node);

/* Put a reference to a device node - decreases reference count */
void apml_put_device_node(struct apml_device_node *node)
{
	if (!node)
		return;

	if (!kref_put(&node->refcount, apml_device_node_release))
		wake_up_all(&node->release_wait);
}
EXPORT_SYMBOL_GPL(apml_put_device_node);

/*
 * Wait until only the registration kref remains, then drop it and free.
 * Uses release_wait rather than kref internals; pairs with wake_up_all()
 * in apml_put_device_node() and apml_device_node_release().
 */
static void apml_put_device_node_and_wait(struct apml_device_node *node)
{
	wait_event(node->release_wait, kref_read(&node->refcount) == 1);
	apml_put_device_node(node);
}

static void apml_add_device_node(struct apml_device_node *node)
{
	mutex_lock(&apml_devices_lock);
	list_add_tail(&node->apml_dev_list, &apml_devices);
	mutex_unlock(&apml_devices_lock);
}

static int apml_register_rmi(struct apml_device_node *node,
			     struct apml_sbrmi_device *rmi_dev)
{
	if (!rmi_dev->i3cdev && !rmi_dev->client) {
		pr_err("APML: Invalid RMI device - no I2C or I3C device\n");
		return -EINVAL;
	}

	node->rmi_dev = rmi_dev;
	pr_info("APML: Registered SBRMI device at address 0x%x\n",
		rmi_dev->i3cdev ? rmi_dev->dev_static_addr
				: rmi_dev->client->addr);
	return 0;
}

static int apml_register_tsi(struct apml_device_node *node,
			     struct apml_sbtsi_device *tsi_dev)
{
	if (!tsi_dev->i3cdev && !tsi_dev->client) {
		pr_err("APML: Invalid TSI device - no I2C or I3C device\n");
		return -EINVAL;
	}

	node->tsi_dev = tsi_dev;
	pr_info("APML: Registered SBTSI device at address 0x%x\n",
		tsi_dev->i3cdev ? tsi_dev->dev_static_addr
				: tsi_dev->client->addr);
	return 0;
}

/*
 * apml_register_device - record a registered SB-RMI / SB-TSI device.
 *
 * Returns 0 on success, -EINVAL on a malformed device, -ENOMEM on alloc
 * failure. Callers may treat failure as non-fatal: SBRMI/SBTSI probe paths
 * log dev_warn() and continue so hwmon/misc still bind.
 */
int apml_register_device(void *device, enum apml_device_type dev_type)
{
	struct apml_device_node *node;
	int ret;

	if (!device) {
		pr_info("APML: Cannot register NULL device\n");
		return -EINVAL;
	}

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	node->dev_type = dev_type;
	kref_init(&node->refcount);
	init_waitqueue_head(&node->release_wait);

	switch (dev_type) {
	case APML_RMI_DEVICE:
		ret = apml_register_rmi(node, device);
		break;
	case APML_TSI_DEVICE:
		ret = apml_register_tsi(node, device);
		break;
	default:
		ret = -EINVAL;
	}
	if (ret) {
		kfree(node);
		return ret;
	}

	apml_add_device_node(node);
	return 0;
}
EXPORT_SYMBOL_GPL(apml_register_device);

/*
 * apml_unregister_device - drop a previously registered SB-RMI / SB-TSI device.
 *
 * Safe to call with an unknown @device; the function is a no-op in that case.
 *
 * Unlink is immediate; the call then waits until Alert_L (or any other
 * consumer) drops its kref before freeing the registry wrapper.  That keeps
 * @device valid for the regmap I/O in a running IRQ thread.  Duration is
 * bounded by the in-flight alert handler, which in turn relies on the I2C/I3C
 * adapter transaction timeouts rather than a separate registry timer.  A
 * timeout here cannot safely free the node without risking use-after-free.
 */
void apml_unregister_device(void *device, enum apml_device_type dev_type)
{
	struct apml_device_node *node, *tmp, *found_node = NULL;
	bool found;

	if (!device)
		return;

	mutex_lock(&apml_devices_lock);
	list_for_each_entry_safe(node, tmp, &apml_devices, apml_dev_list) {
		found = false;

		if (node->dev_type != dev_type)
			continue;

		switch (dev_type) {
		case APML_RMI_DEVICE:
			found = node->rmi_dev == device;
			break;
		case APML_TSI_DEVICE:
			found = node->tsi_dev == device;
			break;
		}

		if (found) {
			found_node = node;
			list_del(&found_node->apml_dev_list);
			break;
		}
	}
	mutex_unlock(&apml_devices_lock);

	if (!found_node)
		return;

	/* Wait out consumer krefs, then drop the registration kref and free. */
	apml_put_device_node_and_wait(found_node);
}
EXPORT_SYMBOL_GPL(apml_unregister_device);

MODULE_AUTHOR("Sathya Priya Kumar <sathyapriya.k@amd.com>");
MODULE_AUTHOR("Akshay Gupta <akshay.gupta@amd.com>");
MODULE_DESCRIPTION("AMD APML common registry for SB-RMI/SB-TSI devices(Alert_L)");
MODULE_LICENSE("GPL");
