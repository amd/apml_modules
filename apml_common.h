/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * apml_common.h - Shared registry for SB-RMI / SB-TSI Alert_L endpoints
 *
 * Copyright (C) 2025 Advanced Micro Devices, Inc.
 */

#ifndef _AMD_APML_COMMON_H_
#define _AMD_APML_COMMON_H_

#include <linux/list.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include "sbrmi-common.h"
#include "sbtsi-common.h"

/*
 * Global list of registered APML device nodes.
 */
extern struct list_head apml_devices;
extern struct mutex apml_devices_lock;

enum apml_device_type {
	APML_RMI_DEVICE,	/* SB-RMI (RAS alerts via register 0x4C) */
	APML_TSI_DEVICE,	/* SB-TSI (temperature alerts via register 0x02) */
};

/* Registry wrapper for one SB-RMI or SB-TSI device exposed to Alert_L. */
/* apml_dev_list: link on the global apml_devices list.
 * refcount: reference count held by Alert_L during alert dispatch; registration
 * starts with a count of one and the node is freed when the count drops
 * to zero.
 * release_wait: apml_unregister_device() waits here until every consumer
 * has dropped refcount before freeing the wrapper.
 * dev_type: selects whether rmi_dev or tsi_dev is valid in the union below.
 * rmi_dev: SB-RMI parent when dev_type is APML_RMI_DEVICE.
 * tsi_dev: SB-TSI parent when dev_type is APML_TSI_DEVICE.
 */
struct apml_device_node {
	struct list_head apml_dev_list;
	struct kref refcount;
	wait_queue_head_t release_wait;
	enum apml_device_type dev_type;
	union {
		struct apml_sbrmi_device *rmi_dev;
		struct apml_sbtsi_device *tsi_dev;
	};
};

int apml_register_device(void *device, enum apml_device_type dev_type);
void apml_unregister_device(void *device, enum apml_device_type dev_type);
struct apml_device_node *apml_get_device_node(struct apml_device_node *node);
void apml_put_device_node(struct apml_device_node *node);

#endif /* _AMD_APML_COMMON_H_ */
