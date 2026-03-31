/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * sbtsi-common.h - Common header file to share the APML SB-TSI structure
 *
 * Copyright (c) 2025 Advanced Micro Devices, Inc.
 */

#ifndef _AMD_SBTSI_COMMON_H_
#define _AMD_SBTSI_COMMON_H_

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>

struct i2c_client;
struct i3c_device;
struct regmap;

/* Each client has this additional data */
/* in_progress: set during any transaction, hwmon read/write or IOCTL,
 * to indicate a transaction is in progress.
 * no_new_trans: set in rmmod/unbind path to indicate,
 * not to accept new transactions
 */
struct apml_sbtsi_device {
	struct miscdevice sbtsi_misc_dev;
	struct i2c_client *client;
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	struct mutex lock;	/* lock for tsi devices */
	u8 dev_static_addr;
	atomic_t in_progress;
	atomic_t no_new_trans;
	struct completion misc_fops_done;
};

#endif
