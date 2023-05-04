/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */

#ifndef _AMD_APML_SBRMI_H_
#define _AMD_APML_SBRMI_H_

#include <linux/miscdevice.h>
#include "amd-apml.h"

/* Each client has this additional data */
/* in_progress: set during any transaction, mailbox/cpuid/mcamsr/readreg,
 * to indicate a transaction is in progress.
 * no_new_trans: set in rmmod/unbind path to indicate,
 * not to accept new transactions
 */
struct apml_sbrmi_device {
	struct miscdevice sbrmi_misc_dev;
	struct completion misc_fops_done;
	struct regmap *regmap;
	struct mutex lock;
	u32 pwr_limit_max;
	atomic_t in_progress;
	atomic_t no_new_trans;
	u8 rev;
} __packed;

int rmi_mca_msr_read(struct apml_sbrmi_device *rmi_dev,
		     struct apml_message *msg);
int rmi_cpuid_read(struct apml_sbrmi_device *rmi_dev,
		   struct apml_message *msg);
int rmi_mailbox_xfer(struct apml_sbrmi_device *rmi_dev,
		     struct apml_message *msg);
#endif /*_AMD_APML_SBRMI_H_*/
