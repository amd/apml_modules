/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2025 Advanced Micro Devices, Inc.
 */

#ifndef _AMD_APML_ALERT_L__
#define _AMD_APML_ALERT_L__

struct device;

/* struct apml_alertl_data - APML Alert_L driver data structure */
struct apml_alertl_data {
	struct device *dev;
	int irq_num;
};

#endif /*_AMD_APML_ALERT_L__*/
