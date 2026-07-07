// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * sbrmi.c - hwmon driver for a SB-RMI mailbox
 *           compliant AMD SoC device.
 *
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "sbrmi-common.h"
#include "apml_common.h"

/* Do not allow setting negative power limit */
#define SBRMI_PWR_MIN	0

/* Try 2 byte address size before switching to 1 byte */
#define MAX_RETRY	5


/* SBRMI REVISION REG */
#define SBRMI_REV	0x0

#define MAX_WAIT_TIME_SEC	(3)

/* SBRMI registers data out is 1 byte */
#define SBRMI_REG_DATA_SIZE		0x1
/* Default SBRMI register address is 1 byte */
#define SBRMI_REG_ADDR_SIZE_DEF		0x1
/* TURIN SBRMI register address is 2 byte */
#define SBRMI_REG_ADDR_SIZE_TWO_BYTE	0x2

/* Two xfers, one write and one read require to read the data */
#define I3C_I2C_MSG_XFER_SIZE		0x2

/* DIMM temp */
#define DIMM_BASE_ID		0x80	/* Mode 1: Bit[7]=1 */
#define DIMM_TS_SELECT_BIT	BIT(6)	/* 0 = TS0, 1 = TS1 */
#define DIMM_TEMP_OFFSET	21
#define DIMM_CHANNELS_PER_TS	16
#define SBRMI_MAX_DIMMS		DIMM_CHANNELS_PER_TS
#define SBRMI_NUM_DIMM_CHANNELS	(DIMM_CHANNELS_PER_TS * 2) /* TS0 + TS1 */
#define PID_RMI_GENOA_TURIN	0x22400000002ULL
/*
 * DIMM_ADDR() - Encode a hwmon channel into a Mode 1 DIMM_ADDRESS byte
 *   Bit[7]   : 1        - Mode 1
 *   Bit[6]   : TS select - ch / DIMM_CHANNELS_PER_TS yields 0 (TS0) or
 *               1 (TS1), shifted to bit 6
 *   Bit[3:0] : UMC/DDRPHY Instance ID - ch % DIMM_CHANNELS_PER_TS
 */
#define DIMM_ADDR(ch)						\
	(DIMM_BASE_ID |					\
	 (((ch) / DIMM_CHANNELS_PER_TS) << 6) |		\
	 ((ch) % DIMM_CHANNELS_PER_TS))

/*
 * DIMM thermal config kept separate from apml_sbrmi_device (common header).
 * Co-allocated with rmi_dev; probes still use apml_sbrmi_device * as drvdata.
 */
struct sbrmi_dimm_config {
	u8 *dimm_ids;
	int num_umc;
};

struct sbrmi_device_alloc {
	struct apml_sbrmi_device rmi_dev;
	struct sbrmi_dimm_config dimm;
};

static struct apml_sbrmi_device *sbrmi_dev_alloc(struct device *dev)
{
	struct sbrmi_device_alloc *alloc;

	alloc = devm_kzalloc(dev, sizeof(*alloc), GFP_KERNEL);
	if (!alloc)
		return NULL;

	return &alloc->rmi_dev;
}

static struct sbrmi_dimm_config *sbrmi_to_dimm_config(struct apml_sbrmi_device *rmi_dev)
{
	return &container_of(rmi_dev, struct sbrmi_device_alloc, rmi_dev)->dimm;
}

static int configure_regmap(struct apml_sbrmi_device *rmi_dev);
static bool sbrmi_dimm_channel_valid(const struct sbrmi_dimm_config *dimm, int channel);
static int sbrmi_init_dimm_config(struct device *dev,
				  struct apml_sbrmi_device *rmi_dev);

enum sbrmi_msg_id {
	SBRMI_READ_PKG_PWR_CONSUMPTION = 0x1,
	SBRMI_WRITE_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_MAX_PWR_LIMIT,
	SBRMI_READ_DIMM_THERMAL_SENSOR = 0x48,
};

static int sbrmi_prepare_lock(struct apml_sbrmi_device *rmi_dev)
{
	mutex_lock(&rmi_dev->lock);
	/* Verify device unbind/remove is not invoked */
	if (atomic_read(&rmi_dev->no_new_trans)) {
		mutex_unlock(&rmi_dev->lock);
		return -EBUSY;
	}

	/*
	 * Set the in_progress variable to true, to wait for
	 * completion during unbind/remove of driver
	 */
	atomic_set(&rmi_dev->in_progress, 1);
	return 0;
}

static void sbrmi_prepare_unlock(struct apml_sbrmi_device *rmi_dev)
{
	/* Send complete only if device is unbinded/remove */
	if (atomic_read(&rmi_dev->no_new_trans))
		complete(&rmi_dev->misc_fops_done);

	atomic_set(&rmi_dev->in_progress, 0);
	mutex_unlock(&rmi_dev->lock);
}

static int sbrmi_get_max_pwr_limit(struct apml_sbrmi_device *rmi_dev)
{
	struct apml_message msg = { 0 };
	int ret = 0;

	msg.cmd = SBRMI_READ_PKG_MAX_PWR_LIMIT;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;
	ret = rmi_mailbox_xfer(rmi_dev, &msg);
	if (ret < 0)
		return ret;
	rmi_dev->pwr_limit_max = msg.data_out.mb_out[RD_WR_DATA_INDEX];

	return ret;
}

static int sbrmi_read(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long *val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct sbrmi_dimm_config *dimm = sbrmi_to_dimm_config(rmi_dev);
	struct apml_message msg = { 0 };
	int ret = 0;

	if (type != hwmon_power && type != hwmon_temp)
		return -EINVAL;

	ret = sbrmi_prepare_lock(rmi_dev);
	if (ret)
		return ret;

	/* Configure regmap if not configured yet */
	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in hwmon read ops\n", ret);
			sbrmi_prepare_unlock(rmi_dev);
			return ret;
		}
	}

	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;

	if (type == hwmon_power) {
		switch (attr) {
		case hwmon_power_input:
			msg.cmd = SBRMI_READ_PKG_PWR_CONSUMPTION;
			ret = rmi_mailbox_xfer(rmi_dev, &msg);
			break;
		case hwmon_power_cap:
			msg.cmd = SBRMI_READ_PKG_PWR_LIMIT;
			ret = rmi_mailbox_xfer(rmi_dev, &msg);
			break;
		case hwmon_power_cap_max:
			if (!rmi_dev->pwr_limit_max) {
				/* Cache maximum power limit */
				ret = sbrmi_get_max_pwr_limit(rmi_dev);
			}
			msg.data_out.mb_out[RD_WR_DATA_INDEX] = rmi_dev->pwr_limit_max;
			break;
		default:
			ret = -EINVAL;
		}
	} else {
		switch (attr) {
		case hwmon_temp_input:
			if (!sbrmi_dimm_channel_valid(dimm, channel)) {
				ret = -EINVAL;
				break;
			}
			msg.cmd = SBRMI_READ_DIMM_THERMAL_SENSOR;
			msg.data_in.mb_in[RD_WR_DATA_INDEX] = dimm->dimm_ids[channel];
			ret = rmi_mailbox_xfer(rmi_dev, &msg);
			if (ret == -EPROTOTYPE)
				dev_dbg(dev, "FW error code:0x%x\n", msg.fw_ret_code);
			break;
		default:
			ret = -EINVAL;
		}
	}

	if (!ret) {
		if (type == hwmon_power) {
			/* hwmon power attributes are in microWatt */
			*val = (long)msg.data_out.mb_out[RD_WR_DATA_INDEX] * 1000;
		} else {
			/*
			 * Temperature is an 11-bit two's complement signed value
			 * (bits 10:0) with 0.25 deg C resolution. Bit 10 is the
			 * sign bit; if set, extend the sign by subtracting 0x800.
			 * Convert to milli-degrees C using integer math (multiply
			 * by 250) to avoid floating point in the kernel.
			 */
			*val = msg.data_out.mb_out[RD_WR_DATA_INDEX] >> DIMM_TEMP_OFFSET;
			if (*val & 0x400)
				*val -= 0x800;
			/* Report temperature in mC as per hwmon standards */
			*val *= 250;
		}
	}

	sbrmi_prepare_unlock(rmi_dev);
	return ret;
}

static u8 sbrmi_dimm_addr_from_ts0(u8 ts0, unsigned int ts)
{
	return (ts0 & ~DIMM_TS_SELECT_BIT) | (ts ? DIMM_TS_SELECT_BIT : 0);
}

static bool sbrmi_dimm_channel_valid(const struct sbrmi_dimm_config *dimm, int channel)
{
	int umc = channel % DIMM_CHANNELS_PER_TS;
	int ts = channel / DIMM_CHANNELS_PER_TS;

	return dimm->dimm_ids && ts < 2 && umc >= 0 && umc < dimm->num_umc;
}

/*
 * sbrmi_init_dimm_config() - Parse DT and build DIMM mailbox address table.
 *
 * Optional "dimm-ids" lists TS0 base addresses in order (one per UMC).
 * The number of entries sets the populated UMC count (1..16). TS1 is derived
 * by setting bit[6] (TS0 | 0x40). When absent, use legacy 0x80 encoding
 * for all 16 UMC instances.
 */
static int sbrmi_init_dimm_config(struct device *dev,
				  struct apml_sbrmi_device *rmi_dev)
{
	struct sbrmi_dimm_config *dimm = sbrmi_to_dimm_config(rmi_dev);
	struct device_node *np = dev->of_node;
	u32 *raw_ids;
	int count, umc, ts, ret;

	dimm->num_umc = SBRMI_MAX_DIMMS;
	dimm->dimm_ids = devm_kcalloc(dev, SBRMI_NUM_DIMM_CHANNELS,
				      sizeof(u8), GFP_KERNEL);
	if (!dimm->dimm_ids)
		return -ENOMEM;

	if (!np)
		goto default_ids;

	count = of_property_count_elems_of_size(np, "dimm-ids", sizeof(u32));
	if (count <= 0)
		goto default_ids;

	if (count < 1 || count > SBRMI_MAX_DIMMS) {
		dev_err(dev, "dimm-ids must have 1..%d entries, got %d\n",
			SBRMI_MAX_DIMMS, count);
		return -EINVAL;
	}

	dimm->num_umc = count;

	raw_ids = devm_kcalloc(dev, count, sizeof(u32), GFP_KERNEL);
	if (!raw_ids)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "dimm-ids", raw_ids, count);
	if (ret) {
		dev_err(dev, "failed to read dimm-ids: %d\n", ret);
		return ret;
	}

	for (umc = 0; umc < count; umc++) {
		u8 ts0 = raw_ids[umc] & 0xff;

		for (ts = 0; ts < 2; ts++) {
			int ch = ts * DIMM_CHANNELS_PER_TS + umc;

			dimm->dimm_ids[ch] = sbrmi_dimm_addr_from_ts0(ts0, ts);
		}
	}

	dev_dbg(dev, "Using %d platform dimm-ids\n", count);
	return 0;

default_ids:
	for (umc = 0; umc < dimm->num_umc; umc++) {
		for (ts = 0; ts < 2; ts++) {
			int ch = ts * DIMM_CHANNELS_PER_TS + umc;

			dimm->dimm_ids[ch] = DIMM_ADDR(ch);
		}
	}

	return 0;
}

static int sbrmi_write(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct apml_message msg = { 0 };
	int ret;

	if (type != hwmon_power && attr != hwmon_power_cap)
		return -EINVAL;

	/*
	 * hwmon power attributes are in microWatt
	 * mailbox read/write is in mWatt
	 */
	val /= 1000;

	val = clamp_val(val, SBRMI_PWR_MIN, rmi_dev->pwr_limit_max);

	msg.cmd = SBRMI_WRITE_PKG_PWR_LIMIT;
	msg.data_in.mb_in[RD_WR_DATA_INDEX] = val;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 0;

	ret = sbrmi_prepare_lock(rmi_dev);
	if (ret)
		return ret;

	/* Configure regmap if not configured yet */
	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in hwmon write ops\n", ret);
			sbrmi_prepare_unlock(rmi_dev);
			return ret;
		}
	}

	ret = rmi_mailbox_xfer(rmi_dev, &msg);

	sbrmi_prepare_unlock(rmi_dev);
	return ret;
}

static umode_t sbrmi_is_visible(const void *data,
				enum hwmon_sensor_types type,
				u32 attr, int channel)
{
	switch (type) {
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_cap_max:
			return 0444;
		case hwmon_power_cap:
			return 0644;
		}
		break;
	case hwmon_temp:
	{
		const struct apml_sbrmi_device *rmi_dev = data;

		if (!sbrmi_dimm_channel_valid(&container_of(rmi_dev,
							    const struct sbrmi_device_alloc,
							    rmi_dev)->dimm, channel))
			return 0;
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_label:
			return 0444;
		}
		break;
	}
	default:
		break;
	}
	return 0;
}

static const u32 sbrmi_temp_config[SBRMI_NUM_DIMM_CHANNELS + 1] = {
	[0 ... (SBRMI_NUM_DIMM_CHANNELS - 1)] = HWMON_T_INPUT | HWMON_T_LABEL,
};

static const struct hwmon_channel_info sbrmi_temp_channel_info = {
	.type = hwmon_temp,
	.config = sbrmi_temp_config,
};

static const struct hwmon_channel_info *sbrmi_info[] = {
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_CAP | HWMON_P_CAP_MAX),
	&sbrmi_temp_channel_info,
	NULL
};

/*
 * Static label table - avoids any allocation in the read path.
 * Channels  0-15: TS0, UMC/DDRPHY instances 0-15
 * Channels 16-31: TS1, UMC/DDRPHY instances 0-15
 */
static const char * const sbrmi_temp_labels[SBRMI_NUM_DIMM_CHANNELS] = {
	[0]  = "DIMM_TS0_UMC0",  [1]  = "DIMM_TS0_UMC1",
	[2]  = "DIMM_TS0_UMC2",  [3]  = "DIMM_TS0_UMC3",
	[4]  = "DIMM_TS0_UMC4",  [5]  = "DIMM_TS0_UMC5",
	[6]  = "DIMM_TS0_UMC6",  [7]  = "DIMM_TS0_UMC7",
	[8]  = "DIMM_TS0_UMC8",  [9]  = "DIMM_TS0_UMC9",
	[10] = "DIMM_TS0_UMC10", [11] = "DIMM_TS0_UMC11",
	[12] = "DIMM_TS0_UMC12", [13] = "DIMM_TS0_UMC13",
	[14] = "DIMM_TS0_UMC14", [15] = "DIMM_TS0_UMC15",
	[16] = "DIMM_TS1_UMC0",  [17] = "DIMM_TS1_UMC1",
	[18] = "DIMM_TS1_UMC2",  [19] = "DIMM_TS1_UMC3",
	[20] = "DIMM_TS1_UMC4",  [21] = "DIMM_TS1_UMC5",
	[22] = "DIMM_TS1_UMC6",  [23] = "DIMM_TS1_UMC7",
	[24] = "DIMM_TS1_UMC8",  [25] = "DIMM_TS1_UMC9",
	[26] = "DIMM_TS1_UMC10", [27] = "DIMM_TS1_UMC11",
	[28] = "DIMM_TS1_UMC12", [29] = "DIMM_TS1_UMC13",
	[30] = "DIMM_TS1_UMC14", [31] = "DIMM_TS1_UMC15",
};

static int sbrmi_read_string(struct device *dev,
			     enum hwmon_sensor_types type,
			     u32 attr, int channel, const char **str)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct sbrmi_dimm_config *dimm = sbrmi_to_dimm_config(rmi_dev);

	if (type != hwmon_temp || attr != hwmon_temp_label)
		return -EOPNOTSUPP;
	if (!sbrmi_dimm_channel_valid(dimm, channel))
		return -EINVAL;
	*str = sbrmi_temp_labels[channel];
	return 0;
}

static const struct hwmon_ops sbrmi_hwmon_ops = {
	.is_visible = sbrmi_is_visible,
	.read = sbrmi_read,
	.read_string = sbrmi_read_string,
	.write = sbrmi_write,
};

static const struct hwmon_chip_info sbrmi_chip_info = {
	.ops = &sbrmi_hwmon_ops,
	.info = sbrmi_info,
};

static long sbrmi_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int __user *arguser = (int  __user *)arg;
	struct apml_message msg = { 0 };
	bool read = false;
	int ret = -EFAULT;
	struct apml_sbrmi_device *rmi_dev;

	rmi_dev = fp->private_data;
	if (!rmi_dev)
		return -ENODEV;

	/*
	 * If device remove/unbind is called do not allow new transaction
	 */
	if (atomic_read(&rmi_dev->no_new_trans))
		return -EBUSY;
	/* Copy the structure from user */
	if (copy_struct_from_user(&msg, sizeof(msg), arguser,
				  sizeof(struct apml_message)))
		return ret;
	/*
	 * Only one I2C/I3C transaction can happen at
	 * one time. Take lock across so no two protocol is
	 * invoked at same time, modifying the register value.
	 */
	ret = sbrmi_prepare_lock(rmi_dev);
	if (ret)
		return ret;

	/* Is this a read/monitor/get request */
	if (msg.data_in.reg_in[RD_FLAG_INDEX])
		read = true;

	switch (msg.cmd) {
	case 0 ... 0x999:
		/* Mailbox protocol */
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case APML_CPUID:
		ret = rmi_cpuid_read(rmi_dev, &msg);
		break;
	case APML_MCA_MSR:
		/* MCAMSR protocol */
		ret = rmi_mca_msr_read(rmi_dev, &msg);
		break;
	case APML_REG:
		/* REG R/W */
		if (read) {
			ret = regmap_read(rmi_dev->regmap,
					  msg.data_in.mb_in[REG_OFF_INDEX],
					  &msg.data_out.mb_out[RD_WR_DATA_INDEX]);
		} else {
			ret = regmap_write(rmi_dev->regmap,
					    msg.data_in.mb_in[REG_OFF_INDEX],
					    msg.data_in.reg_in[REG_VAL_INDEX]);
		}
		break;
	default:
		break;
	}

	sbrmi_prepare_unlock(rmi_dev);

	/* Copy results back to user only for get/monitor commands and firmware failures */
	if ((read && !ret) || ret == -EPROTOTYPE) {
		if (copy_to_user(arguser, &msg, sizeof(struct apml_message)))
			ret = -EFAULT;
	}
	return ret;
}

static int sbrmi_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct apml_sbrmi_device *rmi_dev = container_of(mdev, struct apml_sbrmi_device,
							 sbrmi_misc_dev);
	int ret;

	if (!rmi_dev)
		return -ENODEV;

	ret = sbrmi_prepare_lock(rmi_dev);
	if (ret)
		return ret;

	if (!rmi_dev->regmap) {
		ret = configure_regmap(rmi_dev);
		if (ret < 0) {
			pr_err("regmap configuration failed with return value:%d in misc dev open\n", ret);
			sbrmi_prepare_unlock(rmi_dev);
			return ret;
		}
	}

	sbrmi_prepare_unlock(rmi_dev);
	filp->private_data = rmi_dev;
	return 0;
}

static int sbrmi_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static const struct file_operations sbrmi_fops = {
	.owner		= THIS_MODULE,
	.open		= sbrmi_open,
	.release	= sbrmi_release,
	.unlocked_ioctl	= sbrmi_ioctl,
	.compat_ioctl	= sbrmi_ioctl,
};

static int create_misc_rmi_device(struct apml_sbrmi_device *rmi_dev,
				  struct device *dev)
{
	int ret;

	rmi_dev->sbrmi_misc_dev.name		= devm_kasprintf(dev, GFP_KERNEL,
						  "sbrmi-%x", rmi_dev->dev_static_addr);
	rmi_dev->sbrmi_misc_dev.minor		= MISC_DYNAMIC_MINOR;
	rmi_dev->sbrmi_misc_dev.fops		= &sbrmi_fops;
	rmi_dev->sbrmi_misc_dev.parent		= dev;
	rmi_dev->sbrmi_misc_dev.nodename	= devm_kasprintf(dev, GFP_KERNEL,
						  "sbrmi-%x", rmi_dev->dev_static_addr);
	rmi_dev->sbrmi_misc_dev.mode		= 0600;

	ret = misc_register(&rmi_dev->sbrmi_misc_dev);
	if (ret)
		return ret;

	dev_info(dev, "register %s device\n", rmi_dev->sbrmi_misc_dev.name);
	return ret;
}

static int sbrmi_i2c_reg_read(struct i2c_client *i2cdev, int reg_size, u32 *val)
{
	struct i2c_msg xfer[I3C_I2C_MSG_XFER_SIZE];
	int reg = SBRMI_REV;
	int val_size = SBRMI_REG_DATA_SIZE;

	xfer[0].addr = i2cdev->addr;
	xfer[0].flags = 0;
	xfer[0].len = reg_size;
	xfer[0].buf = (void *)&reg;

	xfer[1].addr = i2cdev->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = val_size;
	xfer[1].buf = (void *)val;

	return i2c_transfer(i2cdev->adapter, xfer, I3C_I2C_MSG_XFER_SIZE);
}

static int update_reg_addr_size(u32 rev, u32 *size)
{
	switch (rev) {
	case 0x10:
	case 0x20:
	case 0x30:
		*size = SBRMI_REG_ADDR_SIZE_DEF;
		return 0;
	case 0x21:
	case 0x31:
		*size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
		return 0;
	default:
		return -EIO;
	}
}

static int sbrmi_i2c_identify_reg_addr_size(struct i2c_client *i2c, u32 *size, u32 *rev)
{
	u32 reg_size;
	int ret, i;

	reg_size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;

	/*
	 * Sending 1 byte address size in Turin cause unrecoverable error
	 * Before trying to switch to 1 bytes, retry. 
	 */
	for (i = 0; i < MAX_RETRY; i++) {
		ret = sbrmi_i2c_reg_read(i2c, reg_size, rev);
		if (ret != I3C_I2C_MSG_XFER_SIZE) {
			usleep_range(10000, 20000);
			continue;
		} else {
			break;
		}
	}

	/*
	 * Validate the rev value is not 0xFF, as this value can be incorrectly cached
	 * when executing 2-byte address operations on SBRMI rev 0x10.
	 * For SBRMI rev 0x20 an error is returned.
	 */
	if (ret != I3C_I2C_MSG_XFER_SIZE || *rev == 0xFF) {
		reg_size = SBRMI_REG_ADDR_SIZE_DEF;
		ret = sbrmi_i2c_reg_read(i2c, reg_size, rev);
		if (ret != I3C_I2C_MSG_XFER_SIZE) {
			pr_err("I2C reg read failed with return value:%d\n", ret);
			return ret;
		}
	}

	return update_reg_addr_size(*rev, size);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int sbrmi_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *rmi_id)
#else
static int sbrmi_i2c_probe(struct i2c_client *client)
#endif
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct apml_sbrmi_device *rmi_dev;
	const char *name;
	int ret;

	rmi_dev = sbrmi_dev_alloc(dev);
	if (!rmi_dev)
		return -ENOMEM;

	atomic_set(&rmi_dev->in_progress, 0);
	atomic_set(&rmi_dev->no_new_trans, 0);
	rmi_dev->client = client;
	mutex_init(&rmi_dev->lock);

	dev_set_drvdata(dev, (void *)rmi_dev);

	rmi_dev->dev_static_addr = client->addr;

	ret = sbrmi_init_dimm_config(dev, rmi_dev);
	if (ret)
		return ret;

	switch (rmi_dev->dev_static_addr) {
	case 0x3c:
		name = devm_kasprintf(dev, GFP_KERNEL, "sbrmi_%s", "0.0");
		break;
	case 0x38:
		name = devm_kasprintf(dev, GFP_KERNEL, "sbrmi_%s", "1.0");
		break;
	default:
		name = devm_kasprintf(dev, GFP_KERNEL, "sbrmi_%x",
				      rmi_dev->dev_static_addr);
		break;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev, name,
							 rmi_dev,
							 &sbrmi_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	init_completion(&rmi_dev->misc_fops_done);
	ret = create_misc_rmi_device(rmi_dev, dev);
	if (ret)
		return ret;

	/*
	 * Best-effort APML common registry hookup. Probe still succeeds if this
	 * fails (-EINVAL, -ENOMEM); hwmon and misc stay up but Alert_L will
	 * not dispatch alerts for this device until registration succeeds.
	 */
	ret = apml_register_device(rmi_dev, APML_RMI_DEVICE);
	if (ret != 0)
		dev_warn(dev, "Failed to register with ALERT_L common system: %d\n", ret);

	return 0;
}

static int sbrmi_i3c_reg_read(struct i3c_device *i3cdev, int reg_size, u32 *val)
{
	struct i3c_priv_xfer xfers[I3C_I2C_MSG_XFER_SIZE];
	int reg = SBRMI_REV;
	int val_size = SBRMI_REG_DATA_SIZE;

	xfers[0].rnw = false;
	xfers[0].len = reg_size;
	xfers[0].data.out = &reg;

	xfers[1].rnw = true;
	xfers[1].len = val_size;
	xfers[1].data.in = val;

	return i3c_device_do_priv_xfers(i3cdev, xfers, I3C_I2C_MSG_XFER_SIZE);
}

static int sbrmi_i3c_identify_reg_addr_size(struct i3c_device *i3cdev, u32 *size, u32 *rev)
{
	u32 reg_size;
	int ret, i;

	reg_size = SBRMI_REG_ADDR_SIZE_TWO_BYTE;
	for (i = 0; i < MAX_RETRY; i++) {
		ret = sbrmi_i3c_reg_read(i3cdev, reg_size, rev);
		if (ret < 0) {
			usleep_range(10000, 20000);
			continue;
		} else {
			break;
		}
	}

	/*
	 * Validate the rev value is not 0xFF, as this value can be incorrectly cached
	 * when executing 2-byte address operations on SBRMI rev 0x10.
	 * For SBRMI rev 0x20 an error is returned.
	 */
	if (ret < 0 || *rev == 0xFF) {
		reg_size = SBRMI_REG_ADDR_SIZE_DEF;
		ret = sbrmi_i3c_reg_read(i3cdev, reg_size, rev);
		if (ret < 0) {
			pr_err("I3C reg read failed with return value:%d\n", ret);
			return ret;
		}
	}

	return update_reg_addr_size(*rev, size);
}

static int init_rmi_regmap(struct apml_sbrmi_device *rmi_dev, u32 size, u32 rev)
{
	struct regmap_config sbrmi_regmap_config = {
		.reg_bits = 8 * size,
		.val_bits = 8,
		.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	};
	struct regmap *regmap;

	if (rmi_dev->i3cdev) {
		regmap = devm_regmap_init_i3c(rmi_dev->i3cdev,
					      &sbrmi_regmap_config);
		if (IS_ERR(regmap)) {
			dev_err(&rmi_dev->i3cdev->dev,
				"Failed to register i3c regmap %d\n",
				(int)PTR_ERR(regmap));
			return PTR_ERR(regmap);
		}
	} else if (rmi_dev->client) {
		regmap = devm_regmap_init_i2c(rmi_dev->client,
					      &sbrmi_regmap_config);
		if (IS_ERR(regmap))
			return PTR_ERR(regmap);
	} else {
		return -ENODEV;
	}

	rmi_dev->regmap = regmap;
	rmi_dev->rev = rev;
	return 0;
}

/*
 * configure_regmap call should happen in probe, however if the server is power off,
 * regmap configuration may fail and hence driver probe will fail.
 * configure the regmap in hwmon/ioctl
 */
static int configure_regmap(struct apml_sbrmi_device *rmi_dev)
{
	u32 size;
	u32 rev = 0;
	int ret = 0;

	if (rmi_dev->i3cdev) {
		ret = sbrmi_i3c_identify_reg_addr_size(rmi_dev->i3cdev, &size, &rev);
		if (ret < 0) {
			pr_err("Reg size identification failed with return value:%d\n", ret);
			return ret;
		}
	} else if (rmi_dev->client) {
		ret = sbrmi_i2c_identify_reg_addr_size(rmi_dev->client, &size, &rev);
		if (ret < 0) {
			pr_err("Reg size identification failed with return value:%d\n", ret);
			return ret;
		}
	} else {
		return ret;
	}
	ret = init_rmi_regmap(rmi_dev, size, rev);
	return ret;
}

static int sbrmi_i3c_probe(struct i3c_device *i3cdev)
{
	struct device *dev = &i3cdev->dev;
	struct device *hwmon_dev;
	struct apml_sbrmi_device *rmi_dev;
	const char *name;
	int ret;

	if (!(I3C_PID_INSTANCE_ID(i3cdev->desc->info.pid) == 1 ||
	    i3cdev->desc->info.pid == PID_RMI_GENOA_TURIN))
		return -ENXIO;

	rmi_dev = sbrmi_dev_alloc(dev);
	if (!rmi_dev)
		return -ENOMEM;

	atomic_set(&rmi_dev->in_progress, 0);
	atomic_set(&rmi_dev->no_new_trans, 0);
	rmi_dev->i3cdev = i3cdev;
	mutex_init(&rmi_dev->lock);

	dev_set_drvdata(dev, (void *)rmi_dev);

	/*
	 * I3C dynamic address (post-DAA); fall back to static if
	 * not yet assigned
	 */
	if (i3cdev->desc->info.dyn_addr)
		rmi_dev->dev_static_addr = i3cdev->desc->info.dyn_addr;
	else
		rmi_dev->dev_static_addr = i3cdev->desc->info.static_addr;

	ret = sbrmi_init_dimm_config(dev, rmi_dev);
	if (ret)
		return ret;

	switch (rmi_dev->dev_static_addr) {
	case 0x3c:
		name = devm_kasprintf(dev, GFP_KERNEL, "sbrmi_%s", "0.0");
		break;
	case 0x38:
		name = devm_kasprintf(dev, GFP_KERNEL, "sbrmi_%s", "1.0");
		break;
	default:
		name = devm_kasprintf(dev, GFP_KERNEL, "sbrmi_%x",
				      rmi_dev->dev_static_addr);
		break;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev, name, rmi_dev,
							 &sbrmi_chip_info, NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	init_completion(&rmi_dev->misc_fops_done);
	ret = create_misc_rmi_device(rmi_dev, dev);
	if (ret)
		return ret;

	/*
	 * Best-effort APML common registry hookup. Probe still succeeds if this
	 * fails (-EINVAL, -ENOMEM); hwmon and misc stay up but Alert_L will
	 * not dispatch alerts for this device until registration succeeds.
	 */
	ret = apml_register_device(rmi_dev, APML_RMI_DEVICE);
	if (ret != 0)
		dev_warn(dev, "Failed to register with ALERT_L common system: %d\n", ret);

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int sbrmi_i2c_remove(struct i2c_client *client)
#else
static void sbrmi_i2c_remove(struct i2c_client *client)
#endif
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(&client->dev);

	if (!rmi_dev)
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
		return 0;
#else
		return;
#endif

	/*
	 * Set the no_new_trans so no new transaction can
	 * occur in sbrmi_ioctl
	 */
	atomic_set(&rmi_dev->no_new_trans, 1);
	/*
	 * If any transaction is in progress wait for the
	 * transaction to get complete
	 * Max wait for 3 sec for any pending transaction to
	 * complete
	 */
	if (atomic_read(&rmi_dev->in_progress))
		wait_for_completion_timeout(&rmi_dev->misc_fops_done,
					    MAX_WAIT_TIME_SEC * HZ);

	/* Unregister from APML common system */
	apml_unregister_device(rmi_dev, APML_RMI_DEVICE);
	misc_deregister(&rmi_dev->sbrmi_misc_dev);
	/* Assign fops and parent of misc dev to NULL */
	rmi_dev->sbrmi_misc_dev.fops = NULL;
	rmi_dev->sbrmi_misc_dev.parent = NULL;

	dev_info(&client->dev, "Removed sbrmi driver\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
static int sbrmi_i3c_remove(struct i3c_device *i3cdev)
#else
static void sbrmi_i3c_remove(struct i3c_device *i3cdev)
#endif
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(&i3cdev->dev);

	if (!rmi_dev)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
		return 0;
#else
		return;
#endif

	/*
	 * Set the no_new_trans so no new transaction can
	 * occur in sbrmi_ioctl
	 */
	atomic_set(&rmi_dev->no_new_trans, 1);
	/*
	 * If any transaction is in progress wait for the
	 * transaction to get complete
	 * Max wait for 3 sec for any pending transaction to
	 * complete
	 */
	if (atomic_read(&rmi_dev->in_progress))
		wait_for_completion_timeout(&rmi_dev->misc_fops_done,
					    MAX_WAIT_TIME_SEC * HZ);
	/* Unregister from APML common system */
	apml_unregister_device(rmi_dev, APML_RMI_DEVICE);
	misc_deregister(&rmi_dev->sbrmi_misc_dev);
	/* Assign fops and parent of misc dev to NULL */
	rmi_dev->sbrmi_misc_dev.fops = NULL;
	rmi_dev->sbrmi_misc_dev.parent = NULL;

	dev_info(&i3cdev->dev, "Removed sbrmi_i3c driver\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0)
	return 0;
#endif
}

static const struct i2c_device_id sbrmi_id[] = {
	{"sbrmi", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sbrmi_id);

static const struct of_device_id __maybe_unused sbrmi_of_match[] = {
	{
		.compatible = "amd,sbrmi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sbrmi_of_match);

static const struct i3c_device_id sbrmi_i3c_id[] = {
	I3C_DEVICE_EXTRA_INFO(0x112, 0x0, 0x2, NULL), /* Genoa/Turin */
	I3C_DEVICE_EXTRA_INFO(0x112, 0x0, 0x118, NULL), /* Socket:0, IOD:0 Venice*/
	I3C_DEVICE_EXTRA_INFO(0x112, 0x100, 0x118, NULL), /* Socket:1 IOD:0 Venice */
	{}
};
MODULE_DEVICE_TABLE(i3c, sbrmi_i3c_id);

static struct i2c_driver sbrmi_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "sbrmi",
		.of_match_table = of_match_ptr(sbrmi_of_match),
	},
	.probe = sbrmi_i2c_probe,
	.remove = sbrmi_i2c_remove,
	.id_table = sbrmi_id,
};

static struct i3c_driver sbrmi_i3c_driver = {
	.driver = {
		.name = "sbrmi_i3c",
	},
	.probe = sbrmi_i3c_probe,
	.remove = sbrmi_i3c_remove,
	.id_table = sbrmi_i3c_id,
};

module_i3c_i2c_driver(sbrmi_i3c_driver, &sbrmi_driver)

MODULE_AUTHOR("Akshay Gupta <akshay.gupta@amd.com>");
MODULE_AUTHOR("Naveenkrishna Chatradhi <naveenkrishna.chatradhi@amd.com>");
MODULE_DESCRIPTION("Hwmon driver for AMD SB-RMI emulated sensor");
MODULE_LICENSE("GPL");
