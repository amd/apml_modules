.. SPDX-License-Identifier: GPL-2.0
# amd apml modules (apml_sbtsi, apml_sbrmi and apml_alertl)

amd-apml: APML interface drivers for BMC

EPYC processors from AMD provide APML interface for
BMC users to monitor and configure the system parameters
via the Advanced Platform Management List (APML) interface
defined in EPYC processor PPR.

This chapter defines custom protocols over i2c/i3c bus
  - Mailbox
  - CPUID [RO]
  - MCA MSR {RO]
  - RMI/TSI register [RW]

module_i2c_i3c based sbrmi and sbtsi modules, which
are probed as i2c or i3c client devices, depending on the
platforms DTS.

https://developer.amd.com/resources/epyc-resources/epyc-specifications

APMl library provides C API fo the user space application on top of this
module.


Disclaimer
===========

The amd apml modules are supported only on AMD Family 19h (including
third-generation AMD EPYC processors (codenamed "Milan")) or later
CPUs. Using the amd apml modules on earlier CPUs could produce unexpected
results, and may cause the processor to operate outside of your motherboard
or system specifications. Correspondingly, defaults to only executing on
AMD Family 19h Model (0h ~ 1Fh & 30h ~ 3Fh) server line of processors.

Interface
---------

Both apml_sbtsi and apml_sbrmi modules register a misc_device
to provide ioctl interface to user space, allowing them
to run these custom protocols.

apml_sbtsi module registers hwmon sensors for monitoring
current temperature, managing max and min thresholds.

apml_sbrmi module registers hwmon sensors for monitoring
power_cap_max, current power consumption and managing
power_cap. It also reports per-UMC DIMM thermal sensor
temperatures (TS0 and TS1) when platform firmware supports
mailbox command 0x48 (SBRMI_READ_DIMM_THERMAL_SENSOR).


DIMM Thermal Sensors (apml_sbrmi)
=================================

Each populated UMC/DDRPHY instance exposes two on-DIMM thermal
sensors, TS0 and TS1. The driver reads them via the SB-RMI mailbox
and publishes up to 32 hwmon temperature channels per socket:

  - Channels  0-15: TS0 for UMC instances 0-15
  - Channels 16-31: TS1 for UMC instances 0-15

Each channel exposes temp(N+1)_input (milli-degrees Celsius) and
temp(N+1)_label in sysfs, where N is the 0-based hwmon channel index.
Labels follow the pattern DIMM_TS0_UMCn and DIMM_TS1_UMCn (n = 0..15).
Only channels corresponding to populated UMC instances are visible;
unpopulated channels return -EINVAL.

Mode 1 DIMM_ADDRESS encoding (mailbox data byte):

  - Bit[7]:   1 (Mode 1)
  - Bit[6]:   TS select (0 = TS0, 1 = TS1)
  - Bit[3:0]: UMC/DDRPHY instance ID (0-15)

When the optional dimm-ids device-tree property is omitted, the
driver derives addresses using the legacy 0x80-based encoding for
all 16 UMC instances (0x80/0xC0 for UMC0 TS0/TS1, 0x81/0xC1 for
UMC1, and so on). When dimm-ids is present, each entry supplies
the TS0 base address for one UMC in array order; the driver sets
bit[6] to form the corresponding TS1 address (TS1 = TS0 | 0x40).

DTS node definition for apml_sbrmi DIMM thermal sensors
---------------------------------------------------------

The apml_sbrmi node is probed as an I2C or I3C client depending on
the platform bus wiring. See Documentation/amd,sbrmi.yaml for the
full devicetree binding schema.

required:
  - compatible: must be "amd,sbrmi"
  - reg: I2C slave address or I3C dynamic address

optional:
  - dimm-ids: array of TS0 mailbox addresses, one per populated UMC
    instance (1..16 entries). Entry count sets how many UMC
    instances expose TS0 and TS1 hwmon channels.

examples:

&i3c4 {
	sbrmi_p0_sp8: sbrmi@0,2240000111A {
		reg = <0x0 0x224 0x0000111A>;
		assigned-address = <0x3c>;
		/* Sixteen populated UMC instances; alternating TS0 layout */
		dimm-ids = <0x80 0x90 0x81 0x91 0x82 0x92 0x83 0x93
			    0x84 0x94 0x85 0x95 0x86 0x96 0x87 0x97>;
	};
};

/* Eight populated UMC instances on a single socket */
&i3c4 {
	sbrmi@0,2240000111A {
		reg = <0x0 0x224 0x0000111A>;
		assigned-address = <0x3c>;
		dimm-ids = <0x80 0x90 0x81 0x91 0x82 0x92 0x83 0x93>;
	};
};

/* Legacy encoding: omit dimm-ids to use 0x80-based addresses */
&i2c15 {
	sbrmi@3c {
		compatible = "amd,sbrmi";
		reg = <0x3c>;
	};
};

After probe, DIMM temperatures appear under the hwmon device. Each
hwmon channel N (0-based) maps to temp(N+1)_input and temp(N+1)_label
in sysfs, for example:

#> cat /sys/class/hwmon/hwmonN/temp1_input    /* channel 0, DIMM_TS0_UMC0 */
#> cat /sys/class/hwmon/hwmonN/temp1_label
#> cat /sys/class/hwmon/hwmonN/temp17_input   /* channel 16, DIMM_TS1_UMC0 */
#> cat /sys/class/hwmon/hwmonN/temp17_label


Build and Install
-----------------

Kernel development packages for the running kernel need to be installed
prior to building the amd apml modules. A Makefile is provided which should
work with most kernel source trees.

To cross compile for arm based BMC

export CC=arm-openbmc-linux-gnueabi-gcc # Or similar
export ARCH=arm
KDIR=<Path to prebuilt BMC kernel>

To build the kernel module:

#> make

To install the kernel module:

#> sudo make modules_install

To clean the kernel module build directory:

#> make clean


Note: There is a fix required in the upstream linux kerenl header to handle
 the i3c_dev. the patch is kept in patches/ folder of this repo.

Loading
-------

If the apml modules were installed you should use the modprobe command to
load the module.

#> sudo modprobe apml_sbrmi apml_sbtsi

The apml modules can also be loaded using insmod if the module was not
installed:

#> sudo insmod ./apml_sbrmi.ko
#> sudo insmod ./apml_sbtsi.ko

APML_ALERTL
===========
Disclaimer: The apml_alertl module is currently experimental and may change in the future.

EPYC processors from AMD provide APML ALERT_L for BMC users to monitor
events.

   |-------------------|
   | socket       SBRMI|==== i2c/i3c bus
   |              SBTSI|==== i2c/i3c bus
   |            Alert_L|---- gpio line
   |-------------------|

APML Alert_L is asserted in multiple events:
1) Machine Check Exception occurs within the system
2) The processor alerts the SBI on system fatal error event
3) Set by hardware as a result of a 0x71/0x72/0x73 command completion
4) Set by firmware to indicate the completion of a mailbox operation
5) Temperature Alert

Driver Implementation Design Update
-----------------------------------

The driver has been redesigned to provide a more robust and standardized
interface for user-space alert handling, offering richer context compared
to the previous signal-based approach.


When Alert_L asserts, **apml_alertl** runs its threaded interrupt handler,
locks the global **apml_devices** list (owned by **apml_common**), and
visits every registered node. For each **SBRMI** entry it reads the RAS
status register; for each **SBTSI** entry it reads the temperature alert
status register.

**apml_sbrmi** and **apml_sbtsi** register with **apml_common** during their
probe paths; **apml_alertl** finds peers only through that registry.

**Planned upstream binding:** the global **apml_devices** registry is interim.
Alert_L will probe as an auxiliary driver; alert handling will then run in
auxiliary bind/unbind for each SB-RMI/SB-TSI device instead of a list walk.

User space is notified via **kobject_uevent_env()** (**KOBJ_CHANGE**), not
signals or debugfs.

apml_alertl depends on **apml_common.ko** being loaded. SBRMI/SBTSI devices
must be registered (typically by loading **apml_sbrmi.ko** and
**apml_sbtsi.ko**) before alerts can be attributed to a concrete device.

Key Changes
-----------

This update aims to enhance the robustness and flexibility of alert handling:

 - Improved Interface: The new design enhances alert handling by providing
   detailed context and improving interaction with user-space applications.
 - I3C Hot-Join Support: The new design will support hot-join for I3C
   devices, allowing for dynamic device connections.
 - Legacy Support: The previous implementation is available on the
   alertl-legacy branch for those who need it, though this branch will be
   phased out over time.

DTS node definition for Alert_L module
--------------------------------------

required:
  - compatible: "apml-alertl"
  - status
  - gpios: GPIO line wired to Alert_L for this socket (consumer binding)

optional:
  - socket-num: 8-bit value (`/bits/ 8` encoding in device tree); if present,
    the threaded IRQ is named **apml_irq** with that number as a suffix
    (e.g. **apml_irq0**). If omitted, the IRQ name is **apml_irq** (useful on
    multi-socket systems when you need distinct /proc/interrupts entries per
    socket). Use `/bits/ 8 <N>`, the driver reads this property with
    `of_property_read_u8()`.

Example:

/ {
	/* Alert_L associated with socket 0 */
	alertl_sock0 {
		compatible = "apml-alertl";
		status = "okay";
		gpios = <&gpio0 ASPEED_GPIO(I, 7) GPIO_ACTIVE_LOW>;
		socket-num = /bits/ 8 <0>;
	};

	/* Alert_L associated with socket 1 */
	alertl_sock1 {
		compatible = "apml-alertl";
		status = "okay";
		gpios = <&gpio0 ASPEED_GPIO(U, 4) GPIO_ACTIVE_LOW>;
		socket-num = /bits/ 8 <1>;
	};
};

Loading
-------
Load **apml_common** and the bus drivers so devices register, then
**apml_alertl**:

#> sudo modprobe apml_common
#> sudo modprobe apml_sbrmi apml_sbtsi
#> sudo modprobe apml_alertl

Or with insmod from a build tree (order matters):

#> sudo insmod ./apml_common.ko
#> sudo insmod ./apml_sbrmi.ko
#> sudo insmod ./apml_sbtsi.ko
#> sudo insmod ./apml_alertl.ko

Unloading
---------

Unload in reverse dependency order, e.g.:

#> sudo rmmod apml_alertl
#> sudo rmmod apml_sbtsi apml_sbrmi
#> sudo rmmod apml_common

If the driver is built in, the platform device can be unbound/rebound under:

#> cd /sys/bus/platform/drivers/apml_alertl
#> echo <device-name> > unbind
#> echo <device-name> > bind

USAGE (uevents)
---------------

Applications should listen for **change** uevents on the Alert_L platform
device (for example via **udev** rules or **libudev**), not for POSIX signals.

Each notification includes these environment variables (see **apml_alertl.c**):

| Variable   | Meaning |
|------------|---------|
| SOURCE     | For **RMI** (RAS): RAS status register (**0x4C**). For **TSI**
|            | (temperature): TSI status left-shifted 24 bits. |
| BUS_NUM    | Linux bus number (**I3C** bus **id** or **I2C** adapter
|            | **nr**). |
| PID        | **I3C** provisioned ID (**64-bit** hex); **0** for **I2C**
|            | devices. |
| ADDRESS    | **I3C** static address or **I2C** client address (**hex**). |

For RAS alerts, the driver clears the RAS status register and writes
**RAS_ALERT_ASYNC** to the RMI status register (**0x2**) as part of servicing
the event.
