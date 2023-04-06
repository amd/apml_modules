.. SPDX-License-Identifier: GPL-2.0
# amd apml modules (apml_sbtsi, apml_sbrmi and apml_alert)

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
power_cap.


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
Disclaimer: apml_alert module is currently experimental and may change in the future

EPYC processors from AMD provide APML ALERT_L for BMC users to monitor events.

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

apml_alertl module defines an interface for user space to register their PID for
notifications and an ISR which identifies the source of the interrupt and signals
user space application.

apml_alertl module depends on apml_sbrmi module for identifying the source.

DTS node definition for Alert_L module
--------------------------------------

required:
  - compatible
  - status
  - gpios: GPIO associated with the Alert_L of the socket
  - sbrmi: Array of RMI devices on the system

Example:

/ {
	/* Alert_L associated with socket 0 */
	 alertl_sock0 {
		compatible = "apml-alertl";
		status = "okay";
		gpios = <&gpio0 ASPEED_GPIO(I, 7) GPIO_ACTIVE_LOW>;
		sbrmi = <&sbrmi_p0_1 &sbrmi_p1_1>;
	};

	/* Alert_L associated with socket 1 */
	alertl_sock1 {
		compatible = "apml-alertl";
		status = "okay";
		gpios = <&gpio0 ASPEED_GPIO(U, 4) GPIO_ACTIVE_LOW>;
		sbrmi = <&sbrmi_p1_1 &sbrmi_p0_1>;
	};
};

Loading
-------
To install apml_alertl driver builtin as module, user can use the modprobe or insmod
command

#> sudo modprobe apml_alertl

#> sudo insmod ./apml_alertl.ko

Note: Dependency on apml_sbrmi.ko module

Unloading
---------

#> sudo rmmod apml_alertl

If the driver is inbuilt can be removed/inserted by running bind/unbind command.

#> cd /sys/bus/platform/drivers/alertl
#> echo alertl_rmi# > unbind/bind

USAGE
-----

User need to register the PID of the process with the apml_alertl module,
by writing the PID to the debugfs entry, /sys/kernel/debug/apml_alert/ras_fatal_pid

#> echo $PID > /sys/kernel/debug/apml_alert/$alert_source

User application needs to wait for the signal from the apml_alertl module.

Signal from module carries a "struct kernel_siginfo" with following data

- si_int: is filled with the event data
  [15:0]  = ras_status register
  [23:16] = rmi static address

- si_signo: 44

Currently the kernel driver send signal only in event, RAS status register bit set.

- In case of RAS fatal error the status register BIT(1) will
  set, and ISR clears the bit to avoid interfere with alerts during the ISR.

Future versions of the driver may include support for other events mentioned above.
