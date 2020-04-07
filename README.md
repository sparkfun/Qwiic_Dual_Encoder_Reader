SparkFun Qwiic Dual Encoder Reader
===========================================================

Firmware for an ATTiny84 that will read two quadrature encoders of your choice, and report readings (aka counts) over I2C.

This Firmware is used on the SparkFun Auto pHAT for Raspberry Pi.

![SparkFun Auto pHAT for Raspberry Pi](https://cdn.sparkfun.com//assets/parts/1/5/0/3/5/16328-SparkFun_Auto_pHAT_for_Raspberry_Pi-01.jpg)

[*SparkFun Auto pHAT for Raspberry Pi*](https://www.sparkfun.com/products/16328)


Sometimes you just need to read two encoders. Whether it's for a pair of motors on a driving robot, or for multiple user inputs. This firmware, programmed on an ATTiny84, will offload all the work needed to read those encoders. 

One rotation in the clockwise direction increases the respective channel count by 96 and -96 in the counter clockwise direction. The number of 'ticks' or steps the encoder has turned on both channels is readable over I2C. 

The I2C address of Qwiic Dual Encoder Reader is software configurable which means you can hookup over 100 on a single I2C bus!

New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).


Repository Contents
-------------------

* **/Documents** - Datasheet for the ATTiny84
* **/Firmware/Qwiic_Dual_Encoder_Reader** - Firmware for the Qwiic Dual Encoder Reader

Documentation
--------------
* **[Qwiic Dual Encoder Reader Python Module](https://github.com/sparkfun/Qwiic_Dual_Encoder_Reader_Py)** - Python pip install supported!
* **[SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)** - Basic information on how to install drivers for the qwiic ecosystem.

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please visit the [SparkFun Forum](https://forum.sparkfun.com/index.php) and post a topic. For more general questions related to our qwiic system, please visit this section of the forum: [SparkFun Forums: QWIIC SYSTEMS](https://forum.sparkfun.com/viewforum.php?f=105)

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release any derivative under the same license.

Distributed as-is; no warranty is given.
- Your friends at SparkFun.
