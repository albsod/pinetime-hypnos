# PineTime Hypnos

This is a [Zephyr](https://www.zephyrproject.org/)-based firmware for the
[PineTime](https://www.pine64.org/pinetime/) smartwatch focused on low power consumption.

It started as a fork of najnesnaj's [firmware toolkit](https://github.com/najnesnaj/pinetime-zephyr)
and is inspired by [PineTime Hermes](https://github.com/Dejvino/pinetime-hermes-firmware).

> **Hypnos**, son of Night and Darkness</br>
> He is said to be a calm and gentle god, as he helps humans in need and, due to their sleep, owns
> half of their lives.<sup>[1](https://en.wikipedia.org/wiki/Hypnos)</sup>


## Features

- [x] Power management by design: CPU idle in main loop; all functionality triggered by interrupts
- [x] Debug output via JLink RTT
- [x] Display
- [x] Touch sensor: tap to light up the display
- [x] Button: press to light up the display
- [x] Clock: accurately increment current time
- [x] Build current time of host machine into the firmware image
- [x] Battery: show state of charge (%) and whether it's charging
- [x] Graphics: show time, date and battery status using LittlevGL
- [ ] Set alarm
- [ ] Set current time from BLE-connected device
- [ ] Show notifications from BLE-connected device
- [ ] Firmware update over BLE
- [ ] Quick glance via lift-to-wake
- [ ] Wrist vibration
- [ ] Step counting


## Getting started

Follow Zephyr's [Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html)
up to step 3) "Get the source code". Here you should run the commands below
instead of the ones in the guide:

```
$ mkdir ~/pinetime-hypnos
$ cd ~/pinetime-hypnos
$ west init -m https://github.com/endian-albin/pinetime-hypnos
$ west update
```

Then complete the remaining steps.

To build and install the application, run

```
$ west build -p -b pinetime hypnos
$ west flash
```


## Copying

The Hypnos application is under the Mozilla Public License 2.0 and
the documentation, including this README, is CC BY-SA 3.0.

The graphics library is under MIT/Expat and everything else (including Zephyr)
should be under Apache 2.0. Look for SPDX lines and check the source tree
history to be sure.

MPL 2.0 is a file-based copyleft license compatible with all commonly used
GNU licenses (LGPL/GPL/AGPL) as well as Apache 2.0.

![MPL compatibility](https://opensource.com/sites/default/files/styles/image-full-size/public/lead-images/OSCD_MPL2_520x292_FINAL.png?itok=6vv4XnEz)
<br />Image by Opensource.com
