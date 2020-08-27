# Hypnos

This is a work-in-progress [Zephyr](https://www.zephyrproject.org/)-based firmware for the
[PineTime](https://www.pine64.org/pinetime/) and P8 smartwatches. Contributions are welcome!

<img src="app/hypnos/hypnos-photo.png" title="Background image: Night and Sleep by Evelyn De Morgan (1878)" width="240px" height="240px">

> **Hypnos**, son of Night and Darkness</br>
> He is said to be a calm and gentle god, as he helps humans in need and, due to their sleep, owns
> half of their lives.<sup>[1](https://en.wikipedia.org/wiki/Hypnos)</sup>

## Features

- [x] 100 % Free Software
- [x] Battery life: about one week
- [x] Battery status: get state of charge and whether it's charging
- [x] Clock: accurately increment current time
- [x] Button: press to toggle time synchronization with Bluetooth-connected device
- [x] Touch sensor: tap to light up the display
- [x] LVGL graphics: show time, date, battery and Bluetooth status
- [x] Optional debug output via JLink RTT
- [x] Optional support for the PineTime bootloader
- [ ] Show notifications from Bluetooth-connected device
- [ ] Set alarm
- [ ] Wrist vibration
- [ ] Firmware update over Bluetooth
- [ ] Quick glance via lift-to-wake (requires a free driver for the accelerometer)
- [ ] Step counting (see above)

## Getting started

### Setting up the development environment

Follow Zephyr's [Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html)
up to step 3.2 "Get the Zephyr source code". Here you should run the commands below
instead of the ones in the guide:
```
$ git clone https://github.com/endian-albin/pinetime-hypnos
$ cd pinetime-hypnos
$ west init -l app/
$ west update
```

Then complete the remaining steps.

### Building

Optionally enable logging and/or bootloader support:
```
$ export LOGGING="on"
$ export BOOTLOADER="on"
```

Build the firmware for either `pinetime` or `p8` (replace `<board>` below):
```
$ cd app/
$ west build -p -b <board> hypnos
```

### Installing without bootloader

Install/program using a JLink:
```
$ west flash
```

If you have access to an ST-Link v2 or Raspberry Pi, you can program `build/zephyr/zephyr.bin` to offset 0 by following Lup Yuen's
[OpenOCD instructions](https://lupyuen.github.io/pinetime-rust-mynewt/articles/mcuboot#select-the-openocd-interface-st-link-or-raspberry-pi-spi).

### Installing with bootloader support

If you have built Hypnos with bootloader support, you will need to follow the steps below instead.

Generate an image containing the MCUBoot header:
```
$ cd ../bootloader/mcuboot/scripts
$ pip3 install --user setuptools
$ pip3 install --user -r requirements.txt
$ ./imgtool.py create --align 4 --version 1.0.0 --header-size 256 --slot-size 475136 ../../../app/build/zephyr/zephyr.bin hypnos-mcuboot-app-img.bin
```

Install using OpenOCD:
```
program hypnos-mcuboot-app-img.bin 0x8000
```

or JLink:
```
$ /opt/SEGGER/JLink/JLinkExe -device nrf52 -if swd -speed 4000 -autoconnect 1
J-Link> loadbin hypnos-mcuboot-app-img.bin, 0x8000
```

To install the bootloader itself, follow Lup Yuen's own [instructions](https://github.com/lupyuen/pinetime-rust-mynewt/releases/tag/v4.1.7).


## Copying

This software may be used under the terms of the Apache License 2.0,
unless explicitly stated otherwise.

The documentation contained in this README and on the wiki are under
the CC BY-SA 4.0 license.
