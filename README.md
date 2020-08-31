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
- [x] Optional support for the PineTime bootloader
- [x] Optional debug output via JLink RTT
- [ ] Show notifications from Bluetooth-connected device
- [ ] Set alarm
- [ ] Wrist vibration
- [ ] Firmware update over Bluetooth
- [ ] Quick glance via lift-to-wake (requires a free driver for the accelerometer)
- [ ] Step counting (see above)

## Getting started

### Set up the development environment

Follow Zephyr's [Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html)
up to step 3.2 "Get the Zephyr source code". Here you should run the commands below
instead of the ones in the guide:
```
$ git clone https://github.com/endian-albin/pinetime-hypnos
$ cd pinetime-hypnos
$ west init -l app/
$ west update
```

Then complete the remaining steps under section 3 and 4.

### Configure

Optionally enable bootloader support and/or RTT logging:
```
$ export BOOTLOADER="on"
$ export RTT_LOG="on"
```

### Build

Build the firmware for either `pinetime` or `p8` (replace `<board>` below):
```
$ cd app/
$ west build -p -b <board> hypnos
```

### Install

Connect your "in-circuit" programmer to the SWD pins on the watch.

Program using pyocd:
```
$ west flash
```

...or run `west flash --context` for more options.

If you have built Hypnos with bootloader support, you will need to follow the steps below instead.

Generate a [DFU](https://docs.zephyrproject.org/2.3.0/guides/device_mgmt/dfu.html) image:
```
$ cd ../bootloader/mcuboot/scripts
$ pip3 install --user setuptools
$ pip3 install --user -r requirements.txt
$ ./imgtool.py create --align 4 --version 1.0.0 --header-size 512 --slot-size 475136 ../../../app/build/zephyr/zephyr.bin hypnos-dfu-app.bin
```

Flash the image to offset 0x8000:
```
pyocd flash -e sector -a 0x8000 -t nrf52 hypnos-dfu-app.bin
```

### Build and install the bootloader

To install the bootloader used by PineTime, follow Lup Yuen's [build instructions](https://lupyuen.github.io/pinetime-rust-mynewt/articles/mcuboot#build-and-flash-mcuboot-bootloader)
or [fetch the prebuilt binary](https://lupyuen.github.io/pinetime-rust-mynewt/articles/mcuboot#build-and-flash-mcuboot-bootloader).

Then flash it to the beginning of the internal memory:

```
pyocd flash -e sector -t nrf52 bootloader-image.bin
```

## Copying

This software may be used under the terms of the Apache License 2.0,
unless explicitly stated otherwise.

The documentation contained in this README and on the wiki are under
the CC BY-SA 4.0 license.
