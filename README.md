# Hypnos

This is a work-in-progress [Zephyr](https://www.zephyrproject.org/)-based firmware for the
[PineTime](https://www.pine64.org/pinetime/) smartwatch. Contributions are welcome!

<img src="app/hypnos/hypnos-photo.png" title="Background image: Night and Sleep by Evelyn De Morgan (1878)" width="240px" height="240px">

> **Hypnos**, son of Night and Darkness</br>
> He is said to be a calm and gentle god, as he helps humans in need and, due to their sleep, owns
> half of their lives.<sup>[1](https://en.wikipedia.org/wiki/Hypnos)</sup>

## Features

- [x] 100 % Free Software
- [x] Battery life: about one week
- [x] Battery status: get state of charge and whether it's charging
- [x] Clock: accurately increment current time
- [x] Time and date synchronization with Bluetooth-connected device
- [x] Touch sensor: tap to light up the display; swipe to display version information
- [x] LVGL graphics: show time, date, battery and Bluetooth status
- [x] Support for the PineTime bootloader and over-the-air firmware upgrades
- [x] Optional debug output via JLink RTT
- [ ] Show notifications from Bluetooth-connected device
- [ ] Set alarm
- [ ] Wrist vibration
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

Optionally enable RTT logging and/or disable bootloader support:
```
$ export BOOTLOADER="off"
$ export LOGGING="on"
```

### Build

Build the firmware for either `pinetime` or `p8` (replace `<board>` below):
```
$ cd app/
$ west build -p -b <board> hypnos
```

### Install

Generate an mcuboot app image:
```
$ cd ../bootloader/mcuboot/scripts
$ pip3 install --user setuptools
$ pip3 install --user -r requirements.txt
$ ./imgtool.py create --align 4 --version 1.0.0 --header-size 512 --slot-size 475136 ../../../app/build/zephyr/zephyr.bin hypnos-mcuboot-app.bin
```
Connect your in-circuit programmer to the SWD pins on the watch.

Flash the image to offset 0x8000:
```
pyocd flash -e sector -a 0x8000 -t nrf52 hypnos-dfu-app.bin
```

If you have disabled bootloader support, flash the application directly using pyocd:
```
$ west flash
```
...or run `west flash --context` for more options.


### Build and install the bootloader

To install the compatible PineTime bootloader, follow Lup Yuen's [build instructions](https://lupyuen.github.io/pinetime-rust-mynewt/articles/mcuboot#build-and-flash-mcuboot-bootloader)
or [fetch the prebuilt binary](https://github.com/lupyuen/pinetime-rust-mynewt/releases/tag/v5.0.0).

Then flash it to the beginning of the internal memory:

```
pyocd flash -e sector -t nrf52 bootloader-image.bin
```

## Firmware upgrade over Bluetooth LE

### SMP

Hypnos supports firmware image management over the Simple Management Protocol.

To make use of this feature, get the [mcumgr](https://github.com/apache/mynewt-mcumgr#command-line-tool) command-line tool.
Then run the commands below to list, upload, test and confirm firmware images over BLE:

```
# mcumgr --conntype="ble" --connstring ctlr_name=hci0,peer_name='Hypnos' image list
# mcumgr --conntype="ble" --connstring ctlr_name=hci0,peer_name='Hypnos' image upload hypnos-mcuboot-app.bin
# mcumgr --conntype="ble" --connstring ctlr_name=hci0,peer_name='Hypnos' image test <hash of slot-1 image>
# mcumgr --conntype="ble" --connstring ctlr_name=hci0,peer_name='Hypnos' reset
# mcumgr --conntype="ble" --connstring ctlr_name=hci0,peer_name='Hypnos' image confirm
```

If you are unhappy with the new image, simply run the `reset` command again instead of `image confirm` to revert to the old one.
[See this document for more information](https://docs.zephyrproject.org/latest/samples/subsys/mgmt/mcumgr/smp_svr/README.html).

### DFU / InfiniTime

To install Hypnos over the air from [InfiniTime](https://github.com/JF002/Pinetime) you need to create a SoftDevice DFU zip package and upload that.

Install adafruit-nrfutil:
```
$ pip3 install --user wheel
$ pip3 install --user setuptools
$ pip3 install --user adafruit-nrfutil
```

Create the DFU zip package from the imgtool.py output above:
```
$ adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application hypnos-mcuboot-app.bin hypnos-mcuboot-app-dfu.zip
```

Connect to InfiniTime and upload `hypnos-mcuboot-app-dfu.zip`.

## Copying

This software may be used under the terms of the Apache License 2.0,
unless explicitly stated otherwise.

The documentation contained in this README and on the wiki are under
the CC BY-SA 4.0 license.
