# Copyright (c) 2020 Endian Technologies AB
#
# SPDX-License-Identifier: Apache-2.0

ifneq ($(BOARD),p8)
BOARD   := $(shell cat build/zephyr/.config 2>/dev/null | grep CONFIG_BOARD= \
                       | cut -d'"' -f 2 2>/dev/null)
ifndef BOARD
BOARD := pinetime
endif
endif

ifneq ($(BOOTLOADER),n)
BOOTLOADER := $(shell cat build/zephyr/.config 2>/dev/null \
                      | grep BOOTLOADER_MCUBOOT \
                      | cut -d'=' -f 2 | tr -cd '[ny]\n')
endif

BUILD   := build/zephyr/zephyr.bin
VERSION := $(shell git describe --tags --dirty)
IMGDIR  := images
IMAGE   := $(IMGDIR)/$(BOARD)-hypnos-$(VERSION)-mcuboot-app-img.bin
PACKAGE := $(IMGDIR)/$(BOARD)-hypnos-$(VERSION)-mcuboot-app-dfu.zip

.PHONY: build clean help tools

ifneq  ($(BOOTLOADER),n)
dfu: $(BUILD) $(IMAGE) $(PACKAGE)
image: $(BUILD) $(IMAGE)

$(IMAGE): $(BUILD) | $(IMGDIR)
	@echo "Creating an MCUBoot app image"
	bootloader/mcuboot/scripts/imgtool.py create --align 4 --version 1.0.0 \
	--header-size 512 --slot-size 475136 $(BUILD) $(IMAGE)
endif
build:
	west build -p -b $(BOARD) app/hypnos

help:
	@echo "Build and flash the Hypnos application firmware as well as"
	@echo "images and DFU packages for devices running Lup Yuen Lee's"
	@echo "custom MCUBoot bootloader\n"
	@echo "Usage:"
	@echo "  make [TARGET] [BOARD=p8]\n"
	@echo "Targets:"
	@echo "  build    build application firmware"
	@echo "  clean    remove the build directory"
	@echo "  dfu      build a Device Firmare Upgrade package"
	@echo "  flash    flash the most recent image over the SWD interface"
	@echo "  help     show this message and exit"
	@echo "  image    build an MCUBoot app image"
	@echo "  print    print various strings for debugging this Makefile"
	@echo "  tools    install tools for creating images and DFU packages"

tools:
	@echo "Installing tools for creating app images and DFU packages"
	pip3 install --user setuptools
	pip3 install --user -r bootloader/mcuboot/scripts/requirements.txt
	pip3 install --user pyocd
	@echo "Done"

$(IMGDIR):
	mkdir $(IMGDIR)

$(BUILD):
	west build -p -b $(BOARD) app/hypnos

$(PACKAGE): $(IMAGE)
	@echo "Creating a Device Firmware Update package"
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application $(IMAGE) \
	$(PACKAGE)

ifneq ($(BOOTLOADER),n)
flash: $(BUILD) $(IMAGE)
	@echo "Flashing the last modified Hypnos image after the bootloader"
	pyocd flash -e sector -a 0x8000 -t nrf52 $(shell ls -t images/*.bin | head -1)
else
flash: $(BUILD)
	@echo "Flashing Hypnos firmware without bootloader support"
	west flash
endif

clean:
	rm -rf build/

print:
	@echo "Board:      $(BOARD)"
	@echo "Bootloader: $(BOOTLOADER)"
	@echo "Version:    $(VERSION)"
	@echo "Build:      $(BUILD)"
	@echo "Image:      $(IMAGE)"
	@echo "Package:    $(PACKAGE)"
