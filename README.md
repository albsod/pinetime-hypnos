# PineTime Hypnos

This is a WIP [Zephyr RTOS](https://www.zephyrproject.org/) firmware for the
[PineTime](https://www.pine64.org/pinetime/) smartwatch.

It started as a fork of najnesnaj's [firmware toolkit](https://github.com/najnesnaj/pinetime-zephyr)
and is inspired by [PineTime Hermes](https://github.com/Dejvino/pinetime-hermes-firmware).

> **Hypnos**, son of Night and Darkness</br>
> He is said to be a calm and gentle god, as he helps humans in need and, due to their sleep, owns
> half of their lives.<sup>[1](https://en.wikipedia.org/wiki/Hypnos)</sup>

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

## License

All code and documentation are licensed under the Apache 2.0 (see LICENSE),
unless stated otherwise.
