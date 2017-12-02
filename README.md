A few examples of code for [the bat board], using [libopencm3].

Most of them are just modified examples from the official [libopencm3 examples].

They aren't too specific for the bat boards, except the usual pinout considerations. What's sometimes not so easy to change/adapt is the chip. These examples are for STM32 F072 and L052.

## Short summary for the examples

### usb_cdcacm

The board presents itself as a serial port (CDC). It echoes back what it receives; toggling the LED after each received packet. Pressing the `PA1` button will print `hello, world` over this port.

### usb_dfu

L052 only. This is the DFU bootloader that I use on the bat boards with L052. More info about it on [the bat board] main page.

## General setup notes

You are expected to a C compiler (for ARM) and some development tools (e.g. make and git) installed and ready to use. You can get a [toolchain from arm.com]; or use one provided by your system package manager (if you're on a normal system like some linux, bsd or mac). For the development tools, use your package manager.

[libopencm3] is supposed to go into the `libopencm3` directory; it's git submodule if you're going the git way; otherwise just unpack it there. You'll need to run `make` in the `libopencm3` directory once, to compile the library. Any subsequent compilation of the examples here will just use these precompiled objects. This means that if you update, or make any changes to, libopencm3 sources, you'll need to recompile it explicitly like this. In other words, running `make` in any of the examples it not going to recompile the library.

From time to time there are breaking changes in the library (e.g. L0 support is relatively new), so no guarantees that things compile (although they should with the commit referenced for the submodule). From time to time I pull in the new updates, but again, no guarantees. (Timestamp 2017-12-02T18:41:26).

To compile an example, `cd` into its directory and run `make -f Makefile.l0btld` to compile. The three versions of the `Makefile`s are for STM32F072CBT6 (f0), STM32L052C8T6 (l0), STM32L052C8T6 with DFU bootloader occupying the first 8k of the flash (more about this setup on [the bat board] page). Cleaning up (important if you're switching to a different chip!) is done with the usual `make clean`.

I didn't set up flashing to the board into the `Makefiles`, so flashing should be done manually. Here are the most common options:

### DFU bootloader

This requires [dfu-util] software. Put the board into DFU mode (on F072 by holding the `PA1` button during reset; on L052 by shorting `GND` and `PB3` during reset). The command is then
```sh
dfu-util -d 0483:df11  -a 0 -s 0x08002000 -D cdcacm.bin
```
where the adjustable parts are of course the name of the firmware (`cdcacm.bin` above) and importantly the address: it should be `0x08002000` for L052 with the 8k DFU bootloader, otherwise `0x08000000`).

### built-in USART bootloader

This requires a means of communicating over serial port (ideally 3.3V levels; although the required pins are 5V tolerant on L052 and F072). This usually means an external hardware, an USB-to-serial converter (there are loads on e.g. ebay; the most commonly used chips are FT231X, FT232RL, CP2102, CH340G). Although on boards like the Pi you can use the built-in UART.

The built-in USART bootloader talks on PA9(TX) and PA10(RX) (and some others; but these are conveniently placed on the bat board). It is entered by holding the `PA1` button during a reset.

On the software side, I prefer [stm32flash]. The command is then
```sh
stm32flash -e 255 -v -w cdcacm.bin /dev/ttyUSB0
```
where the adjustable parts are of course firmware filename and the serial port (`/dev/ttyUSB0` above).




[the bat board]: https://flabbergast.github.io/bat-board
[libopencm3]: https://github.com/libopencm3/libopencm3
[libopencm3 examples]: https://github.com/libopencm3/libopencm3-examples
[toolchain from arm.com]: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
[dfu-util]: http://dfu-util.sourceforge.net/


License (for the original bits): CC-by-SA 2.0

