# README

This example implements a USB Device Firmware Upgrade (DFU) bootloader
to demonstrate the use of the USB device stack.

It is intended to run on STM32L0 chips; this particular instance on STM32L052C8Tx.

## Usage notes

The bootloader has been tested on a STM32L052C8T7, it takes 8kB of the flash space. It has been tested with [dfu-util] 0.9 on Mac OS X. The usual command is something like

        dfu-util -s 0x8002000 -D <binary_fw>

The "user firmware" needs to be compiled to start at 0x8002000 (i.e. the vector table should be there), and set `SCB_VTOR` to 0x8002000 immediately after starting.

## Implementation notes

To modify for another chip, the particular places to check are:

 * `MCU_PAGE_SIZE` define: check the reference manual for page size (usually in the section "NVM organization"). The code assumes that the page size is the same for the whole part where the bootloader will write (i.e. from 0x8002000 to the end of flash).
 * The dfuse string in `usb_strings`. This describes the various parts of the flash, and the DfuSe app or dfu-util will use this to determine e.g. the page size, or which parts of the flash are writable.
 * The clock setup in `main()`.
 * The easy bits in `main()`: which pin to be checked (and how) for forcing the bootloader, which pin to blink on and how fast to blink.

 * The `PROGRAM_BY_HALF_PAGE` define decides whether to write to flash word-by-word, or a half-a-page at once.
 * `hardfault.c` hooks into the hard fault handler, and defines some variables to make it easy(er) to figure out what caused the fault, in e.g. gdb. To enable this, just uncomment it in `OBJS` in the `Makefile`.

## Beware

The programming speed is about 3.2Kb/sec (compared to 1440 bytes/sec over the ROM USART bootloader using 115200 baud; and to 3K-6K/sec via SWD). It is achieved by pretending (over DFU) that the page size is 1K (instead of the internal 128 bytes) - the bottleneck turned out to be the USB.

Also it uses writing-half-a-page-at-once from RAM, instead of the (slower) option of writing word-by-word. (With word-by-word, pretending 1K blocks, it's 950 bytes/sec, with word-by-word OR half-a-page, but native 128 byte blocks. it's 420 bytes/sec).

## Credits

[Original code]: Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>

Modifications to run on STM32L0: (C) 2016 flabbergast <flabbergast@drak.xyz>

[Original code]: https://github.com/libopencm3/libopencm3-examples/tree/8924042d2acd8b0a9dd7c307a3ea925cca87d554/examples/stm32/f1/other/usb_dfu
[dfu-util]: http://dfu-util.sourceforge.net/
