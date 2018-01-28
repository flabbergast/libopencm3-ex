This is STM32F072 port of jcw's **SerPlus**:

- https://github.com/jeelabs/embello/tree/master/explore/1649-f103/serplus
- https://jeelabs.org/article/1649f/
- https://github.com/jeelabs/folie


## Modifications

- runs on F072, aimed for [the bat board](https://flabbergast.github.io/bat-board/), with
  [SerPlus shield], which has some extra features, so:
- button controlled mode: either "SerPlus mode" or straight "raw UART with DTR"
- WS2812 neopixel status LED
- control on/off output power with a FET
- measure output power level using ADC
- these can be turned off via #defines in the code

In "raw" mode, it works as any other USB-to-serial converter (with DTR), so it can
be used to upload firmware to arduino-type boards (tested).



## Original Serplus README

**SerPlus** is an STM32F103-based bridge between USB and a serial port.

Inline telnet escape codes can be used to control DTR, RTS, and parity.

Based on the `usart1_irq_printf` and `usb_cdcacm` code from [libopencm3-examples][EX] (LGPL3).

For the RAM-based version, to be loaded at $20002000, use:

    make -f Makefile-ram

   [EX]: https://github.com/libopencm3/libopencm3-examples/tree/master/examples/stm32/f1/stm32-h103

