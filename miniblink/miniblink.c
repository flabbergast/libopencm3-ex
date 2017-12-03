/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define LED_GPIO GPIOA
#define LED_PIN GPIO15
#define LED_RCC_GPIO RCC_GPIOA

// MCUs run at different speeds, so adjust to your liking
#if defined(STM32F0)
#define DELAY_CONSTANT 100000
#elif defined(STM32L0)
#define DELAY_CONSTANT 100000
#elif defined(STM32F1)
#define DELAY_CONSTANT 2000000
#endif /* STM32F1 */

static void gpio_setup(void)
{
	/* Enable the correct clock. */
	/* Using API functions; for how to do it "manually" see libopencm3-examples/.../miniblink */
	rcc_periph_clock_enable(LED_RCC_GPIO);

	/* Set LED_PIN to 'output push-pull'. */
#if defined(STM32F0) || defined(STM32L0)
	gpio_mode_setup(LED_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
#elif defined(STM32F1)
	gpio_set_mode(LED_GPIO, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
#endif /* STM32F1 */
}

int main(void)
{
	int i;

#if defined(BOOTLOADER8K)
	// need to relocate the vector table if the bootloader is in flash
#		define SCB_VTOR MMIO32(0xE000ED08)
	SCB_VTOR = (uint32_t) 0x08002000;
#endif

#if defined(STM32F1)
	// disable JTAG, leave SWD (the bat board has LED on one of the JTAG pins)
	rcc_periph_clock_enable(RCC_AFIO);
	AFIO_MAPR |= (1<<25); // not universal code; assumes RESET AFIO_MAPR state 

	rcc_clock_setup_in_hse_16mhz_out_72mhz(); // also test if the ext crystal works
#endif

	gpio_setup();

	/* Blink the LED on the board. */
	while (1) {
		gpio_toggle(LED_GPIO, LED_PIN);
		// manually set: GPIOA_BRR = GPIO15;
		// manually clear: GPIOA_BSRR = GPIO15;
		for (i = 0; i < DELAY_CONSTANT; i++)
			__asm__("nop");
	}

	return 0;
}

// vim: tabstop=4:shiftwidth=4:noexpandtab
