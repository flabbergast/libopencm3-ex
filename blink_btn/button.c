/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
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
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>

#define LED_GPIO GPIOA
#define LED_PIN GPIO15
#define LED_RCC_GPIO RCC_GPIOA
#define BUTTON_GPIO GPIOA
#define BUTTON_PIN GPIO1
#define BUTTON_RCC_GPIO RCC_GPIOA

/* Set the clock to max speed. */
static void clock_setup(void)
{
#if defined(STM32F0)
	rcc_clock_setup_in_hsi_out_48mhz();
#elif defined(STM32L0)
	/* After a reset, the system uses MSI@2.1MHz. */
	/* end result: 32MHz PLLVCO from HSI16,
	 *	 no system/periph clock divide.
	 */

	/* increase the latency to 1 wait state (we'll be speeding up) */
	flash_set_ws(1);

	/* turn on HSI16 */
	rcc_osc_on(RCC_HSI16);
	rcc_wait_for_osc_ready(RCC_HSI16);

	/* run AHB, APB1, APB2 at full speed */
	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre1(RCC_CFGR_PPRE1_NODIV);
	rcc_set_ppre2(RCC_CFGR_PPRE2_NODIV);

	/* turn off PLL and wait for it to fully stop */
	rcc_osc_off(RCC_PLL);
	while (RCC_CR & RCC_CR_PLLRDY);

	/* set PLL source to HSI16 */
	RCC_CFGR &= ~(1<<16); // RCC_CFGR_PLLSRC

	/* set up PLL */
	rcc_set_pll_multiplier(RCC_CFGR_PLLMUL_MUL4);
	rcc_set_pll_divider(RCC_CFGR_PLLDIV_DIV2);

	/* turn on and switch to PLL */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_ahb_frequency = 32000000;
	rcc_apb1_frequency = 32000000;
	rcc_apb2_frequency = 32000000;
#else
	#error "Implement a clock setup."
#endif
}

static void gpio_setup(void)
{
	/* Enable clocks. */
	rcc_periph_clock_enable(LED_RCC_GPIO);
	rcc_periph_clock_enable(BUTTON_RCC_GPIO);

	/* Set the LED pin to 'output push-pull'. */
	gpio_mode_setup(LED_GPIO, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, LED_PIN);

	/* Set the button pin to 'input' (external pull-down). */
	gpio_mode_setup(BUTTON_GPIO, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BUTTON_PIN);
}

#define SCB_VTOR MMIO32(0xE000ED08)

int main(void)
{
#if defined(BOOTLOADER8K)
	SCB_VTOR = (uint32_t) 0x08002000;
#endif

	int i;

	clock_setup();
	gpio_setup();

	/* Blink the LED on the board. */
	while (1) {
		gpio_toggle(LED_GPIO, LED_PIN);

		/* Upon button press, blink more slowly. */
		if (gpio_get(BUTTON_GPIO, BUTTON_PIN)) {
			for (i = 0; i < 2000000; i++) {	/* Wait a bit. */
				__asm__("nop");
			}
		}

		for (i = 0; i < 2000000; i++) {		/* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}

// vim: tabstop=4:shiftwidth=4:noexpandtab
