#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include <libopencm3/stm32/usart.h>
#include "serial_helper.h"


static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

#if defined(STM32F0)
	/* USART1 pins: PA9/TX PA10/RX */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9|GPIO10);
#elif defined(STM32L0)
	/* USART1 pins: PA9/TX PA10/RX */
	gpio_set_af(GPIOA, GPIO_AF4, GPIO9|GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO9|GPIO10);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
#endif

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	//usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Set PA15 for LED */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
}

static void i2c_setup(void)
{
	uint32_t i2c;
#if defined(STM32F0)
	/* Enable clocks for I2C2. */
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_af(GPIOB, GPIO_AF1, GPIO10|GPIO11);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10|GPIO11);

	i2c = I2C2;

#elif defined(STM32L0)
	/* Enable clocks for I2C1. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8|GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO8|GPIO9);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8|GPIO9);

	i2c = I2C1;
#endif /* STM32L0 */
	i2c_reset(i2c);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(i2c);

	/* setup from libopencm3-examples */
	i2c_enable_analog_filter(i2c);
	i2c_set_digital_filter(i2c, I2C_CR1_DNF_DISABLED);
	i2c_set_speed(i2c, i2c_speed_sm_100k, 8);
	i2c_enable_stretching(i2c);
	i2c_set_7bit_addr_mode(i2c);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(i2c);
}


#if defined(STM32L0)
static void clock_setup_l0(void)
{
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
}
#endif /* STM32L0 */

int main(void)
{
#if defined(BOOTLOADER8K)
#	define SCB_VTOR MMIO32(0xE000ED08)
	SCB_VTOR = (uint32_t) 0x08002000;
#endif /* BOOTLOADER8K */

#if defined(STM32F0)
	uint32_t i2c = I2C2;
#elif defined(STM32L0)
	uint32_t i2c = I2C1;
#endif /* STM32L0 */
	uint8_t eeprom_addr = 0x50;

#if defined(STM32F0)
	rcc_clock_setup_in_hsi_out_48mhz();
#elif defined(STM32L0)
	clock_setup_l0();
#endif
	gpio_setup();
	usart_setup();
	i2c_setup();

	/* Send a message on USART1. */
	pstr("stm\r\n");

	uint8_t temp[10];
	uint8_t wb[2];

	// one byte addresses for smaller epproms
	temp[0] = 0;
	// bytes to write
	temp[1] = 0x11;
	temp[2] = 0x22;
	temp[3] = 0x33;

	// write
	i2c_transfer7(i2c, eeprom_addr, temp, 4, NULL, 0);

	uint32_t t;
	// the pause needs to be long enough for the eeprom to finish
	// writing the data above; otherwise the 'read' call below
	// hangs
	t = 500000;
	while(t-- > 0)
		__asm__("nop");

	// two byte addresses for bigger eeproms (change also transfer7 call!)
	//wb[0] = (uint8_t)( 0 >>8);
	//wb[1] = (uint8_t)( 0 &0xFF);
	// one byte addresses for smaller epproms (change also transfer7 call!)
	wb[0] = 0;

	// read
	i2c_transfer7(i2c, eeprom_addr, wb, 1, temp, 10);

	gpio_set(GPIOA, GPIO15); /* LED on */

	uint8_t i;
	for(i=0; i<10; i++)
		phex(temp[i]);
	pent();

	while (1); /* Halt. */

	return 0;
}

// vim: tabstop=4:shiftwidth=4:noexpandtab
