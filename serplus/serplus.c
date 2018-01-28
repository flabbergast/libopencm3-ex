/* {{{ LICENSES
 * Original serplus code by jcw https://github.com/jeelabs/embello/tree/master/explore/1649-f103/serplus
 * Modified for F072 by flabbergast
 */

/*
 * WS2821 code originally from https://github.com/hwhw/stm32-projects
 */

/*
 * This code is derived from example code in the libopencm3 project:
 *
 *	https://github.com/libopencm3/libopencm3-examples/tree/master/
 *			examples/stm32/f1/stm32-h103/usart_irq_printf
 *	and		examples/stm32/f1/stm32-h103/usb_cdcacm
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2010, 2013 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2016 Jean-Claude Wippler <jc@wippler.nl>
 *
 * This code is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this code.  If not, see <http://www.gnu.org/licenses/>.
 * }}} */


// {{{ software configuration
#define HAVE_MODES
#define HAVE_ADC
#define HAVE_NEOPIX
// }}}


// {{{ includes
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/adc.h>
// }}}


// {{{ global variables
#define BUFFER_SIZE 256
struct ring input_ring, output_ring;
uint8_t input_ring_buffer[BUFFER_SIZE], output_ring_buffer[BUFFER_SIZE];
volatile uint32_t ticks;
uint8_t extbut_state, extbut_laststate, extbut_changed, extbut_acted;
uint32_t extbut_time, extbut_lastchange;
#ifdef HAVE_MODES
volatile uint16_t usart_mode; // messes up the USB driver if it's uint8_t
#endif // HAVE_MODES
volatile uint32_t usart_serplus_baudrate, usart_serplus_databits, usart_serplus_stopbits, usart_serplus_parity, usart_serplus_flowcontrol;
// }}}


// {{{ ringbuffer
/******************************************************************************
 * Simple ringbuffer implementation from open-bldc's libgovernor that
 * you can find at:
 * https://github.com/open-bldc/open-bldc/tree/master/source/libgovernor
 *****************************************************************************/

typedef int32_t ring_size_t;

struct ring {
	uint8_t *data;
	ring_size_t size;
	uint32_t begin;
	uint32_t end;
};

#define RING_SIZE(RING)  ((RING)->size - 1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin == (RING)->end)

static void ring_init(struct ring *ring, uint8_t *buf, ring_size_t size)
{
	ring->data = buf;
	ring->size = size;
	ring->begin = 0;
	ring->end = 0;
}

static int32_t ring_write_ch(struct ring *ring, uint8_t ch)
{
	if (((ring->end + 1) % ring->size) != ring->begin) {
		ring->data[ring->end++] = ch;
		ring->end %= ring->size;
		return (uint32_t)ch;
	}

	return -1;
}

static int32_t ring_write(struct ring *ring, uint8_t *data, ring_size_t size)
{
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_write_ch(ring, data[i]) < 0)
			return -i;
	}

	return i;
}

static int32_t ring_read_ch(struct ring *ring, uint8_t *ch)
{
	int32_t ret = -1;

	if (ring->begin != ring->end) {
		ret = ring->data[ring->begin++];
		ring->begin %= ring->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}

static int32_t ring_read(struct ring *ring, uint8_t *data, ring_size_t size)
{
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_read_ch(ring, data + i) < 0)
			return i;
	}

	return -i;
}

// }}}


// {{{ clock setup
static void clock_setup(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();
	crs_autotrim_usb_enable();
	rcc_set_usbclk_source(RCC_HSI48);

	// for gpio, usart
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART1);

#ifdef HAVE_NEOPIX
	// for WS2812 (pwm/dma)
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_DMA);
#endif // HAVE_NEOPIX

#ifdef HAVE_ADC
	// ADC
	rcc_periph_clock_enable(RCC_ADC);
#endif // HAVE_ADC
}
// }}}


// {{{ gpio
// Bat Board
#define LED_GPIO GPIOA
#define LED_PIN  GPIO15
#define LED_ON gpio_set(LED_GPIO, LED_PIN)
#define LED_OFF gpio_clear(LED_GPIO, LED_PIN)
#define LED_TOGGLE gpio_toggle(LED_GPIO, LED_PIN)
#define BOOTBUT_GPIO GPIOA
#define BOOTBUT_PIN  GPIO1
#define BOOTBUT_PRESSED (gpio_get(BOOTBUT_GPIO,BOOTBUT_PIN)!=0)
#define EXTBUT_GPIO  GPIOA
#define EXTBUT_PIN	 GPIO6
#define EXTBUT_PRESSED (gpio_get(EXTBUT_GPIO,EXTBUT_PIN)==0)
#define FET_ON (gpio_clear(GPIOB,GPIO3))
#define FET_OFF (gpio_set(GPIOB,GPIO3))

#define RTS_GPIO GPIOB
#define RTS_PIN  GPIO11
#define DTR_GPIO GPIOB
#define DTR_PIN  GPIO10

static void usart_dtr_rts_serplus_setup(void) {
	gpio_mode_setup(DTR_GPIO, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, DTR_PIN);	// serplus: dtr: default high
	gpio_set(DTR_GPIO, DTR_PIN);
	gpio_mode_setup(RTS_GPIO, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, RTS_PIN);	// serplus: rts: default low
	gpio_clear(RTS_GPIO, RTS_PIN);
}

#ifdef HAVE_MODES
static void usart_dtr_rts_raw_setup(void) {
	gpio_mode_setup(DTR_GPIO, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, DTR_PIN);	// serplus: dtr: default high
	gpio_set(DTR_GPIO, DTR_PIN);
	gpio_mode_setup(RTS_GPIO, GPIO_MODE_INPUT,
			GPIO_PUPD_NONE, RTS_PIN);	// serplus: rts ignored: floating
}
#endif // HAVE_MODES

static void gpio_setup(void) {
	gpio_mode_setup(LED_GPIO, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, LED_PIN);	// bat: LED
	LED_OFF;
	gpio_mode_setup(BOOTBUT_GPIO, GPIO_MODE_INPUT,
			GPIO_PUPD_NONE, BOOTBUT_PIN);	  // bat: button (has ext pull-down)
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO3);		// serplus: P-FET
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_LOW, GPIO3);
	usart_dtr_rts_serplus_setup();

	// serplus: button (int pull-up)
	gpio_mode_setup(EXTBUT_GPIO, GPIO_MODE_INPUT,
			GPIO_PUPD_PULLUP, EXTBUT_PIN);
	extbut_time = ticks;
	extbut_state = EXTBUT_PRESSED;
	extbut_laststate = extbut_state;
	extbut_changed = 0;
	extbut_lastchange = extbut_time;
	extbut_acted = 0;
}

#define DEBOUNCE_MS 50
static uint8_t service_extbut(void) {
	static uint8_t pinval;
	pinval = EXTBUT_PRESSED;
	if(ticks - extbut_lastchange < DEBOUNCE_MS) {
		extbut_time = ticks;
		extbut_changed = 0;
		return extbut_state;
	} else {
		extbut_laststate = extbut_state;
		extbut_state = pinval;
		extbut_time = ticks;
		if(extbut_state != extbut_laststate) {
			extbut_lastchange = ticks;
			extbut_changed = 1;
		} else {
			extbut_changed = 0;
		}
		return extbut_state;
	}
}
// }}}


// {{{ WS2812
/**************************************************
 ******************** WS2812 **********************
 **************************************************/
#ifdef HAVE_NEOPIX


// maximum is at about 4000
#define LED_COUNT 1
// minimum ID offset is 0x100 (first ID byte mustn't be 0x00)
#define ID_OFFSET 0xA000
// for the purposes of computing the delays
#define CPU_MHZ 48

#define TICK_NS (1000/CPU_MHZ)
#define WS0 (350 / TICK_NS)
#define WS1 (800 / TICK_NS)
#define WSP (1300 / TICK_NS)
#define WSL (20000 / TICK_NS)

#define DMA_BANK_SIZE 2 * 8 * 3 // 2 = number of LEDs; 8*3 = bits*colours
#define DMA_SIZE (DMA_BANK_SIZE*2)
static uint8_t dma_data[DMA_SIZE];
static volatile uint32_t led_data[LED_COUNT];
static volatile uint32_t led_cur = 0;


static void pwm_setup(void) {
	// Configure GPIOs: OUT=PA7
	gpio_mode_setup(GPIOA, GPIO_MODE_AF,
		GPIO_PUPD_NONE, GPIO7 );
	gpio_set_af(GPIOA, GPIO_AF1, GPIO7 );

	timer_reset(TIM3);

	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_disable_oc_output(TIM3, TIM_OC2);
	timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
	timer_disable_oc_clear(TIM3, TIM_OC2);
	timer_set_oc_value(TIM3, TIM_OC2, 0);
	timer_enable_oc_preload(TIM3, TIM_OC2);
	timer_set_oc_polarity_high(TIM3, TIM_OC2);
	timer_enable_oc_output(TIM3, TIM_OC2);

	timer_set_dma_on_update_event(TIM3);
	timer_enable_irq(TIM3, TIM_DIER_UDE); // in fact, enable DMA on update

	timer_enable_preload(TIM3);
	timer_continuous_mode(TIM3);
	timer_set_period(TIM3, WSP);

	timer_enable_counter(TIM3);
}

static void populate_dma_data(uint8_t *dma_data_bank) {
	for(int i=0; i<DMA_BANK_SIZE;) {
		led_cur = led_cur % (LED_COUNT+3);
		if(led_cur < LED_COUNT) {
			uint32_t v = led_data[led_cur];
			for(int j=0; j<24; j++) {
				dma_data_bank[i++] = (v & 0x800000) ? WS1 : WS0;
				v <<= 1;
			}
		} else {
			for(int j=0; j<24; j++) {
				dma_data_bank[i++] = 0;
			}
		}
		led_cur++;
	}
}

static void dma_int_enable(void) {
	// SPI1 TX on DMA1 Channel 3
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 0);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
}

/* Not used in this example
static void dma_int_disable(void) {
	nvic_disable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
}
*/

static int timer_dma(uint8_t *tx_buf, int tx_len)
{
	dma_int_enable();

	// Reset DMA channels
	dma_channel_reset(DMA1, DMA_CHANNEL3);

	// Set up tx dma
	dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&TIM_CCR2(TIM3));
	dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)tx_buf);
	dma_set_number_of_data(DMA1, DMA_CHANNEL3, tx_len);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_32BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);

	dma_enable_circular_mode(DMA1, DMA_CHANNEL3);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
	dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL3);
	dma_enable_channel(DMA1, DMA_CHANNEL3);

	return 0;
}

// SPI transmit completed with DMA
void dma1_channel2_3_isr(void)
{
	if ((DMA1_ISR & DMA_ISR_TCIF3) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF3;
		populate_dma_data(&dma_data[DMA_BANK_SIZE]);
	}
	if ((DMA1_ISR & DMA_ISR_HTIF3) != 0) {
		DMA1_IFCR |= DMA_IFCR_CHTIF3;
		populate_dma_data(dma_data);
	}
}

static void ws2812_pre(void) {
	memset(dma_data, 0, DMA_SIZE);
	memset((void*)led_data, 0, LED_COUNT*sizeof(*led_data));
	//populate_dma_data(dma_data);
	//populate_dma_data(&dma_data[DMA_BANK_SIZE]);
	timer_dma(dma_data, DMA_SIZE);
}

static void neopix_green(uint8_t intens) {
	led_data[0] = (led_data[0]&0x00FFFF)|(intens<<16);
}

static void neopix_red(uint8_t intens) {
	led_data[0] = (led_data[0]&0xFF00FF)|(intens<<8);
}

static void neopix_blue(uint8_t intens) {
	led_data[0] = (led_data[0]&0xFFFF00)|(intens);
}

#endif // HAVE_NEOPIX
// }}}


// {{{ usart
#ifdef HAVE_MODES
// global modes for USART
#define MODE_SERPLUS 1
#define MODE_RAW     2
#endif // HAVE_MODES

static void usart_set_serplus_params(void) {
	usart_set_baudrate(USART1, usart_serplus_baudrate);
	usart_set_databits(USART1, usart_serplus_databits);
	usart_set_stopbits(USART1, usart_serplus_stopbits);
	usart_set_parity(USART1, usart_serplus_parity);
	usart_set_flow_control(USART1, usart_serplus_flowcontrol);
}

static void usart_setup(void) {
	// Initialize input and output ring buffers. 
	ring_init(&input_ring, input_ring_buffer, BUFFER_SIZE);
	ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);

	// Disable the USART, in case this gets called more than once
	usart_disable(USART1);

	// Enable the USART1 interrupt.
	nvic_enable_irq(NVIC_USART1_IRQ);

	// Setup PA9 pin for USART1 transmit.
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);

	// Setup PA10 pin for USART1 receive.
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10); // weak pull-up avoids picking up noise
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO10);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

	// Setup UART parameters.
	usart_serplus_baudrate = 115200;
	usart_serplus_databits = 8;
	usart_serplus_stopbits = USART_STOPBITS_1;
	usart_serplus_parity = USART_PARITY_NONE;
	usart_serplus_flowcontrol = USART_FLOWCONTROL_NONE;

	usart_set_serplus_params();
	usart_set_mode(USART1, USART_MODE_TX_RX);

	// Enable USART1 Receive interrupt.
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	// Finally enable the USART.
	usart_enable(USART1);
}

// telnet escape codes and special values:
enum {
	IAC=255, WILL=251, SB=250, SE=240,
	CPO=44, SETPAR=3, SETCTL=5,
	PAR_NONE=1, PAR_ODD=2, PAR_EVEN=3,
	DTR_ON=8, DTR_OFF=9, RTS_ON=11, RTS_OFF=12,
};

void usart1_isr(void)
{
	// Check if we were called because of RXNE.
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
			((USART_ISR(USART1) & USART_ISR_RXNE) != 0)) {

		// Retrieve the data from the peripheral.
		uint8_t c = usart_recv(USART1);
		ring_write_ch(&input_ring, c);

		// Indicate that we got data.
#ifdef HAVE_NEOPIX
		neopix_green(c>>2);
#else // ! HAVE_NEOPIX
		LED_TOGGLE;
#endif // HAVE_NEOPIX

#ifdef HAVE_MODES
		if( usart_mode == MODE_SERPLUS ) {
#endif // HAVE_MODES
			// telnet: escape the escape character, i.e. send it twice
			if (c == IAC)
				ring_write_ch(&input_ring, c);
#ifdef HAVE_MODES
		}
#endif // HAVE_MODES
	}

	// Check if we were called because of TXE.
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
			((USART_ISR(USART1) & USART_ISR_TXE) != 0)) {

		int32_t data = ring_read_ch(&output_ring, NULL);

		if (data == -1) {
			// Disable the TXE interrupt, it's no longer needed.
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
		} else {
#ifdef HAVE_MODES
			if( usart_mode == MODE_SERPLUS ) {
#endif // HAVE_MODES
				// state machine to decode telnet request before sending it on
				static int state = 0;

				switch (state) {
					default: // default state
						if (data == IAC)
							state = 1;
						else
							usart_send(USART1, data);
						break;

					case 1: // IAC seen
						state = 0;
						if (data == IAC)
							usart_send(USART1, data);
						else
							state = data == SB ? 3 : data >= WILL ? 2 : 0;
						break;

					case 2: // IAC, WILL (or WONT/DO/DONT) seen
						state = 0;
						break;

					case 3: // IAC, SB seen
						state = data == CPO ? 4 : 5;
						break;

					case 4: // IAC, SB, CPO seen
						state = data == SETPAR ? 7 :
							data == SETCTL ? 8 : 5;
						break;

					case 5: // wait for IAC + SE
						if (data == IAC)
							state = 6;
						break;

					case 6: // wait for SE
						if (data != IAC)
							state = data == SE ? 0 : data == SB ? 3 : 5;
						break;

					case 7: // set parity
						state = 5;
						switch (data) {
							case PAR_NONE:
								usart_serplus_databits = 8;
								usart_serplus_parity = USART_PARITY_NONE;
								usart_disable(USART1);
								usart_set_serplus_params();
								usart_enable(USART1); break;
							case PAR_ODD:
								usart_serplus_databits = 9;
								usart_serplus_parity = USART_PARITY_ODD;
								usart_disable(USART1);
								usart_set_serplus_params();
								usart_enable(USART1); break;
							case PAR_EVEN:
								usart_serplus_databits = 9;
								usart_serplus_parity = USART_PARITY_EVEN;
								usart_disable(USART1);
								usart_set_serplus_params();
								usart_enable(USART1); break;
						}
						break;

					case 8: // set control
						state = 5;
						switch (data) {
							case DTR_ON:
								gpio_clear(DTR_GPIO, DTR_PIN);
								break;
							case DTR_OFF:
								gpio_set(DTR_GPIO, DTR_PIN);
								break;
							case RTS_ON:
								gpio_clear(RTS_GPIO, RTS_PIN);
								break;
							case RTS_OFF:
								gpio_set(RTS_GPIO, RTS_PIN);
								break;
						}
						break;
				}
#ifdef HAVE_MODES
			} else if( usart_mode == MODE_RAW ) {
				usart_send(USART1, data);
			}
#endif // HAVE_MODES
		}
	}
}
// }}}


// {{{ systick
static void systick_setup(void)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
	// clear counter so it starts right away
	STK_CVR = 0;

	systick_set_reload((rcc_ahb_frequency/8000)-1); // every ms
	systick_counter_enable();
	systick_interrupt_enable();
}

void sys_tick_handler(void)
{
	++ticks;
}
// }}}

 
// {{{ USB serial
static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x83,
		.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
		.wMaxPacketSize = 16,
		.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	},
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_CDC,
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
		.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
		.iInterface = 0,

		.endpoint = comm_endp,

		.extra = &cdcacm_functional_descriptors,
		.extralen = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,

		.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
		.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
		.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static char serial_no[9] = "killbill";

static const char *usb_strings[] = {
	"JeeLabs",
	"SerPlus",
	serial_no,
};

// Buffer to be used for control requests.
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
#ifndef HAVE_MODES
	(void)buf;
#endif // ! HAVE_MODES
	(void)usbd_dev;

	switch (req->bRequest) {
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
#ifdef HAVE_MODES
				if( usart_mode == MODE_RAW ) {
					if(req->wValue & (1 << 0)) { // dtr set
						gpio_clear(DTR_GPIO, DTR_PIN);
					} else {
						gpio_set(DTR_GPIO, DTR_PIN);
					}
					//if(req->wValue & (1 << 1)) { // rts set
					//}
				}
#else // HAVE_MODES
				// original code from SerPlus:
				/*
				 * This Linux cdc_acm driver requires this to be implemented
				 * even though it's optional in the CDC spec, and we don't
				 * advertise it in the ACM functional descriptor.
				 */
				char local_buf[10];
				struct usb_cdc_notification *notif = (void *)local_buf;

				// We echo signals back to host as notification.
				notif->bmRequestType = 0xA1;
				notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
				notif->wValue = 0;
				notif->wIndex = 0;
				notif->wLength = 2;
				local_buf[8] = req->wValue & 3;
				local_buf[9] = 0;
				// usbd_ep_write_packet(0x83, buf, 10);
#endif // HAVE_MODES
				return 1;
			}
		case USB_CDC_REQ_SET_LINE_CODING: {
				if (*len < sizeof(struct usb_cdc_line_coding))
					return 0;
#ifdef HAVE_MODES
				if( usart_mode == MODE_RAW ) {
					struct usb_cdc_line_coding *coding;
					coding = (struct usb_cdc_line_coding *)*buf;
					uint8_t databits;

					usart_disable(USART1);
					usart_set_baudrate(USART1,coding->dwDTERate);

					//do the databits ourselves, because the routine isn't yet adjusted to F072 (M1 reg)
					//usart_set_databits(USART1,coding->bDataBits);
					databits = coding->bDataBits;
					USART_CR1(USART1) &= ~(USART_CR1_M0|USART_CR1_M1);
					// at least 'stm32flash' expects the parity to be an extra bit in
					// addition to these; but the MCU includes them in "databits"
					// not sure what the actual USB_CDC standard is
					if((databits < 9) && (coding->bParityType == USB_CDC_EVEN_PARITY ||
							coding->bParityType == USB_CDC_ODD_PARITY)) {
						databits++;
					}
					switch(databits) {
						case 7: USART_CR1(USART1) |= USART_CR1_M1; break;
						case 9: USART_CR1(USART1) |= USART_CR1_M0; break;
					}

					switch(coding->bCharFormat) {
						case USB_CDC_1_STOP_BITS: usart_set_stopbits(USART1, USART_STOPBITS_1); break;
						case USB_CDC_1_5_STOP_BITS: usart_set_stopbits(USART1, USART_STOPBITS_1_5); break;
						case USB_CDC_2_STOP_BITS: usart_set_stopbits(USART1, USART_STOPBITS_2); break;
					}
					switch(coding->bParityType) {
						case USB_CDC_NO_PARITY: usart_set_parity(USART1, USART_PARITY_NONE); break;
						case USB_CDC_ODD_PARITY: usart_set_parity(USART1, USART_PARITY_ODD); break;
						case USB_CDC_EVEN_PARITY: usart_set_parity(USART1, USART_PARITY_EVEN); break;
					}
					usart_enable(USART1);
				}
#endif // HAVE_MODES
				return 1;
			}
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	// back pressure: don't read the packet if there's not enough room in ring
	if ((output_ring.begin - (output_ring.end+1)) % BUFFER_SIZE <= 64)
		return;

	uint8_t buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof buf);

	if (len) {
		// Retrieve the data from the peripheral.
		ring_write(&output_ring, buf, len);

		// Enable usart transmit interrupt so it sends out the data.
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	(void)usbd_dev;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
			usbd_dev,
			USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			cdcacm_control_request);
}

static char *get_dev_unique_id(char *s)
{
#if defined(STM32F4) || defined(STM32F2)
#	define UNIQUE_SERIAL_R 0x1FFF7A10
#	define FLASH_SIZE_R    0x1fff7A22
#elif defined(STM32F3)
#	define UNIQUE_SERIAL_R 0x1FFFF7AC
#	define FLASH_SIZE_R    0x1fff77cc
#elif defined(STM32L1)
#	define UNIQUE_SERIAL_R 0x1ff80050
#	define FLASH_SIZE_R    0x1FF8004C
#elif defined(STM32F0)
# define UNIQUE_SERIAL_R 0x1FFFF7AC
# define FLASH_SIZE_R	 0x1FFFF7CC
#else
#	define UNIQUE_SERIAL_R 0x1FFFF7E8;
#	define FLASH_SIZE_R    0x1ffff7e0
#endif
  volatile uint32_t *unique_id_p = (volatile uint32_t *)UNIQUE_SERIAL_R;
  uint32_t unique_id = *unique_id_p ^  // was "+" in original BMP
	  *(unique_id_p + 1) ^ // was "+" in original BMP
	  *(unique_id_p + 2);
	int i;

	// Calculated the upper flash limit from the exported data
	//  in theparameter block
	//max_address = (*(uint32_t *) FLASH_SIZE_R) <<10;
	// Fetch serial number from chip's unique ID
	for(i = 0; i < 8; i++) {
		s[7-i] = ((unique_id >> (4*i)) & 0xF) + '0';
	}
	for(i = 0; i < 8; i++)
		if(s[i] > '9')
			s[i] += 'A' - '9' - 1;
	s[8] = 0;

	return s;
}
// }}}


// {{{ ADC
/**************************************************
 ********************* ADC ************************
 **************************************************/

#ifdef HAVE_ADC
static void adc_setup(void) {
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG,
			GPIO_PUPD_NONE, GPIO4);		// serplus: detect pwr

	//gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	//gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	uint8_t channel_array[] = { 4 }; // which channels to enable // ADC_CHANNEL_TEMP 
	adc_set_regular_sequence(ADC1, 1, channel_array); // mid param = num_of_chan
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	// Wait for ADC starting up.
	int i;
	for (i = 0; i < 800000; i++) {	  // Wait a bit.
		__asm__("nop");
	}
}
#endif // HAVE_ADC

// }}}


// {{{ main
int main(void) {
	clock_setup();
	systick_setup();

	gpio_setup();

#ifdef HAVE_ADC
	uint16_t volts; // ADC reading (PA4, 10k/10k divider to PWR_OUT)
	adc_setup();
#endif // HAVE_ADC

#ifdef HAVE_MODES
	// begin in serplus mode
	usart_mode = MODE_SERPLUS;
#endif // HAVE_MODES

#ifdef HAVE_NEOPIX
	// WS2812 setup
	ws2812_pre();
	pwm_setup();
#endif // HAVE_NEOPIX

	FET_ON;

	for (int i = 0; i < 1000000; i++)
		__asm__("");

	get_dev_unique_id(serial_no);

	usbd_device *usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, &config,
			usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	for (int i = 0; i < 1000000; i++)
		__asm__("");

	usart_setup(); // late config to allow USB setup to complete first

	LED_TOGGLE;

	while (1) {
		// poll USB while waiting for 2 ms to elapse
		// it takes 2.7 ms to send 64 bytes at 230400 baud 8N1
		for (int i = 0; i < 2; ++i) {
			uint32_t lastTick = ticks;
			while (ticks == lastTick)
				usbd_poll(usbd_dev);
		}

		// put up to 64 pending bytes into the USB send packet buffer
		uint8_t buf[64];
		int len = ring_read(&input_ring, buf, sizeof buf);
		if (len > 0) {
			usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
			//buf[len] = 0;
		}

#ifdef HAVE_ADC
		// ADC
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));
		volts = adc_read_regular(ADC1);
		if( volts >= 0x7A0 && volts <= 0x890 ) { // around 3v3
#ifdef HAVE_NEOPIX
			neopix_red(0x06);
#endif // HAVE_NEOPIX
		} else if( volts >= 0xc00 && volts <= 0xd00 ) { // around 5v
#ifdef HAVE_NEOPIX
			neopix_red(0x33);
#endif // HAVE_NEOPIX
		} else if( volts <= 0x100 ) { // around 0v
#ifdef HAVE_NEOPIX
			neopix_red(0);
#endif // HAVE_NEOPIX
		} else {
#ifdef HAVE_NEOPIX
			neopix_red(0xFF);
#endif // HAVE_NEOPIX
		}
#endif // HAVE_ADC

		// buttons
		service_extbut();
		if(extbut_state == 1 && (extbut_time - extbut_lastchange) >= 1000) { // long press
			if( extbut_acted == 0) {
#ifdef HAVE_MODES
				if(usart_mode == MODE_SERPLUS) { // switching to RAW
					usart_mode = MODE_RAW;
					usart_dtr_rts_raw_setup();
				} else { // switching to SERPLUS
					usart_mode = MODE_SERPLUS;
					usart_dtr_rts_serplus_setup();
					usart_disable(USART1);
					usart_set_serplus_params();
					usart_enable(USART1);
				}
#endif // HAVE_MODES
				extbut_acted = 1;
			}
		} else {
			extbut_acted = 0;
		}
#ifdef HAVE_MODES
#ifdef HAVE_NEOPIX
		neopix_blue(usart_mode == MODE_SERPLUS ? 0 : 0x20);
#endif // HAVE_NEOPIX
#endif // HAVE_MODES
	}
}
// }}}


// vim: shiftwidth=4:tabstop=4:noexpandtab:foldmethod=marker:
