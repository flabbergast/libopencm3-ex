/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

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
	 }
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
	.extralen = sizeof(cdcacm_functional_descriptors)
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

static const char *usb_strings[] = {
	"flabbergast",
	"CDC-ACM Test",
	"DeMo",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* Aux functions */
static void led_flip(void) {
  if(GPIOA_IDR & (1<<15)) {
	  GPIOA_BRR = GPIO15;
  } else {
	  GPIOA_BSRR = GPIO15;
  }
}

static int cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *)local_buf;

		/* We echo signals back to host as notification. */
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		// usbd_ep_write_packet(0x83, buf, 10);
		return 1;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if(*len < sizeof(struct usb_cdc_line_coding))
			return 0;

		return 1;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if (len) {
		usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
		buf[len] = 0;
	led_flip();
	}
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

int main(void)
{
	usbd_device *usbd_dev;

#if defined(BOOTLOADER8K)
#	define SCB_VTOR MMIO32(0xE000ED08)
	SCB_VTOR = (uint32_t) 0x08002000;
#endif

#if defined(STM32F0)
	rcc_clock_setup_in_hsi_out_48mhz();
	crs_autotrim_usb_enable();
	rcc_set_usbclk_source(RCC_HSI48);

	// this is needed for F042 in TSSOP-20 package; USB is on shared pins
	//rcc_periph_clock_enable(RCC_SYSCFG_COMP);
	//SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
 
#elif defined(STM32L0)

	// libopencm3 clock setup can be a bit ad-hoc, so let's configure manually
	// 32MHz, PLL supplied by the internal HSI16 oscillator
	/* Set the flash latency */
	FLASH_ACR |= FLASH_ACR_LATENCY_1WS;
	/* Turn on HSI16 */
	RCC_CR |= RCC_CR_HSI16ON;
	while ((RCC_CR & RCC_CR_HSI16RDY) == 0);
	/* Make sure PLL is off (it should be after reset, but ...) */
	RCC_CR &= ~RCC_CR_PLLON;
	while (RCC_CR & RCC_CR_PLLRDY);
	/* Set the PLL source to HSI16 */
	RCC_CFGR &= ~(1<<16); // RCC_CFGR_PLLSRC
	/* Set up the PLL */
	uint32_t reg = RCC_CFGR
				   & ~( (RCC_CFGR_PLLMUL_MASK << RCC_CFGR_PLLMUL_SHIFT)
					  | (RCC_CFGR_PLLDIV_MASK << RCC_CFGR_PLLDIV_SHIFT));
	RCC_CFGR = reg | (RCC_CFGR_PLLMUL_MUL4 << RCC_CFGR_PLLMUL_SHIFT)
				   | (RCC_CFGR_PLLDIV_DIV2 << RCC_CFGR_PLLDIV_SHIFT);
	/* Turn on PLL and switch to it */
	RCC_CR |= RCC_CR_PLLON;
	while ((RCC_CR & RCC_CR_PLLRDY) == 0);
	RCC_CFGR |=  RCC_CFGR_SW_PLL;
	/* Set the peripheral clock frequencies used. */
	rcc_ahb_frequency  = 32000000; 
	rcc_apb1_frequency = 32000000;
	rcc_apb2_frequency = 32000000;
	// end of manual clock configuration

	// Enable VREFINT reference for HSI48
	rcc_periph_clock_enable(RCC_SYSCFG);
	SYSCFG_CFGR3 |= SYSCFG_CFGR3_ENREF_HSI48;

	rcc_set_hsi48_source_rc48();
	crs_autotrim_usb_enable();

	rcc_osc_on(RCC_HSI48);
	rcc_wait_for_osc_ready(RCC_HSI48);

	/* MCO on PA9, to verify MCU speed with a scope */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO9);
	RCC_CFGR |= (RCC_CFGR_MCO_SYSCLK << RCC_CFGR_MCO_SHIFT) | (RCC_CFGR_MCOPRE_DIV16 << 28);

#else
	#error "Edit the sources and do the clock setup!"
#endif

	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15); // LED
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);   // button, has external pull-down

	usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	char greeting[16] = "hello, world\r\n";

	while (1) {
		usbd_poll(usbd_dev);
		if (gpio_get(GPIOA, GPIO1)) {
			usbd_ep_write_packet(usbd_dev, 0x82, greeting, 15);
			for (uint32_t i = 0; i < 2000000; i++) {	/* Wait a bit, cheap "debounce" */
				__asm__("nop");
			}
		}
	}
}

// vim: tabstop=4:shiftwidth=4:noexpandtab
