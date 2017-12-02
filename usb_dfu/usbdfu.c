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

#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>

#define APP_ADDRESS	0x08002000

#define PROGRAM_BY_HALF_PAGE

#define DFU_PACKET_SIZE 1024
#define MCU_PAGE_SIZE 128
/* Note: DFU_PACKET_SIZE has to be an integer multiple of MCU_PAGE_SIZE */
/*       DFU_PACKET_SIZE needs to match the DfuSe string below */

/* This is supposed to be in libopencm3/lib/stm32/l0/flash.c but isn't */
void flash_erase_page(uint32_t address);

#if defined(PROGRAM_BY_HALF_PAGE)

#define __FROM_RAM __attribute__((__long_call__, section(".data"),optimize("Os")))

void flash_half_page(uint32_t address, uint32_t *data);
__FROM_RAM void flash_half_page(uint32_t address, uint32_t *data)
{
	uint16_t i;

	__asm__ volatile ("CPSID I\n");

	FLASH_PECR |= FLASH_PECR_PROG | FLASH_PECR_FPRG;

	i = 0;
	while(i < (MCU_PAGE_SIZE/8)) { /* half a page, 4byte words */
		/* Address doesn't need to be increased ... */
		MMIO32(address) = *data;
		data++;
		i++;
	}

	while (FLASH_SR & FLASH_SR_BSY);

	__asm__ volatile ("CPSIE I\n");

	if ((FLASH_SR & FLASH_SR_EOP) != 0) {
		FLASH_SR = FLASH_SR_EOP;
	} /* else error */

	FLASH_PECR &= ~(FLASH_PECR_PROG | FLASH_PECR_FPRG);
}

#endif /* PROGRAM_BY_HALF_PAGE */

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

uint8_t usbd_control_buffer[DFU_PACKET_SIZE];

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog;

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0xDF11,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = DFU_PACKET_SIZE,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,

	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"flabbergast",
	"L052 DFU Bootloader",
	"STM32L052C8Tx",
	/* This string is used by ST Microelectronics' DfuSe utility. */
	"@Internal Flash   /0x08000000/8*001Ka,56*001Kg"
};

/* Notes about the dfuse string above:
 * /<start_address>/<number>*<page_size><multiplier><memtype>
 *  <number>: how many pages
 *  <page_size>: self explanatory
 *  <multiplier>: 'B'(ytes), 'K'(ilobytes), 'M'(egabytes)
 *  <memtype>: the bottom three bits are significant:
 *             writeable|erasable|readable
 * subsequent blocks separated by commas
 *
 * Using the internal page size: "@Internal Flash   /0x08000000/64*128Ba,448*128Bg"
 * Using 1K blocks: "@Internal Flash   /0x08000000/8*001Ka,56*001Kg"
 */

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout)
{
	switch (usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete. */
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	default:
		return DFU_STATUS_OK;
	}
}

static void usbdfu_getstatus_complete(usbd_device *usbd_dev, struct usb_setup_data *req)
{
	int i;
	(void)req;
	(void)usbd_dev;

	switch (usbdfu_state) {
	case STATE_DFU_DNBUSY:
		flash_unlock();
		if (prog.blocknum == 0) {
			switch (prog.buf[0]) {
			/* code to protect the bootloader? */
			case CMD_ERASE:
				{
					uint32_t *dat = (uint32_t *)(prog.buf + 1);
					i = 0;
					while ( i*MCU_PAGE_SIZE < DFU_PACKET_SIZE ) {
						flash_erase_page(*dat + i*MCU_PAGE_SIZE);
						i++;
					}
				}
			case CMD_SETADDR:
				{
					uint32_t *dat = (uint32_t *)(prog.buf + 1);
					prog.addr = *dat;
				}
			}
		} else {
			uint32_t baseaddr = prog.addr + ((prog.blocknum - 2) *
					   dfu_function.wTransferSize);
#if defined(PROGRAM_BY_HALF_PAGE)
			for (i = 0; i < prog.len; i += (MCU_PAGE_SIZE/2)) {
				uint32_t *dat = (uint32_t *)(prog.buf + i);
				flash_half_page(baseaddr+i, dat);
			}
#else
			for (i = 0; i < prog.len; i += 4) {
				uint32_t *dat = (uint32_t *)(prog.buf + i);
				flash_program_word(baseaddr + i, *dat);
			}
#endif /* PROGRAM_BY_HALF_PAGE */
		}
		flash_lock();

		/* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	case STATE_DFU_MANIFEST:
		/* USB device must detach, we just reset... */
		scb_reset_system();
		return; /* Will never return. */
	default:
		return;
	}
}

static int usbdfu_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)usbd_dev;

	if ((req->bmRequestType & 0x7F) != 0x21)
		return 0; /* Only accept class request. */

	switch (req->bRequest) {
	case DFU_DNLOAD:
		if ((len == NULL) || (*len == 0)) {
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return 1;
		} else {
			/* Copy download data for use on GET_STATUS. */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return 1;
		}
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE. */
		if (usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state. */
		usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_UPLOAD:
		/* Upload not supported for now. */
		return 0;
	case DFU_GETSTATUS: {
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */
		(*buf)[0] = usbdfu_getstatus(&bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0; /* iString not used here */
		*len = 6;
		*complete = usbdfu_getstatus_complete;
		return 1;
		}
	case DFU_GETSTATE:
		/* Return state with no state transision. */
		*buf[0] = usbdfu_state;
		*len = 1;
		return 1;
	}

	return 0;
}

static void usbdfu_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);
}


/* This is for some reason missing from libopencm3/l0 flash.c
 * (I thought it was in the pull request I've done for l0 but
 *  apparently not.)
 */
/*---------------------------------------------------------------------------*/
/** @brief Erase a Page of FLASH

This performs all operations necessary to erase a page in FLASH memory.
The page should be checked to ensure that it was properly erased. A page must
first be fully erased before attempting to program it.

Note that the page sizes differ between devices. See the reference manual or
the FLASH programming manual for details.

@param[in] address Memory address of the first word on the page to be erased.
*/
__FROM_RAM void flash_erase_page(uint32_t address)
{
	while (FLASH_SR & FLASH_SR_BSY);

	FLASH_PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;

	MMIO32(address) = (uint32_t)0;

	while (FLASH_SR & FLASH_SR_BSY);

	if ((FLASH_SR & FLASH_SR_EOP) != 0) {
		FLASH_SR = FLASH_SR_EOP;
	} /* else error */

	FLASH_PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_PROG);
}

/* End of the bit that's supposed to be in libopencm3/lib/stm32/l0/flash.c */


int main(void)
{
	usbd_device *usbd_dev;
	uint32_t k;

	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO3);

	if (gpio_get(GPIOB, GPIO3)) {
		/* Boot the application if it's valid. */
		if ((*(volatile uint32_t *)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
			/* Set vector table base address. */
			SCB_VTOR = APP_ADDRESS & 0xFFFF;
			/* Initialise master stack pointer. */
			asm volatile("msr msp, %0"::"g"
					 (*(volatile uint32_t *)APP_ADDRESS));
			/* Jump to application. */
			(*(void (**)())(APP_ADDRESS + 4))();
		}
	}

    // libopencm3 clock setup can be a bit ad-hoc, so let's configure "manually"
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
    /* end of "manual" clock setup */

	/* Enable VREFINT reference for HSI48 */
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

    /* for the LED */
	rcc_periph_clock_enable(RCC_GPIOA);

	usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, &config, usb_strings, 4, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usbdfu_set_config);

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);

	k = 0;

	while (1) {
		usbd_poll(usbd_dev);
		if(++k == 10000) {
			k = 0;
			gpio_toggle(GPIOA, GPIO15);
		}
	}
}

// vim: tabstop=4:shiftwidth=4:noexpandtab
