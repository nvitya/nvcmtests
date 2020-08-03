/* -----------------------------------------------------------------------------
 * This file is a part of the NVCM Tests project: https://github.com/nvitya/nvcmtests
 * Copyright (c) 2018 Viktor Nagy, nvitya
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 * --------------------------------------------------------------------------- */
/*
 *  file:     usbdev_main.cpp
 *  brief:    USB Device Definition (USB to Serial USB Device)
 *  version:  1.00
 *  date:     2020-07-31
 *  authors:  nvitya
*/

#include <usbdev_main.h>
#include "string.h"
#include "traces.h"
#include "hwpins.h"
#include "usbdevice.h"
#include "hwusbctrl.h"
#include "hwuart.h"
#include "hwdma.h"


TUsbDevMain          usbdev;

TUifCdcUartControl   uif_cdcuart;
TUifCdcUartData      uif_cdcuart_data;

THwUart              usbuart;
THwDmaChannel        usbuart_dma_tx;
THwDmaChannel        usbuart_dma_rx;

#if defined(BOARD_MIN_F103)

static void init_usb_uart()
{
	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT | PINCFG_PULLUP);  // USART2_RX - do not set AF for inputs!
	usbuart.Init(2);

	usbuart_dma_tx.Init(1, 7, 2); // USART2_TX
	usbuart_dma_rx.Init(1, 6, 2); // USART2_RX
}

#endif

#if defined(BOARD_MIBO48_STM32G473)

static void init_usb_uart()
{
	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_7 | PINCFG_PULLUP);  // USART2_RX
	usbuart.Init(2);

	usbuart_dma_tx.Init(1, 7, 27); // USART2_TX
	usbuart_dma_rx.Init(1, 6, 26); // USART2_RX
}

#endif

#if defined(BOARD_MIBO48_STM32F303)

static void init_usb_uart()
{
	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_7 | PINCFG_PULLUP);  // USART2_RX
	usbuart.Init(2);

	usbuart_dma_tx.Init(1, 7, 2); // USART2_TX
	usbuart_dma_rx.Init(1, 6, 2); // USART2_RX
}

#endif

#if defined(BOARD_MIBO64_STM32F070)

static void init_usb_uart()
{
	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_1);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_1 | PINCFG_PULLUP);  // USART2_RX
	usbuart.Init(2);

	usbuart_dma_tx.Init(1, 4, 2); // USART2_TX
	usbuart_dma_rx.Init(1, 5, 2); // USART2_RX
}

#endif

#if defined(BOARD_MIBO20_STM32F070)

static void init_usb_uart()
{
	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_1);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_1 | PINCFG_PULLUP);  // USART2_RX
	usbuart.Init(2);

	usbuart_dma_tx.Init(1, 4, 2); // USART2_TX
	usbuart_dma_rx.Init(1, 5, 2); // USART2_RX
}

#endif

#if defined(BOARD_MIBO64_STM32F405)

#warning "Buggy USB implementation (see line coding trace)"

static void init_usb_uart()
{
	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_7 | PINCFG_PULLUP);  // USART2_RX
	usbuart.Init(2);

	usbuart.printf("USB uart test...\r\n");

	usbuart_dma_tx.Init(1, 6, 4); // USART2_TX
	usbuart_dma_rx.Init(1, 5, 4); // USART2_RX
}

#endif

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

#warning "STM32F7 does not work perfectly with big data chunks!"

static void init_usb_uart()
{
	// USART6 - on the Arduino pins D0, D1
	hwpinctrl.PinSetup(PORTNUM_G, 14,  PINCFG_OUTPUT | PINCFG_AF_8);  // USART6_TX
	hwpinctrl.PinSetup(PORTNUM_G,  9,  PINCFG_INPUT  | PINCFG_AF_8 | PINCFG_PULLUP);  // USART6_RX
	usbuart.Init(6);

	usbuart_dma_tx.Init(2, 7, 5); // USART6_TX
	usbuart_dma_rx.Init(2, 2, 5); // USART6_RX
}

#endif

#if defined(BOARD_MIBO64_ATSAME5X)

#warning "Imperfect USB->Serial big block transfer !!!"

static void init_usb_uart()
{
	// SERCOM2
	hwpinctrl.PinSetup(PORTNUM_A, 12, PINCFG_AF_2);  // PAD[0] = TX
	hwpinctrl.PinSetup(PORTNUM_A, 13, PINCFG_AF_2);  // PAD[1] = RX
	usbuart.Init(2);

	usbuart_dma_tx.Init(4, 0x09); // 0x09 = SERCOM2_TX
	usbuart_dma_rx.Init(5, 0x08); // 0x08 = SERCOM2_RX
}

#endif

#if defined(BOARD_MIBO100_ATSAME70)

#warning "Imperfect USB implementation (config descriptor segmentation)"

static void init_usb_uart()
{
	hwpinctrl.PinSetup(PORTNUM_A,  5,  PINCFG_INPUT  | PINCFG_AF_C);  // UART1_RX
	hwpinctrl.PinSetup(PORTNUM_A,  4,  PINCFG_OUTPUT | PINCFG_AF_C);  // UART1_TX
	usbuart.Init(1);

	usbuart_dma_tx.Init(4, 22); // 22 = UART1_TX
	usbuart_dma_rx.Init(5, 23); // 23 = UART1_RX
}

#endif

void usb_device_init()
{
	TRACE("Initializing USB Device\r\n");

	init_usb_uart();

	if (!usbdev.Init()) // calls InitDevice first which sets up the device
	{
		TRACE("Error initializing USB device!\r\n");
		return;
	}

	TRACE("IF input endpoint: %02X\r\n", uif_cdcuart_data.ep_input.index);
	TRACE("IF output endpoint: %02X\r\n", uif_cdcuart_data.ep_output.index);
}

void usb_device_run()
{
	usbdev.HandleIrq();

	uif_cdcuart.Run();  // manage UART DMA transfers
}



//------------------------------------------------

bool TUsbDevMain::InitDevice()
{
	devdesc.vendor_id = 0xDEAD;
	devdesc.product_id = 0xCDC5;
	devdesc.device_class = 0x02; // Communications and CDC Control
	manufacturer_name = "github.com/nvcm";
	device_name = "NVCM CDC UART Example";
	device_serial_number = "NVCM-CDC-UART-1";

	uif_cdcuart.InitCdcUart(&uif_cdcuart_data, &usbuart, &usbuart_dma_tx, &usbuart_dma_rx);

	AddInterface(&uif_cdcuart);
	AddInterface(&uif_cdcuart_data);

	return true;
}

