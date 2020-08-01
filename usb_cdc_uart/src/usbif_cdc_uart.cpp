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
 *  file:     usbif_cdc_uart.cpp
 *  brief:    USB to Serial USB Interface definition
 *  version:  1.00
 *  date:     2020-08-01
 *  authors:  nvitya
*/

#include <usbif_cdc_uart.h>
#include "platform.h"
#include "string.h"
#include "traces.h"

const uint8_t cdc_desc_header_func[] =
{
	/*Header Functional Descriptor*/
	0x05,   /* bLength: Endpoint Descriptor size */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x00,   /* bDescriptorSubtype: Header Func Desc */
	0x10,   /* bcdCDC: spec release number */
	0x01,
};

const uint8_t cdc_desc_call_management[] =
{
	/*Call Management Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x01,   /* bDescriptorSubtype: Call Management Func Desc */
	0x00,   /* bmCapabilities: 0 = no call management */
	0x00,   /* bDataInterface: 0 */   // ???
};

const uint8_t cdc_desc_call_acm_func[] =
{
	/*ACM Functional Descriptor*/
	0x04,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
	0x02,   /* bmCapabilities */
};

const uint8_t cdc_desc_call_union_func[] =
{
	/*Union Functional Descriptor*/
	0x05,   /* bFunctionLength */
	0x24,   /* bDescriptorType: CS_INTERFACE */
	0x06,   /* bDescriptorSubtype: Union func desc */
	0x00,   /* bMasterInterface: Communication class interface */
	0x01,   /* bSlaveInterface0: Data Class Interface */
};

//----------------------------------------------------

bool TUifCdcUartControl::InitCdcUart(TUifCdcUartData * adataif, THwUart * auart, THwDmaChannel * adma_tx, THwDmaChannel * adma_rx)
{
	dataif = adataif;
	adataif->control = this;

	uart = auart;
	dma_tx = adma_tx;
	dma_rx = adma_rx;

	if (!uart || !dma_tx || !dma_rx)
	{
		return false;
	}

	uart->DmaAssign(true,  dma_tx);
	uart->DmaAssign(false, dma_rx);

	linecoding.baudrate = uart->baudrate;
	linecoding.databits = uart->databits;
	linecoding.paritytype = (uart->parity ? (uart->oddparity ? 1 : 2) : 0);
	linecoding.charformat = 0;

	StopUart();

	return true;
}

bool TUifCdcUartControl::InitInterface()
{
	intfdesc.interface_class = 2; // CDC
	intfdesc.interface_subclass = 2; // Abstract Control Model
	intfdesc.interface_protocol = 0; // 0 = no class specitic control

	interface_name = "VCP Control";

	// some other descriptors are required

	AddConfigDesc((void *)&cdc_desc_header_func[0],     true);
	AddConfigDesc((void *)&cdc_desc_call_management[0], true);
	AddConfigDesc((void *)&cdc_desc_call_acm_func[0],   true);
	AddConfigDesc((void *)&cdc_desc_call_union_func[0], true);

	// endpoints

	ep_manage.Init(HWUSB_EP_TYPE_INTERRUPT, false, 64);
	ep_manage.interval = 10; // polling interval
	AddEndpoint(&ep_manage);

	return true;
}

void TUifCdcUartControl::OnConfigured()
{
	//TRACE("SPEC Device Configured.\r\n");

	StartUart();

	//ep_manage.EnableRecv();
}

bool TUifCdcUartControl::HandleTransferEvent(TUsbEndpoint * aep, bool htod)
{
	return false;
}

bool TUifCdcUartControl::HandleSetupRequest(TUsbSetupRequest * psrq)
{
	if (0x20 == psrq->request) // set line coding, data stage follows !
	{
		TRACE("VCP Set line coding (SETUP)\r\n");
		device->StartSetupData();  // start receiving the data part, which will be handled at the HandleSetupData()
		return true;
	}
	else if (0x21 == psrq->request) // Get line coding
	{
		TRACE("VCP Get line coding\r\n");
		device->StartSendControlData(&linecoding, sizeof(linecoding));
		return true;
	}
	else if (0x22 == psrq->request) // Set Control Line State
	{
		TRACE("VCP Set Control Line State: %04X\r\n", psrq->value);
		device->SendControlStatus(true);
		return true;
	}

	return false;
}

// the setup request's data part comes in a separate phase and so it has a separate callback:
bool TUifCdcUartControl::HandleSetupData(TUsbSetupRequest * psrq, void * adata, unsigned adatalen)
{
	if (0x20 == psrq->request) // set line coding
	{
		memcpy(&linecoding, adata, sizeof(linecoding));

		TRACE("VCP Line Coding data:\r\n  baud=%i, format=%i, parity=%i, bits=%i\r\n",
				linecoding.baudrate, linecoding.charformat, linecoding.paritytype, linecoding.databits
		);

		StopUart();

		uart->baudrate = linecoding.baudrate;
		uart->databits = linecoding.databits;
		if (linecoding.paritytype)
		{
			if (1 == linecoding.paritytype)
			{
				uart->oddparity = true;
			}
			else
			{
				uart->oddparity = false;
			}
			uart->parity = true;
		}
		else
		{
			uart->parity = false;
		}

		if (linecoding.charformat)
		{
			// stop bits other than 1 is not supported
		}

		StartUart();

		device->SendControlStatus(true);
		return true;
	}

	return false;
}

void TUifCdcUartControl::StopUart()
{
	uart_running = false;

	if (dma_tx)  dma_tx->Disable();
	if (dma_rx)  dma_rx->Disable();
}

void TUifCdcUartControl::StartUart()
{
	// re-init the uart with the new parameters
	uart->Init(uart->devnum);

	if (!dma_tx || !dma_rx)  // DMA required
	{
		return;
	}

	if (dma_rx)
	{
		serial_rxidx = 0;
		dmaxfer_rx.bytewidth = 1;
		dmaxfer_rx.count = sizeof(serial_rxbuf);
		dmaxfer_rx.dstaddr = &serial_rxbuf[0];
		dmaxfer_rx.flags = DMATR_CIRCULAR;

		uart->DmaStartRecv(&dmaxfer_rx);
	}

	dataif->Reset();

	uart_running = true;
}

bool TUifCdcUartControl::SerialSendBytes(uint8_t * adata, unsigned adatalen)
{
	bool result = false;

	if (adatalen <= sizeof(serial_txbuf[0]) - serial_txlen)
	{
		memcpy(&serial_txbuf[serial_txbufidx][serial_txlen], adata, adatalen);
		serial_txlen += adatalen;
		result = true;
	}

	if (serial_txlen && !dma_tx->Active())
	{
		// setup the TX DMA and flip the buffer

		dmaxfer_tx.flags = 0;
		dmaxfer_tx.bytewidth = 1;
		dmaxfer_tx.count = serial_txlen;
		dmaxfer_tx.srcaddr = &serial_txbuf[serial_txbufidx][0];

		uart->DmaStartSend(&dmaxfer_tx);

		// change the buffer
		serial_txbufidx ^= 1;
		serial_txlen = 0;
	}

	return result;
}

void TUifCdcUartControl::Run()
{
	if (!uart_running)
	{
		return;
	}

	// Put the UART RX data to the inactive USB tx buffer

	uint8_t c;

	unsigned dma_write_idx = sizeof(serial_rxbuf) - dma_rx->Remaining();
	if (dma_write_idx >= sizeof(serial_rxbuf)) // should not happen
	{
		dma_write_idx = 0;
	}

	while (dma_write_idx != serial_rxidx)
	{
		c = serial_rxbuf[serial_rxidx];
		if (!dataif->AddTxByte(c))
		{
			break;
		}

		++serial_rxidx;
		if (serial_rxidx >= sizeof(serial_rxbuf))  serial_rxidx = 0;
	}

	dataif->SendTxBytes(); // send if there are any and it is possible

	// The USB -> Serial transfers are controlled by the USB Events,
	// but when the serial buffer is full then it is stalled

	if (dataif->usb_rxlen)
	{
		dataif->TrySendUsbDataToSerial(); // re-enables the USB receive when the USB Rx data transferred to the serial buffer
	}
}

//------------------------------------------------

bool TUifCdcUartData::InitInterface()
{
	intfdesc.interface_class = 0x0A; // CDC Data
	intfdesc.interface_subclass = 0;
	intfdesc.interface_protocol = 0; // no specific protocol

	interface_name = "VCP Data";

	// endpoints

	ep_input.Init(HWUSB_EP_TYPE_BULK, true, 64);
	AddEndpoint(&ep_input);
	ep_output.Init(HWUSB_EP_TYPE_BULK, false, 64);
	AddEndpoint(&ep_output);

	Reset();

	return true;
}

void TUifCdcUartData::Reset()
{
	usb_txbufidx = 0;
	usb_txlen = 0;
	ready_to_send = true;
}

void TUifCdcUartData::OnConfigured()
{
	TRACE("VCP Data Configured.\r\n");

	Reset();

	ep_input.EnableRecv();
}

bool TUifCdcUartData::HandleTransferEvent(TUsbEndpoint * aep, bool htod)
{
	if (htod)
	{
		usb_rxlen = ep_input.ReadRecvData(&usb_rxbuf[0], sizeof(usb_rxbuf));
		//TRACE("%i byte VCP data arrived\r\n", usb_rxlen);

		TrySendUsbDataToSerial();  // re-enables USB receive when it is successful
	}
	else
	{
		// the data chunk is successfully sent
		ready_to_send = true;
	}

	return true;
}

bool TUifCdcUartData::AddTxByte(uint8_t abyte)
{
	if (usb_txlen < sizeof(usb_txbuf[0]))
	{
		usb_txbuf[usb_txbufidx][usb_txlen] = abyte;
		++usb_txlen;
		return true;
	}
	else
	{
		return false;
	}
}

bool TUifCdcUartData::SendTxBytes()
{
	if (usb_txlen > 0)
	{
		// there is something to send, try to send it
		if (ready_to_send)
		{
			ep_output.StartSendData(&usb_txbuf[usb_txbufidx], usb_txlen);
			ready_to_send = false;

			// change the buffer
			usb_txbufidx ^= 1;
			usb_txlen = 0;
			return true;
		}
	}

	return false;
}

bool TUifCdcUartData::TrySendUsbDataToSerial()
{
	if (!usb_rxlen)
	{
		return true;
	}

	if (control->SerialSendBytes(&usb_rxbuf[0], usb_rxlen))
	{
		usb_rxlen = 0;
		ep_input.EnableRecv();
		return true;
	}
	else
	{
		return false;
	}
}
