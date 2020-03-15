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
 *  file:     usb_spec_test.cpp
 *  brief:    Special USB Device definition
 *  version:  1.00
 *  date:     2020-03-11
 *  authors:  nvitya
 *  notes:
 *   The "definition" of this special USB device is originated from Niklas Gürtler:
 *     https://github.com/Erlkoenig90/f1usb
 *     https://www.mikrocontroller.net/articles/USB-Tutorial_mit_STM32
 *
 *   This implementation is somewhat different, because using enpoint 2 for buld data in.
 *
 *   The device does not provide a standard interface, it can be controller with libusb.
 *   The project for testing it: https://github.com/nvitya/nvcmtests/usbclient_spec
 *   Or using the one from Niklas Gürtler: https://github.com/Erlkoenig90/usbclient
*/

#include <usb_spec_test.h>
#include "traces.h"
#include "hwpins.h"
#include "usbdevice.h"
#include "hwusbctrl.h"

extern TGpioPin led1pin;

TUsbDevSpec   usbdev;
TUifSpecTest  uif_spectest;

void usb_spec_test_init()
{
	TRACE("Initializing Special USB Test\r\n");

	if (!usbdev.Init()) // calls InitDevice first which sets up the device
	{
		TRACE("Error initializing USB device!\r\n");
		return;
	}

	TRACE("IF input endpoint: %02X\r\n", uif_spectest.ep_input.index);
	TRACE("IF output endpoint: %02X\r\n", uif_spectest.ep_output.index);
}

void usb_spec_test_run()
{
	usbdev.HandleIrq();
}

bool TUifSpecTest::InitInterface()
{
	intfdesc.interface_class = 0xFF;
	intfdesc.interface_subclass = 0xFF;
	intfdesc.interface_protocol = 0xFF;

	interface_name = "Test BULK Interface";

	//AddConfigDesc((void *)&USBD_HID_Desc[0], false);
	//AddDesc(HID_REPORT_DESC, (void *)&HID_MOUSE_ReportDesc[0], sizeof(HID_MOUSE_ReportDesc), 0);

	ep_input.Init(HWUSB_EP_TYPE_BULK, true, 64);
	AddEndpoint(&ep_input);
	ep_output.Init(HWUSB_EP_TYPE_BULK, false, 64);
	AddEndpoint(&ep_output);

	return true;
}

void TUifSpecTest::OnConfigured()
{
	TRACE("SPEC Device Configured.\r\n");

	ep_input.EnableRecv();
}

bool TUifSpecTest::HandleTransferEvent(TUsbEndpoint * aep, bool htod)
{
	int r;
	if (htod)
	{
		r = ep_input.ReadRecvData(&databuf[0], sizeof(databuf));
		TRACE("%i byte BULK data arrived\r\n", r);
		if (r > 0)
		{
			for (int i = 0; i < r; ++i)
			{
				databuf[i] = (__RBIT(databuf[i]) >> 24);
			}
		}

		ep_output.StartSendData(&databuf[0], r);
	}
	else
	{
		ep_input.EnableRecv();  // enable new data receive only after when the response was successfully sent
	}

	return true;
}

bool TUsbDevSpec::InitDevice()
{
	devdesc.vendor_id = 0xDEAD;
	devdesc.product_id = 0xBEEF;
	manufacturer_name = "ACME Inc.";
	device_name = "Fluxkompensator";
	device_serial_number = "42-1337-47/11";

	AddInterface(&uif_spectest);

	return true;
}

bool TUsbDevSpec::HandleSpecialSetupRequest()
{
	if ((0xC0 == setuprq.rqtype) && (2 == setuprq.request))
	{
		// query LED state
		ctrlbuf[0] = led1pin.OutValue();
		cdlen = 1;
		StartSendControlData();
		return true;
	}
	else if ((0x40 == setuprq.rqtype) && (1 == setuprq.request))
	{
		// Set LED state
		led1pin.SetTo(setuprq.value & 1);
		SendControlStatus(true);
		return true;
	}

	return false;
}

