// usb_cdc_test.cpp

#include <usb_cdc_test.h>
#include "traces.h"
#include "usbdevice.h"
#include "hwusbctrl.h"

TUsbDevice       usbdev;

TUifCdcControl   cdccontrol;
TUifCdcData      cdcdata;

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
	0x00,   /* bmCapabilities: D0+D1 */
	0x01,   /* bDataInterface: 1 */
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

bool TUifCdcControl::InitInterface()
{
	intfdesc.interface_class = 2; // CDC
	intfdesc.interface_subclass = 2; // Abstract Control Model
	intfdesc.interface_protocol = 1; // Common AT commands

	interface_name = "VCP Control";

	AddConfigDesc((void *)&cdc_desc_header_func[0],     true);
	AddConfigDesc((void *)&cdc_desc_call_management[0], true);
	AddConfigDesc((void *)&cdc_desc_call_acm_func[0],   true);
	AddConfigDesc((void *)&cdc_desc_call_union_func[0], true);

	ep_control.Init(HWUSB_EP_TYPE_INTERRUPT, 8, 0);
	AddEndpoint(&ep_control);

	return true;
}

void TUifCdcControl::OnConfigured()
{
	TRACE("CDC Control Configured.\r\n");
	ep_control.EnableRecv();
}

bool TUifCdcControl::HandleTransferEvent(TUsbEndpoint * aep, bool htod)
{
	TRACE("CDC Control Transfer Event!\r\n");

	if (htod)
	{
		aep->FinishRecv(true);
	}
	else
	{
		aep->FinishSend();
	}

	return true;
}

bool TUifCdcControl::HandleSetupRequest(TUsbSetupRequest * psrq)
{
	uint8_t rqclass = ((psrq->rqtype >> 5) & 3);
	if (1 == rqclass) // class requests
	{
#if 0
		if (0x0B == psrq->request) // set protocol
		{
			protocol = (psrq->value & 0xFF);
			device->SendControlAck();
			return true;
		}
		else if (0x03 == psrq->request) // get protocol
		{
			device->ep_ctrl.StartSend(&protocol, 1);
			return true;
		}
		else if (0x0A == psrq->request) // set idle (sending frequency)
		{
			idlestate = (psrq->value & 0xFF);
			device->SendControlAck();
			return true;
		}
		else if (0x02 == psrq->request) // get idle
		{
			device->ep_ctrl.StartSend(&idlestate, 1);
			return true;
		}
#endif
	}

	// if not handled, call parent

	return super::HandleSetupRequest(psrq);
}

bool TUifCdcData::InitInterface()
{
	intfdesc.interface_class = 0x0A; // CDC
	intfdesc.interface_subclass = 0;
	intfdesc.interface_protocol = 0;

	interface_name = "VCP Data";

	// no additional descriptors

	ep_htod.Init(HWUSB_EP_TYPE_BULK, 64,  0);
	ep_dtoh.Init(HWUSB_EP_TYPE_BULK,  0, 64);
	AddEndpoint(&ep_htod);
	AddEndpoint(&ep_dtoh);

	return true;
}

void TUifCdcData::OnConfigured()
{
	TRACE("CDC Data Configured.\r\n");
	// after the configuration it must send a report immediately!
	ep_htod.EnableRecv();
}

bool TUifCdcData::HandleTransferEvent(TUsbEndpoint * aep, bool htod)
{
	TRACE("CDC Data Transfer Event!\r\n");

	if (htod)
	{
		aep->FinishRecv(true);
	}
	else
	{
		aep->FinishSend();
	}

	return true;
}

bool TUifCdcData::HandleSetupRequest(TUsbSetupRequest * psrq)
{
	uint8_t rqclass = ((psrq->rqtype >> 5) & 3);
	if (1 == rqclass) // class requests
	{
#if 0
		if (0x0B == psrq->request) // set protocol
		{
			protocol = (psrq->value & 0xFF);
			device->SendControlAck();
			return true;
		}
		else if (0x03 == psrq->request) // get protocol
		{
			device->ep_ctrl.StartSend(&protocol, 1);
			return true;
		}
		else if (0x0A == psrq->request) // set idle (sending frequency)
		{
			idlestate = (psrq->value & 0xFF);
			device->SendControlAck();
			return true;
		}
		else if (0x02 == psrq->request) // get idle
		{
			device->ep_ctrl.StartSend(&idlestate, 1);
			return true;
		}
#endif
	}

	// if not handled, call parent

	return super::HandleSetupRequest(psrq);
}


void usb_cdc_test_init()
{
	TRACE("Initializing USB CDC Test\r\n");

	usbdev.devdesc.vendor_id = 0x0483;
	usbdev.devdesc.product_id = 0x5740;
	usbdev.manufacturer_name = "STMicroelectronics";
	usbdev.device_name = "STM32 Virtual ComPort";
	usbdev.device_serial_number = "498F20793933";

	usbdev.AddInterface(&cdccontrol);
	usbdev.AddInterface(&cdcdata);

	if (!usbdev.Init()) // this must be the last one, when the interfaces added
	{
		TRACE("Error initializing USB device!\r\n");
		return;
	}
}

void usb_cdc_test_run()
{
	usbdev.HandleIrq();
}

void usb_cdc_test_heartbeat()
{
}
