// usb_hid_test.cpp

#include "usb_hid_test.h"
#include "traces.h"
#include "usbdevice.h"
#include "hwusbctrl.h"

TUsbDevice    usbdev;
TUifHidTest   hidtest;

#define HID_DESCRIPTOR_TYPE           0x21
#define HID_REPORT_DESC               0x22

//nvitya: this report descriptor is a special one, it describes the report format and its usage
//        these descriptors are usually generated with a special program

const uint8_t HID_MOUSE_ReportDesc[] =
{
  0x05,   0x01,
  0x09,   0x02,
  0xA1,   0x01,
  0x09,   0x01,

  0xA1,   0x00,
  0x05,   0x09,
  0x19,   0x01,
  0x29,   0x03,

  0x15,   0x00,
  0x25,   0x01,
  0x95,   0x03,
  0x75,   0x01,

  0x81,   0x02,
  0x95,   0x01,
  0x75,   0x05,
  0x81,   0x01,

  0x05,   0x01,
  0x09,   0x30,
  0x09,   0x31,
  0x09,   0x38,

  0x15,   0x81,
  0x25,   0x7F,
  0x75,   0x08,
  0x95,   0x03,

  0x81,   0x06,
  0xC0,   0x09,
  0x3c,   0x05,
  0xff,   0x09,

  0x01,   0x15,
  0x00,   0x25,
  0x01,   0x75,
  0x01,   0x95,

  0x02,   0xb1,
  0x22,   0x75,
  0x06,   0x95,
  0x01,   0xb1,

  0x01,   0xc0
};

/* USB HID device Configuration Descriptor */
const uint8_t USBD_HID_Desc[] =
{
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  sizeof(HID_MOUSE_ReportDesc),/*wItemLength: Total length of Report descriptor*/
  0x00,
};


bool TUifHidTest::InitInterface()
{
	hiddata.dx = 0;
	hiddata.dy = 0;

	intfdesc.interface_class = 3; // HID
	intfdesc.interface_subclass = 1; // boot
	intfdesc.interface_protocol = 2; // mouse

	interface_name = "NVCM USB HID Test";

	AddConfigDesc((void *)&USBD_HID_Desc[0], false);
	AddDesc(HID_REPORT_DESC, (void *)&HID_MOUSE_ReportDesc[0], sizeof(HID_MOUSE_ReportDesc), 0);

	ep_hidreport.Init(HWUSB_EP_TYPE_INTERRUPT, false, 4);
	AddEndpoint(&ep_hidreport);

	return true;
}

void TUifHidTest::SendReport(int8_t adx, int8_t ady)
{
	hiddata.dx = adx;
	hiddata.dy = ady;

	ep_hidreport.StartSendData(&hiddata, sizeof(hiddata));
}

void TUifHidTest::OnConfigured()
{
	// after the configuration it must send a report immediately!
	SendReport(0, 0);
}

bool TUifHidTest::HandleSetupRequest(TUsbSetupRequest * psrq)
{
	if (psrq->rqtype == 0x21)
	{
		if (psrq->request == 0x0A)
		{
			// set idle frequency.
			device->SendControlStatus(true);
			return true;
		}
	}
	else if (psrq->rqtype == 0xA1) // vendor request
	{
		device->SendControlStatus(true);
		return true;
	}

	return super::HandleSetupRequest(psrq);
}

bool TUifHidTest::HandleTransferEvent(TUsbEndpoint * aep, bool htod)
{
	if (htod)
	{
		// impossible case
		return false;
	}
	else
	{
		// called when the hid report sent successfully
		return true;
	}
}

void usb_hid_test_init()
{
	TRACE("Initializing USB HID Test\r\n");

	usbdev.devdesc.vendor_id = 0xDEAD;
	usbdev.devdesc.product_id = 0x0A1D;
	usbdev.manufacturer_name = "github.com/nvcm";
	usbdev.device_name = "NVCM Example HID Joystick";
	usbdev.device_serial_number = "NVCM-HID-1";

	usbdev.AddInterface(&hidtest);

	if (!usbdev.Init()) // this must be the last one, when the interfaces added
	{
		TRACE("Error initializing USB device!\r\n");
		return;
	}
}

void usb_hid_test_run()
{
	usbdev.HandleIrq();
}

void usb_hid_test_heartbeat()
{
	if (hidtest.configured)
	{
		hidtest.SendReport(2, 3);
	}
}
