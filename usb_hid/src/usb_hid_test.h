// usb_hid_test.h

#ifndef SRC_USB_HID_TEST_H_
#define SRC_USB_HID_TEST_H_

#include "usbdevice.h"

typedef struct THidData
{
	uint8_t  data1;
	int8_t   dx;
	int8_t   dy;
	uint8_t  data2;
//
} THidData;

class TUsbHidTest : public TUsbInterface
{
public:
	THidData        hiddata;

	TUsbEndpoint    ep_hidreport;

	virtual bool    InitInterface();
	void            SendReport(int8_t adx, int8_t ady);

};

extern TUsbHidTest hidtest;

void usb_hid_test_init();
void usb_hid_test_run();


#endif /* SRC_USB_HID_TEST_H_ */
