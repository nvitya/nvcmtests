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

class TUifHidTest : public TUsbInterface
{
private:
	typedef TUsbInterface super;

public:
	THidData        hiddata;

	uint8_t         protocol = 0;
	uint8_t         idlestate = 1;

	TUsbEndpoint    ep_hidreport;
	void            SendReport(int8_t adx, int8_t ady);

public: // mandatory functions
	virtual bool    InitInterface();
	virtual void    OnConfigured();
	virtual bool    HandleTransferEvent(TUsbEndpoint * aep, bool htod);
	virtual bool    HandleSetupRequest(TUsbSetupRequest * psrq);

};

extern TUifHidTest hidtest;

void usb_hid_test_init();
void usb_hid_test_run();
void usb_hid_test_heartbeat();


#endif /* SRC_USB_HID_TEST_H_ */
