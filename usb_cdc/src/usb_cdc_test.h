// usb_cdc_test.h

#ifndef SRC_USB_CDC_TEST_H_
#define SRC_USB_CDC_TEST_H_

#include "usbdevice.h"
#include "hwuart.h"

class TUifCdcControl : public TUsbInterface
{
private:
	typedef TUsbInterface super;

public:

	TUsbEndpoint    ep_control;
	void            SendReport(int8_t adx, int8_t ady);

public: // mandatory functions
	virtual bool    InitInterface();
	virtual void    OnConfigured();
	virtual bool    HandleTransferEvent(TUsbEndpoint * aep, bool htod);
	virtual bool    HandleSetupRequest(TUsbSetupRequest * psrq);
};

class TUifCdcData : public TUsbInterface
{
private:
	typedef TUsbInterface super;

public:
	THwUart         uart;

	TUsbEndpoint    ep_htod;
	TUsbEndpoint    ep_dtoh;

public: // mandatory functions
	virtual bool    InitInterface();
	virtual void    OnConfigured();
	virtual bool    HandleTransferEvent(TUsbEndpoint * aep, bool htod);
	virtual bool    HandleSetupRequest(TUsbSetupRequest * psrq);
};

extern TUifCdcControl   cdccontrol;
extern TUifCdcData      cdcdata;

void usb_cdc_test_init();
void usb_cdc_test_run();
void usb_cdc_test_heartbeat();


#endif /* SRC_USB_CDC_TEST_H_ */
