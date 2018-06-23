/* -----------------------------------------------------------------------------
 * This file is a part of the NVCM project: https://github.com/nvitya/nvcm
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
 *  file:     usbhiddevice.cpp
 *  brief:    USB HID Device Example
 *  version:  1.00
 *  date:     2018-05-19
 *  authors:  nvitya
*/

#include "usbhiddevice.h"

#define USB_HID_CONFIG_DESC_SIZ       34
#define USB_HID_DESC_SIZ              9
#define HID_MOUSE_REPORT_DESC_SIZE    74

#define HID_DESCRIPTOR_TYPE           0x21
#define HID_REPORT_DESC               0x22

#define HID_HS_BINTERVAL               0x07
#define HID_FS_BINTERVAL               0x0A
#define HID_POLLING_INTERVAL           0x0A

#define HID_REQ_SET_PROTOCOL          0x0B
#define HID_REQ_GET_PROTOCOL          0x03

#define HID_REQ_SET_IDLE              0x0A
#define HID_REQ_GET_IDLE              0x02

#define HID_REQ_SET_REPORT            0x09
#define HID_REQ_GET_REPORT            0x01

#define HID_EPIN_ADDR                 0x81
#define HID_EPIN_SIZE                 0x04

// the device configuration: more descriptors joined together, starting with the configuration descriptor

const uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] =
{
	/************** Configuration Descriptor ****************/
  /* 00 */
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,     /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,              /*bmAttributes: Interrupt endpoint*/
  HID_EPIN_SIZE,     /*wMaxPacketSize: 4 Byte max */
  0x00,
  HID_FS_BINTERVAL,          /*bInterval: Polling Interval (10 ms)*/
  /* 34 */
} ;

/* USB HID device Configuration Descriptor */
const uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ] =
{
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};

/* USB Standard Device Descriptor */
const uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

//nvitya: this report descriptor is a special one, it describes the report format and its usage
//        these descriptors are usually generated with a special program

const uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] =
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


bool TUsbHidDevice::InitDevice()
{
	devdesc.vendor_id = 0x0483; // ST
	devdesc.product_id = 0x5710;

#if 0
	stringtable[USBD_STRIDX_MANUFACTURER] = (char *)"STMicroelectronics";
	stringtable[USBD_STRIDX_PRODUCT] = (char *)"HID Joystick in FS Mode";
	stringtable[USBD_STRIDX_SERIAL] = (char *)"498F20793932";

	//  4     9     8     F     2     0     7     9     3     9     3     2
	// 34 00 39 00 38 00 46 00 32 00 30 00 37 00 39 00 33 00 39 00 33 00 32 00
  SetDesc(USB_DESC_TYPE_CONFIGURATION, (void *)&USBD_HID_CfgDesc[0], USB_HID_CONFIG_DESC_SIZ);
  SetDesc(USB_DESC_TYPE_DEVICE_QUALIFIER, (void *)&USBD_HID_DeviceQualifierDesc[0], USB_LEN_DEV_QUALIFIER_DESC);

  SetDesc(HID_DESCRIPTOR_TYPE, (void *)&USBD_HID_Desc[0], USB_HID_DESC_SIZ);
  SetDesc(HID_REPORT_DESC, (void *)&HID_MOUSE_ReportDesc[0], HID_MOUSE_REPORT_DESC_SIZE);

  hiddata.data1 = 0;
  hiddata.dx = 0;
  hiddata.dy = 0;
  hiddata.data2 = 0;

  usbctrl.AddEndpoint(&ep_hidreport, 1, 4, 0, USBEF_TYPE_INTERRUPT);
#endif

	return true;
}

void TUsbHidDevice::SendReport(int8_t adx, int8_t ady)
{
	hiddata.dx = adx;
	hiddata.dy = ady;

	ep_hidreport.Send(&hiddata, sizeof(hiddata), 0);
}
