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
 *  file:     usbhiddevice.h
 *  brief:    USB HID Device Example
 *  version:  1.00
 *  date:     2018-05-19
 *  authors:  nvitya
*/

#ifndef SRC_USBHIDDEVICE_H_
#define SRC_USBHIDDEVICE_H_

#include "usbdevice.h"


typedef struct THidData
{
	uint8_t  data1;
	int8_t   dx;
	int8_t   dy;
	uint8_t  data2;
//
} THidData;

class TUsbHidDevice : public TUsbDevice
{
public:
	THidData        hiddata;

	THwUsbEndpoint  ep_hidreport;

  virtual bool    InitDevice();

	void            SendReport(int8_t adx, int8_t ady);

};

#endif /* SRC_USBHIDDEVICE_H_ */
