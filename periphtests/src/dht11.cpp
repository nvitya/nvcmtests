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
 *  file:     dht11.cpp
 *  brief:    DHT11 Temperature and Humidity Sensor
 *  version:  1.00
 *  date:     2018-09-29
 *  authors:  nvitya
*/

#include "dht11.h"
#include "clockcnt.h"
#include "traces.h"

bool TDht11::Init()
{
	us_clocks = SystemCoreClock / 1000000;

	initialized = false;
	if (!pin_dio.Assigned())
	{
		return false;
	}

	pin_dio.Setup(PINCFG_OUTPUT | PINCFG_OPENDRAIN | PINCFG_GPIO_INIT_1 | PINCFG_SPEED_SLOW);
	pin_dio.Set0();
	delay_us(1000);
	pin_dio.Set1();
	delay_us(10);
	pin_dio.SwitchDirection(0);
	//pin_dio.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);


	initialized = true;
	return true;
}

bool TDht11::Measure()
{
	if (!initialized)
	{
		return false;
	}

	// start measurement

	pin_dio.SwitchDirection(1); // set to output
	pin_dio.Set0();
	delay_us(20);
	pin_dio.Set1();
	delay_us(30);
	pin_dio.SwitchDirection(0); // set to input

	stime = CLOCKCNT;

	while (1)
	{
		if (pin_dio.Value() == 0)
		{
			TRACE("Sensor answered!\r\n");
			break;
		}

		if (CLOCKCNT - stime > us_clocks * 200)
		{
			TRACE("Sensor answer timeout.\r\n");
			break;
		}
	}

	// to be continued...
	delay_us(10000);

	return true;
}
