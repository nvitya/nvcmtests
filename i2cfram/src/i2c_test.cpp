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
 *  file:     i2c_test.cpp
 *  brief:    FM24V02A I2C FRAM test
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
 *  notes:
 *    SCL = pin 6 of FRAM
 *    SDA = pin 5 of FRAM
 *    the other pins have internal pull-downs
*/

#include "platform.h"
#include "hwpins.h"
#include "traces.h"

void i2c_test()
{
	TRACE("I2C test for FM20V02A FRAM\r\n");

	// TWIHS0
	hwpinctrl.PinSetup(PORTNUM_A,  4, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SCL/TWCK0
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SDA/TWD0
	//conuart.Init(0x000); // UART0

	// Enable TWIHS0
	PMC->PMC_PCER0 = (1 << 19);  // 20, 41

	Twihs * regs = TWIHS0;

	// reset:
	regs->TWIHS_CR = 0
		| (1 << 29)  // FIFO disable
		| (1 << 26)  // lock clear
		| (1 << 24)  // THR clear
		| (1 << 17)  // ACM disable
		| (1 << 11)  // SMB disable
		| (1 <<  9)  // HighSpeed disable
		| (1 <<  5)  // Slave mode disable
		| (1 <<  3)  // Master mode disable
	;

	// setup 400 kHz clock
	unsigned speed = 400000;
	unsigned periphclock = SystemCoreClock;
	if (SystemCoreClock > 150000000)
	{
		periphclock = (SystemCoreClock >> 1);
	}

	unsigned halfclockdiv = ((periphclock / speed) >> 1);
	unsigned ckdiv = 0;
	while (halfclockdiv > 255)
	{
		ckdiv += 1;
		halfclockdiv = (halfclockdiv >> 1);
	}

	regs->TWIHS_CWGR = 0
		| (0 << 24)  // HOLD(5)
		| (0 << 20)  // CKSRC: 0 = periph. clock
		| (ckdiv << 16)  // CKDIV(3): big prescaler
		| (halfclockdiv << 8)  // CHDIV
		| (halfclockdiv << 0)  // CLDIV
  ;

	regs->TWIHS_CR = (1 << 2);  // Master mode enable

	// set device address
	unsigned dadr = 0x50;

	regs->TWIHS_IADR = 0x003C187E;

	regs->TWIHS_MMR = 0
		| (dadr << 16)
		| (1 << 12)  // MREAD: 1 = read
		| (0 <<  8)  // 0 = No internal address bytes
	;

	// send something:

	regs->TWIHS_CR = (1 << 0);   // send a start condition

	unsigned char data;

	unsigned cnt = 0;

	while (cnt < 4)
	{
		while ((regs->TWIHS_SR & (1 << 1)) == 0)
		{
			// wait until something received
		}

		data = regs->TWIHS_RHR;
		TRACE("Received data: %02X\r\n", data);

		++cnt;
	}

	regs->TWIHS_CR = (1 << 1);   // send a stop condition

	while ((regs->TWIHS_SR & (1 << 1)) == 0)
	{
		// wait until something received
	}

	data = regs->TWIHS_RHR;
	TRACE("Last Received data: %02X\r\n", data);

	while ((regs->TWIHS_SR & (1 << 1)) == 0)
	{
		// wait until something received
	}

	data = regs->TWIHS_RHR;
	TRACE("Last Received data: %02X\r\n", data);

	TRACE("I2C Test finished.\r\n");
}



