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
#include "hwi2c.h"
#include "traces.h"

THwI2c  i2c;

void show_mem(void * addr, unsigned len)
{
	unsigned char * cp = (unsigned char *)addr;
	TRACE("Dumping memory at %08X, len = %u\r\n", addr, len);
	for (unsigned n = 0; n < len; ++n)
	{
		TRACE(" %02X", *cp++);
		if (n % 16 == 15) TRACE("\r\n");
	}
	TRACE("\r\n");
}

void i2c_test()
{
	TRACE("I2C test for FM20V02A FRAM\r\n");

#if 0
#elif defined(BOARD_MIBO100_ATSAME70)
	// TWIHS0
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SDA/TWD0
	hwpinctrl.PinSetup(PORTNUM_A,  4, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SCL/TWCK0

	i2c.Init(0); // TWIHS0

	i2c.txdma.Init(14, 14);  // 14 = TWIHS0.TX (see XDMAC controller peripheral connections)
	i2c.rxdma.Init(15, 15);  // 15 = TWIHS0.RX

#elif defined(BOARD_MIBO64_ATSAM4S)
	// TWI0
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SDA/TWD0
	hwpinctrl.PinSetup(PORTNUM_A,  4, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SCL/TWCK0

	i2c.Init(0); // TWIHS0

	// PdmaInit must be called after i2c.Init !!!
	i2c.PdmaInit(true,  nullptr);   // use internal
	i2c.PdmaInit(false,  nullptr);  // use internal

#else
  #error "unknown board."
#endif

	uint8_t rxbuf[32];
	uint8_t txbuf[32];

	unsigned addr = 0;
	unsigned len = 16;

	TRACE("Reading memory at %04X...\r\n", addr);

	i2c.StartReadData(0x50, addr | I2CEX_2, &rxbuf[0], len);
	i2c.WaitFinish();

	show_mem(&rxbuf[0], len);

	TRACE("Writing memory to 0x0008...\r\n", addr);

	txbuf[0] = 0x61;
	txbuf[1] = 0x62;
	txbuf[2] = 0x63;
	txbuf[3] = 0x64;
	txbuf[4] = 0x68;
	txbuf[5] = 0x68;
	txbuf[6] = 0x68;

	i2c.StartWriteData(0x50, 8 | I2CEX_2, &txbuf[0], 4);
	i2c.WaitFinish();

	TRACE("Write finished.\r\n");

	TRACE("Reading memory at %04X...\r\n", addr);

	i2c.StartReadData(0x50, addr | I2CEX_2, &rxbuf[0], len);
	i2c.WaitFinish();

	show_mem(&rxbuf[0], len);


	TRACE("I2C test finished.\r\n");
}
