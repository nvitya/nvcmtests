// sdram.cpp

#include <stdio.h>
#include <string.h>
//#include <stdlib.h>

#include "platform.h"
#include "hwpins.h"
#include "clockcnt.h"

#include "sdram.h"
#include "hwsdram.h"

#include "traces.h"

void sdram_init()
{
	TRACE("Initializing %u MB SDRAM...\r\n", SDRAM_MB_SIZE);

	unsigned pin_flags = PINCFG_AF_12 | PINCFG_SPEED_MEDIUM2; // it does not work with FAST !!!

	// C: 3
	hwpinctrl.PinSetup(PORTNUM_C,  3, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_D,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D,  9, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 10, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_D, 15, pin_flags);


	hwpinctrl.PinSetup(PORTNUM_E,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  7, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E,  9, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 10, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 11, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 12, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 13, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_E, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_F,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  2, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  3, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  4, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F,  5, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 11, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 12, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 13, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 14, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_F, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_G,  0, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  1, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  4, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  5, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G,  8, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_G, 15, pin_flags);

	hwpinctrl.PinSetup(PORTNUM_H,  3, pin_flags);
	hwpinctrl.PinSetup(PORTNUM_H,  5, pin_flags);

	// config the SDRAM device: 8 MByte

	hwsdram.row_bits = 12;
	hwsdram.column_bits = 8;
	hwsdram.bank_count = 4;
	hwsdram.cas_latency = 2;

	hwsdram.row_precharge_delay = 1;
	hwsdram.row_to_column_delay = 1;
	hwsdram.recovery_delay = 1;
	hwsdram.row_cycle_delay = 5;
	hwsdram.exit_self_refresh_delay = 5;
	hwsdram.active_to_precharge_delay = 3; // TRAS

	hwsdram.burst_length = 1;  // it does not like when it bigger than 1

	hwsdram.Init();

  delay_us(1000);

  volatile uint16_t * startaddr = (uint16_t *)hwsdram.address;
  volatile uint16_t * dp = startaddr;

  int i;
  int len = 16;
  for (i = 0; i < len; ++i)
  {
  	*dp++ = 0x55aa + i;
  }

  TRACE("SDRAM read back:\r\n");
  dp = startaddr;
  for (i = 0; i < len; ++i)
  {
  	TRACE(" %04X", *dp++);
  }
	TRACE("\r\n");

	TRACE("SDRAM Init done.\r\n");
}

