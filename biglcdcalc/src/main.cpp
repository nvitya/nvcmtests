// ATARILCD Main

#include "platform.h"
#include "hwclkctrl.h"
#include "hwpins.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "hwpwm.h"
#include "keymatrix.h"

#include "board_config.h"
#include "systimer.h"
#include "sysdisplay.h"
#include "syskeyboard.h"

#include "exprcalc.h"

TExprCalc  g_calc;
TCalcVar   calcvar;

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void _start(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

	mcu_disable_interrupts();

  mcu_preinit_code(); // inline code for preparing the MCU, RAM regions. Without this even the stack does not work on some MCUs.

	// Set the interrupt vector table offset, so that the interrupts and exceptions work
	mcu_init_vector_table();

  unsigned clockspeed = MCU_CLOCK_SPEED;

#ifdef MCU_INPUT_FREQ
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, clockspeed))  // activate the external crystal oscillator with multiplication x2
#else
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, clockspeed))  // activate the external crystal oscillator with multiplication x2
#endif
	{
		while (1)
		{
			// the external oscillator did not start.
		}
	}

	// now the MCU runs faster, start the memory, and C/C++ initialization:
	cppinit();
	// ...from now on all the variables work, static classes are initialized.

	// provide info to the system about the clock speed:
	hwclkctrl.SetClockInfo(clockspeed);

	mcu_enable_fpu();    // enable coprocessor if present
	mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	board_init();

	mcu_enable_interrupts();

	systimer_init();

	g_keyboard.Init();

	pin_led1.Set1();

	g_display.Init();

	g_display.printf("Hello World !\n");

	for (unsigned n = 32; n < 127; ++n)
	{
		g_display.WriteChar(n);
	}

	g_display.Run();

	pin_led1.Set0();

	unsigned hbclocks = SystemCoreClock / 2;  // start blinking fast

	unsigned t0, t1;

	t0 = CLOCKCNT;

	unsigned cyclecnt = 0;

	unsigned prev_scanserial = g_keyscan_events.serial;
	unsigned prev_symserial = g_keysym_events.serial;

	char editrow[128];
	uint8_t editpos = 0;
	editrow[0] = 0;

	g_display.SetPos(0, g_display.rows-1);
	g_display.WriteChar('>');

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		g_keyboard.Run();

#if 0
		g_display.SetPos(0, 2);
		g_display.printf("cyclecnt = %u\n", cyclecnt);
		g_display.printf("systick = %u\n", g_sysms);
		g_display.printf("heartbeat = %u\n", hbcounter);
#endif

#if 0
		if (prev_scanserial != g_keyscan_events.serial)
		{
			TKeyScanEvent * pevent = &g_keyscan_events.events[g_keyscan_events.serial & 31];

      #if 1
				g_display.SetPos(0, 5);
				g_display.printf("scan %i: ev=%i, key=%2i",
						 g_keyscan_events.serial,
						 pevent->evtype,
						 pevent->scancode
				);
      #endif

			prev_scanserial = g_keyscan_events.serial;
		}
#endif

#if 1
		if (prev_symserial != g_keysym_events.serial)
		{
			TKeySymbolEvent * psyme = &g_keysym_events.events[g_keysym_events.serial & 15];

     	#if 0
				g_display.SetPos(0, 6);
				g_display.printf("sym %i: ss=%02X, sym=%04X",
						 g_keysym_events.serial,
						 psyme->shiftstate,
						 psyme->symbol
				);
	    #endif

			// some processing
			uint16_t keysym = psyme->symbol;
			if (KEYSYM_BACKSPACE == keysym)
			{
				if (editpos > 0)
				{
					--editpos;
					editrow[editpos] = 0;
				}
			}
			else if (KEYSYM_UP == keysym)
			{
				if (g_display.cursor_y > 0)  --g_display.cursor_y;
			}
			else if (KEYSYM_LEFT == keysym)
			{
				if (g_display.cursor_x > 0)  --g_display.cursor_x;
			}
			else if (KEYSYM_DOWN == keysym)
			{
				if (g_display.cursor_y < 7)  ++g_display.cursor_y;
			}
			else if (KEYSYM_RIGHT == keysym)
			{
				if (g_display.cursor_x < 39)  ++g_display.cursor_x;
			}
			else if ((editpos < g_display.cols-2) && (keysym >= 32) && (keysym <= 127))
			{
				editrow[editpos] = keysym;
				++editpos;
			}

			// print editrow
			g_display.SetPos(0, g_display.rows-1);
			g_display.WriteChar('>');
			for (unsigned n = 0; n < g_display.cols-2; n++)
			{
				if (n < editpos)
				{
					g_display.WriteChar(editrow[n]);
				}
				else
				{
					g_display.WriteChar(32);
				}
			}

			g_display.cursor_on = true;
			//g_display.cursor_x = editpos;
			//g_display.cursor_y = 7;

			prev_symserial = g_keysym_events.serial;

			if (KEYSYM_ENTER == keysym)
			{
				// execute
				g_display.WriteChar(10); // new line

				int err = g_calc.Evaluate(&editrow[0], editpos, &calcvar);
				if (err)
				{
					g_display.printf("= [error %i]\n>", err);
				}
				else
				{
					g_display.printf("= %0.8f\n>", calcvar.floatvalue);
				}

				// reset
				editpos = 0;
				editrow[0] = 0;
			}
		}
#endif

		g_display.Run(); // must be called regularly

		if (t1-t0 > hbclocks)
		{
			//g_display.printf("cyclecnt = %u\n", cyclecnt);
			t0 = t1;
		}

		++cyclecnt;
	}
}

// ----------------------------------------------------------------------------
