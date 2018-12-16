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
 *  file:     main.cpp (ledstripe)
 *  brief:    Multi-board APA 102 (76 LEDS) example for NVCM
 *  version:  1.00
 *  date:     2018-12-16
 *  authors:  nvitya
 *  notes:
 *    Connect the led stripe power other than CPU 3.3 V, because it can draw over 1 Amps
 *    (I measured 1.2 A with the highest settings (255, 255, 255, 255)
 *    Fortunately the APA 102 led stripe I have allows 5V power and 3.3V control (SPI CLOCK + SPI MOSI)
*/

#include "platform.h"
#include "hwclkctrl.h"
#include "hwpins.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "hwspi.h"

#include "ledstripe.h"

TLedStripe  ledstripe;

THwSpi spi;

#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(PORTNUM_C, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_AF_0);  // SPI1_SCK
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_AF_0);  // SPI1_MOSI

	// SPI and its DMA must be initialized before
	ledstripe.spi.speed = 4000000;
	ledstripe.spi.Init(1);
	ledstripe.dmach.Init(1, 3, 1);  // DMA1/CH3 = SPI1_TX
	ledstripe.Init(76);
}
#endif

#if defined(BOARD_MIBO20_STM32F030) || defined(BOARD_MIBO20_STM32F070)

TGpioPin  led1pin(PORTNUM_B, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_AF_0);  // SPI1_SCK
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_AF_0);  // SPI1_MOSI

	// SPI and its DMA must be initialized before
	ledstripe.spi.speed = 4000000;
	ledstripe.spi.Init(1);
	ledstripe.dmach.Init(1, 3, 1);  // DMA1/CH3 = SPI1_TX
	ledstripe.Init(76);
}
#endif


#if defined(BOARD_ENEBO_A)

TGpioPin  led1pin(PORTNUM_D, 13, true);
TGpioPin  led2pin(PORTNUM_D, 14, true);
TGpioPin  led3pin(PORTNUM_A, 20, true);

#define LED_COUNT 3

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_D, 21, PINCFG_AF_B);  // SPI0_MOSI
	hwpinctrl.PinSetup(PORTNUM_D, 22, PINCFG_AF_B);  // SPI0_SCLK

	ledstripe.spi.speed = 4000000;
	ledstripe.spi.Init(0);
	ledstripe.dmach.Init(5, 1);  // CH5, SPI0_TX
	ledstripe.Init(76);
}

#endif

#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

volatile unsigned systick = 0;

extern "C" void SysTick_Handler(void)
{
	++systick;
}

void idle_task()
{
}

unsigned hbcounter = 0;

void heartbeat_task() // invoked every 0.5 s
{
	++hbcounter;

	led1pin.SetTo(hbcounter >> 0);
#if LED_COUNT > 1
	led2pin.SetTo(hbcounter >> 1);
#endif
#if LED_COUNT > 2
	led3pin.SetTo(hbcounter >> 2);
#endif
#if LED_COUNT > 3
	led4pin.SetTo(hbcounter >> 3);
#endif
#if LED_COUNT > 4
	led5pin.SetTo(hbcounter >> 4);
#endif

	//swo_printf("hbcounter = %i\r\n", hbcounter);

#if 0

  for (int n = 0; n < ledstripe.ledcount; ++n)
  {
  	if ((hbcounter & 3) == 0)
  	{
  		ledstripe.SetIRGB(n, 128, 0, 0, 128);
  	}
  	else if ((hbcounter & 3) == 1)
  	{
  		ledstripe.SetIRGB(n, 128, 0, 128, 0);
  	}
  	else if ((hbcounter & 3) == 2)
  	{
  		ledstripe.SetIRGB(n, 128, 128, 0, 0);
  	}
  	else if ((hbcounter & 3) == 3)
  	{
  		ledstripe.SetIRGB(n, 128, 128, 128, 128);
  	}
  }

  ledstripe.StartDisplay();

#endif

#if 1

  for (int n = 0; n < ledstripe.ledcount; ++n)
  {
  	uint8_t x = n + hbcounter;
#if 1
  	uint8_t r = ((x & 0x03) << 6);
  	uint8_t g = ((x & 0x0C) << 4);
  	uint8_t b = ((x & 0x30) << 2);
#else
  	uint8_t r = ((x & 0x03) << 2);
  	uint8_t g = ((x & 0x0C) << 0);
  	uint8_t b = ((x & 0x30) >> 2);
#endif

  	ledstripe.SetIRGB(n, 127, r, g, b);
  }

  ledstripe.StartDisplay();

#else
  for (int n = 0; n < ledstripe.ledcount; ++n)
  {
  	//ledstripe.SetIRGB(n, (hbcounter & 7) << 5, 255, 255, 255);
  	//ledstripe.SetIRGB(n, 127, 255, 255, 255);

  	uint8_t r = ((hbcounter & 3) == 1 ? 255 : 0);
  	uint8_t g = ((hbcounter & 3) == 2 ? 255 : 0);
  	uint8_t b = ((hbcounter & 3) == 3 ? 255 : 0);
  	ledstripe.SetIRGB(n, 127, r, g, b);
  }

  ledstripe.StartDisplay();

#endif

}

#define LEDCOUNT 75

uint8_t leddata[4 * (1 + LEDCOUNT + 4)];

void test_leds()
{
#if 0
	uint32_t n;

  leddata[0] = 0;
  leddata[1] = 0;
  leddata[2] = 0;
  leddata[3] = 0;

  for (n = 0; n < LEDCOUNT; ++n)
  {
  	leddata[4 + n * 4 + 0] = 0xE7;
  	leddata[4 + n * 4 + 1] = 0x00;  // blue
  	leddata[4 + n * 4 + 2] = 0x00;  // green
  	leddata[4 + n * 4 + 3] = 0xFF;  // red
  }

  leddata[4 * (LEDCOUNT + 2) + 0] = 0xFF;
  leddata[4 * (LEDCOUNT + 2) + 1] = 0xFF;
  leddata[4 * (LEDCOUNT + 2) + 2] = 0xFF;
  leddata[4 * (LEDCOUNT + 2) + 3] = 0xFF;

  uint16_t rd;
  // sending led data
  for (n = 0; n < 4 * (LEDCOUNT + 2); ++n)
  {
  	spi.SendData(leddata[n]);
  	//while (spi.TryRecvData(&rd)) { }
  }
#endif

  for (int n = 0; n < ledstripe.ledcount; ++n)
  {
  	uint8_t r = ((n & 0x03) << 6);
  	uint8_t g = ((n & 0x0C) << 4);
  	uint8_t b = ((n & 0x30) << 2);

  	ledstripe.SetIRGB(n, 128, r, g, b);
  }

  ledstripe.StartDisplay();
}

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
	setup_board();

	test_leds();

	mcu_enable_interrupts();

	SysTick_Config(SystemCoreClock / 1000);

	unsigned hbclocks = SystemCoreClock / 20;  // start blinking fast

	unsigned t0, t1;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		idle_task();

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			t0 = t1;

			//if (hbcounter > 20)  hbclocks = SystemCoreClock * 4;
		}
	}
}

// ----------------------------------------------------------------------------
