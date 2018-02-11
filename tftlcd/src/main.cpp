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
 *  file:     main.cpp (serialflash)
 *  brief:    Multi-board / connection type TFT LCD screen test
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwclkctrl.h"
#include "hwpins.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "textscreen.h"

#include "traces.h"

TTextScreen scr;

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

TGpioPin  led1pin(1, 0, false);
TGpioPin  led2pin(1, 7, false);
TGpioPin  led3pin(1, 14, false);

#define LED_COUNT 3

/* 3.5" RPI Display connected to the CN7 ports on the Nucleo-144
 *
 */

#include "tftlcd_spi.h"

TTftLcd_spi  lcd;

void setup_board()
{
	// LCD control
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_AF_5);  // SPI1_SCK = LCD pin 23
	hwpinctrl.PinSetup(PORTNUM_A, 6, PINCFG_AF_5);  // SPI1_MISO = n.c.
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_AF_5);  // SPI1_MOSI = LCD pin 19

	lcd.pin_reset.Assign(PORTNUM_F, 12, false); // = LCD pin 22
	lcd.pin_reset.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // B0: RESET

	lcd.pin_cs.Assign(PORTNUM_D, 14, false); // LCD pin 24
	lcd.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	lcd.pin_cd.Assign(PORTNUM_D, 15, false); // LCD pin 18
	lcd.pin_cd.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SPI1
	lcd.spi.speed = 48000000;
	lcd.spi.Init(1);

	lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_UNKNOWN, 320, 480);
	lcd.SetRotation(0);
}

#endif

#if defined(BOARD_DEV_STM32F407ZE)

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

#define LED_COUNT 2

#include "tftlcd_mm16_f407ze.h"

TTftLcd_mm16_F407ZE  lcd;

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_UNKNOWN, 240, 320);
	lcd.SetRotation(2);
}

#endif

#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(2, 13, false); // PC13

#include "tftlcd_spi.h"

TTftLcd_spi  lcd;
TGpioPin     lcd_reset_pin(PORTNUM_B, 0, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// LCD control
	//hwpinctrl.PinSetup(PORTNUM_A, 4, PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // SPI1_CS as GPIO
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_OUTPUT | PINCFG_AF_0); // SPI1_SCK
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_OUTPUT | PINCFG_AF_0); // SPI1_MOSI

	lcd.pin_reset.Assign(PORTNUM_B, 0, false);
	lcd.pin_reset.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // B0: RESET

	lcd.pin_cs.Assign(PORTNUM_A, 4, false);
	lcd.pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	lcd.pin_cd.Assign(PORTNUM_B, 1, false);
	lcd.pin_cd.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SPI1
	lcd.spi.speed = 48000000;
	lcd.spi.Init(1);

	//lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_ST7735, 128, 160);
	lcd.SetRotation(2);
}

#endif

#if defined(BOARD_ARDUINO_DUE)

/* 16 bit parallel display module designed to arduino mega connected to DUE
 * The pins arent layed out optimal so it is as fast as an SPI display
 *
 * At least optimization -O1 or -Og must be used otherwise it will be too slow
 */

TGpioPin  led1pin(1, 27, false); // D13

#include "tftlcd_gp16_due.h"

TTftLcd_gp16_due  lcd;

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	lcd.mirrorx = true;
	lcd.Init(LCD_CTRL_HX8357B, 320, 480);
	lcd.SetRotation(0);
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
	scr.Update();
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

	//TRACE("hbcounter = %i\r\n", hbcounter);

	scr.WriteChar(32 + (hbcounter & 127));
}

// the C libraries require "_start" so we keep it as the entry point
extern "C" __attribute__((noreturn)) void _start(void)
{
	// the processor jumps here right after the reset
	// the MCU runs slower, using the internal RC oscillator
	// all variables are unstable, they will be overridden later

	mcu_disable_interrupts();

	// Set the interrupt vector table offset, so that the interrupts and exceptions work
	mcu_init_vector_table();

  mcu_preinit_code(); // inline code for preparing the MCU, RAM regions. Without this even the stack does not work on some MCUs.

  unsigned clockspeed = MAX_CLOCK_SPEED;

#ifdef MCU_INPUT_FREQ
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, MAX_CLOCK_SPEED))  // activate the external crystal oscillator with multiplication x2
#else
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, MAX_CLOCK_SPEED))  // activate the external crystal oscillator with multiplication x2
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

	//mcu_enable_fpu();    // enable coprocessor if present
	//mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();

	//mcu_enable_interrupts();

	lcd.FillScreen(0x0000);
	scr.Init(&lcd);

	TRACE("\r\n------------------------------------------\r\n");
	TRACE("TFT LCD Test\r\n");

	TRACE("Starting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

#if CLOCKCNT_BITS >= 32

	unsigned hbclocks = SystemCoreClock / 100;

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
		}
	}

#else

	// use the SysTick for millisec counting

	unsigned hbticks = 1000 / 20;

	unsigned t0 = systick;

	// Infinite loop
	while (1)
	{
		idle_task();

		if (systick - t0 > hbticks)
		{
			heartbeat_task();
			t0 = systick;

			if (hbcounter > 20)  hbticks = 500 / 2;  // slow down to 0.5 s
		}
	}

#endif
}

// ----------------------------------------------------------------------------
