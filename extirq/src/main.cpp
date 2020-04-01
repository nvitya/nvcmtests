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
 *  file:     main.cpp (extirq)
 *  brief:    Multi-board External IRQ example for NVCM
 *  version:  1.00
 *  date:     2020-04-01
 *  authors:  nvitya
 *  description:
 *    The pins "pin_irqctrl" and "pin_extirq" must be connected externally !
 *    Toggling the "pin_irqctrl" thus causes an IRQ at rising edge, which normally happens every 2 s.
 *    The IRQ accept latency is measured in CPU clocks.
*/

#include "platform.h"
#include "hwpins.h"
#include "hwextirq.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"

#include "traces.h"

THwUart    conuart;  // console uart

THwExtIrq  extirq;

volatile unsigned g_extirq_start_time;
volatile unsigned g_extirq_cnt = 0;
volatile unsigned g_extirq_accept_time = 0;

unsigned g_extirq_cnt_prev = 0;

// It is recommended to define all your irq priorities at the same place:
#define IRQPRIO_EXTINT   5

#if defined(BOARD_MIN_F103)

TGpioPin    led1pin(PORTNUM_C, 13, false);
TGpioPin    pin_irqctrl(PORTNUM_A, 1, false);
TGpioPin    pin_extirq(PORTNUM_A, 0, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_0);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_0);  // USART1_RX
	conuart.Init(1);

	pin_irqctrl.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	pin_extirq.Setup(PINCFG_INPUT);

	// On STM32 the first 16 external IRQs are tied to GPIO sources, where you can select the GPIO port
	extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING);

	// Search the EXTI interrupt number from the MCU header definition (this can be found always at the beginning)
  #define EXTINT_IRQ_NUM                   6
  #define EXTINT_IRQ_HANDLER  IRQ_Handler_06  // overrides the weak definition in vectors.cpp
}
#endif

#if defined(BOARD_MIBO48_STM32F303)

TGpioPin    led1pin(PORTNUM_C, 13, false);
TGpioPin    pin_irqctrl(PORTNUM_A, 1, false);
TGpioPin    pin_extirq(PORTNUM_A, 0, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7 | PINCFG_PULLUP);  // USART1_RX
	conuart.Init(1);

	pin_irqctrl.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	pin_extirq.Setup(PINCFG_INPUT);

	// On STM32 the first 16 external IRQs are tied to GPIO sources, where you can select the GPIO port
	extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING);

	// Search the EXTI interrupt number from the MCU header definition (this can be found always at the beginning)
  #define EXTINT_IRQ_NUM                   6
  #define EXTINT_IRQ_HANDLER  IRQ_Handler_06  // overrides the weak definition in vectors.cpp
}
#endif

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

TGpioPin  led1pin(1, 0, false);
TGpioPin  led2pin(1, 7, false);
TGpioPin  led3pin(1, 14, false);

#define LED_COUNT 3

TGpioPin    pin_extirq(PORTNUM_A, 0, false);
TGpioPin    pin_irqctrl(PORTNUM_A, 1, false);

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

  // USART3: Stlink USB / Serial converter
	// USART3_TX: PD.8
	hwpinctrl.PinSetup(3, 8,  PINCFG_OUTPUT | PINCFG_AF_7);
	// USART3_RX: Pd.9
	hwpinctrl.PinSetup(3, 9,  PINCFG_INPUT  | PINCFG_AF_7);
	conuart.Init(3); // USART3

	pin_irqctrl.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	pin_extirq.Setup(PINCFG_INPUT);

	// On STM32 the first 16 external IRQs are tied to GPIO sources, where you can select the GPIO port
	extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING);

	// Search the EXTI interrupt number from the MCU header definition (this can be found always at the beginning)
  #define EXTINT_IRQ_NUM                   6
  #define EXTINT_IRQ_HANDLER  IRQ_Handler_06  // overrides the weak definition in vectors.cpp
}

#endif

#if defined(BOARD_MIBO64_ATSAME5X)

TGpioPin  led1pin(PORTNUM_A, 1, false);
TGpioPin  pin_extirq(PORTNUM_A, 0, false);
TGpioPin  pin_irqctrl(PORTNUM_A, 2, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// SERCOM0
	hwpinctrl.PinSetup(PORTNUM_A, 4, PINCFG_OUTPUT | PINCFG_AF_3);  // PAD[0] = TX
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_INPUT  | PINCFG_AF_3);  // PAD[1] = RX
	conuart.Init(0);

	pin_irqctrl.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	pin_extirq.Setup(PINCFG_INPUT | PINCFG_AF_A);  // the EXTINT[0]

	extirq.Init(0, HWEXTIRQ_RISING);  // setup EXTINT[0]

	// Search the EIC_x_IRQn interrupt number from the MCU header definition (this can be found always at the beginning)
  #define EXTINT_IRQ_NUM                  12
  #define EXTINT_IRQ_HANDLER  IRQ_Handler_12  // overrides the weak definition in vectors.cpp
}
#endif

#if defined(BOARD_ARDUINO_DUE)

TGpioPin  led1pin(PORTNUM_B, 27, false); // D13
TGpioPin  pin_extirq(  PORTNUM_C, 21, false);  // D9
TGpioPin  pin_irqctrl( PORTNUM_C, 22, false);  // D8

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART - On the Arduino programmer interface
	hwpinctrl.PinSetup(PORTNUM_A, 8, PINCFG_INPUT | PINCFG_AF_0);  // UART_RXD
	hwpinctrl.PinSetup(PORTNUM_A, 9, PINCFG_OUTPUT | PINCFG_AF_0); // UART_TXD
	conuart.Init(0);  // UART

	pin_irqctrl.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	pin_extirq.Setup(PINCFG_INPUT);

	extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING);
	//extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING | HWEXTIRQ_FALLING); // test both edge

	// The shared PIOC IRQ will be used, warning this MCU requires IrqBegin()
  #define EXTINT_IRQ_NUM                  13
  #define EXTINT_IRQ_HANDLER  IRQ_Handler_13  // overrides the weak definition in vectors.cpp
}

#endif

#if defined(BOARD_MIBO100_ATSAME70)

TGpioPin  led1pin(PORTNUM_D, 13, false);
TGpioPin  pin_extirq(PORTNUM_A, 0, false);
TGpioPin  pin_irqctrl(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.Init(0);

	pin_irqctrl.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	pin_extirq.Setup(PINCFG_INPUT);

	extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING);
	//extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING | HWEXTIRQ_FALLING); // test both edge

	// The shared PIOA IRQ will be used, warning this MCU requires IrqBegin()
  #define EXTINT_IRQ_NUM                  10
  #define EXTINT_IRQ_HANDLER  IRQ_Handler_10  // overrides the weak definition in vectors.cpp
}

#endif

#if defined(BOARD_MIBO64_ATSAM4S)

TGpioPin  led1pin(PORTNUM_A, 1, false);
TGpioPin  pin_extirq(PORTNUM_A, 0, false);
TGpioPin  pin_irqctrl(PORTNUM_A, 2, false);  // A1 is used by the LED

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.Init(0);

	pin_irqctrl.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);
	pin_extirq.Setup(PINCFG_INPUT);

	extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING);
	//extirq.Init(pin_extirq.portnum, pin_extirq.pinnum, HWEXTIRQ_RISING | HWEXTIRQ_FALLING); // test both edge

	// The shared PIOA IRQ will be used, warning this MCU requires IrqBegin()
  #define EXTINT_IRQ_NUM                  11
  #define EXTINT_IRQ_HANDLER  IRQ_Handler_11  // overrides the weak definition in vectors.cpp
}

#endif


#ifndef LED_COUNT
  #define LED_COUNT 1
#endif

#ifndef EXTINT_IRQ_HANDLER
  #error "IRQ Handler is not defined!"
#else

extern "C"
//__attribute__ ((interrupt ("IRQ")))
void EXTINT_IRQ_HANDLER()
{
	g_extirq_accept_time = CLOCKCNT;
	++g_extirq_cnt;

	// ATSAM MCUs require IrqBegin(), exactly once per IRQ handler
	// on ATSAM, the IrqBegin() acknowledges the IRQ as well, the IrqAck is empty
	extirq.IrqBegin();

	extirq.IrqAck();
	// The IRQ acknowledge might be delayed, and thus the IRQ will be triggered again, and the IRQ counter
	// is incremented twice. (e.g. this was observed on the STM32F746)
	// Issuing a read to the same peripheral ensures that the IRQ ACK finishes
	// when we leave the IRQ handler:
	if (extirq.IrqPending()) { }
}

#endif

void extirq_init()
{
	IRQn_Type irqnum = IRQn_Type(EXTINT_IRQ_NUM);
	NVIC_SetPriority(irqnum, IRQPRIO_EXTINT);
	NVIC_ClearPendingIRQ(irqnum);
	NVIC_EnableIRQ(irqnum);
}

unsigned hbcounter = 0;

void heartbeat_task()
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

	if (hbcounter & 1)
	{
		g_extirq_start_time = CLOCKCNT;
		pin_irqctrl.Set1();  // the Set1(), Set0() is faster than the Toggle() on STM32
	}
	else
	{
		g_extirq_start_time = CLOCKCNT;
		pin_irqctrl.Set0();  // the Set1(), Set0() is faster than the Toggle() on STM32
	}

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
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, clockspeed))
#else
	if (!hwclkctrl.InitCpuClockIntRC(MCU_INTRC_SPEED, clockspeed))
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

	TRACE("\r\n--------------------------\r\n");
	TRACE("NVCM EXTIRQ TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	TRACE("\r\nStarting main cycle...\r\n");

	extirq_init();

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

	unsigned t0, t1;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		if (g_extirq_cnt != g_extirq_cnt_prev)
		{
			unsigned latency = g_extirq_accept_time - g_extirq_start_time;
			TRACE("EXTIRQ-%i, latency: %u clocks\r\n", g_extirq_cnt, latency); // warning: this takes long, be careful with TRACE-es in IRQ handlers !

			g_extirq_cnt_prev = g_extirq_cnt;
		}

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
