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
 *  file:     main.cpp (i2c_slave)
 *  brief:    I2C Slave Test (EEPROM emulation)
 *  version:  1.00
 *  date:     2019-03-24
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "i2c_eeprom_app.h"

#include "traces.h"

THwUart        conuart;  // console uart

TI2cEepromApp  i2capp;

#define IRQPRIO_I2C    10

#define I2C_ADDRESS  0x50  // EEPROM Emulation

void setup_irq(int airqnum)
{
	IRQn_Type irqnum = IRQn_Type(airqnum);
	NVIC_SetPriority(irqnum, IRQPRIO_I2C);
	NVIC_ClearPendingIRQ(irqnum);
	NVIC_EnableIRQ(irqnum);
}

#if defined(BOARD_MIBO100_ATSAME70)

TGpioPin  led1pin(PORTNUM_D, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX (marked as B10 beside D31)
	conuart.Init(0);

	// TWIHS0
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SDA/TWD0
	hwpinctrl.PinSetup(PORTNUM_A,  4, PINCFG_AF_0 | PINCFG_PULLUP); // TWIHS0: SCL/TWCK0
	i2capp.devnum = 0;

  #define I2C_IRQ_NUM                   19
  #define I2C_IRQ_HANDLER   IRQ_Handler_19
}

#endif

#if defined(BOARD_MIBO64_ATSAM4S)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.Init(0);

	// TWI0
	hwpinctrl.PinSetup(PORTNUM_A,  3, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SDA/TWD0
	hwpinctrl.PinSetup(PORTNUM_A,  4, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SCL/TWCK0
	i2capp.devnum = 0;

  #define I2C_IRQ_NUM                   19
  #define I2C_IRQ_HANDLER   IRQ_Handler_19
}

#endif

#if defined(BOARD_MIBO64_ATSAME5X)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// Console: SERCOM0
	hwpinctrl.PinSetup(PORTNUM_A, 4, PINCFG_OUTPUT | PINCFG_AF_3);  // PAD[0] = TX
	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_INPUT  | PINCFG_AF_3);  // PAD[1] = RX
	conuart.Init(0);

	// I2C: SERCOM4
	hwpinctrl.PinSetup(PORTNUM_B,  8, PINCFG_AF_D | PINCFG_PULLUP); // SERCOM4/PAD0 = SDA
	hwpinctrl.PinSetup(PORTNUM_B,  9, PINCFG_AF_D | PINCFG_PULLUP); // SERCOM4/PAD1 = SCL
	i2capp.devnum = 4;

  #define I2C_IRQ_NUM                   62  // SERCOM4_0: PREC
  #define I2C_IRQ_HANDLER   IRQ_Handler_62

	// additional IRQ vectors are required here
	setup_irq(63);
	setup_irq(64);
	setup_irq(65);
}

// This processor has per interrupt flag an interrupt line

extern "C" void IRQ_Handler_63() // SERCOM4_1: AMATCH
{
	i2capp.HandleIrq();
}

extern "C" void IRQ_Handler_64() // SERCOM4_2: DRDY
{
	i2capp.HandleIrq();
}

extern "C" void IRQ_Handler_65() // SERCOM4_3_7: ERROR
{
	i2capp.HandleIrq();
}


#endif

#if defined(BOARD_ARDUINO_DUE)  // ATSAM3XE8

TGpioPin  led1pin(PORTNUM_B, 27, false); // D13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// UART - On the Arduino programmer interface
	hwpinctrl.PinSetup(PORTNUM_A, 8, PINCFG_INPUT | PINCFG_AF_0);  // UART_RXD
	hwpinctrl.PinSetup(PORTNUM_A, 9, PINCFG_OUTPUT | PINCFG_AF_0); // UART_TXD
	conuart.Init(0);  // UART

	// TWI1
	hwpinctrl.PinSetup(PORTNUM_B, 12, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SDA/TWD1
	hwpinctrl.PinSetup(PORTNUM_B, 13, PINCFG_AF_0 | PINCFG_PULLUP); // TWI0: SCL/TWCK1
	i2capp.devnum = 1;

  #define I2C_IRQ_NUM                   23
  #define I2C_IRQ_HANDLER   IRQ_Handler_23
}

#endif

#if defined(BOARD_MIN_F103)

TGpioPin  led1pin(2, 13, false); // PC13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_1);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_1);  // USART1_RX
	conuart.Init(1);

	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_B,  6, PINCFG_AF_0 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B,  7, PINCFG_AF_0 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SDA

	i2capp.devnum = 1;

  #define I2C_IRQ_NUM                   31
  #define I2C_IRQ_HANDLER   IRQ_Handler_31
}
#endif

#if defined(BOARD_MIBO20_STM32F070)

TGpioPin  led1pin(PORTNUM_B, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART2
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_1);  // USART2_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_1 | PINCFG_PULLUP);  // USART2_RX
	conuart.Init(2);

	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_A,  9, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_A, 10, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SDA

	i2capp.devnum = 1;

  #define I2C_IRQ_NUM                   23
  #define I2C_IRQ_HANDLER   IRQ_Handler_23
}
#endif

#if defined(BOARD_MIBO20_STM32F030)

TGpioPin  led1pin(PORTNUM_B, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  2,  PINCFG_OUTPUT | PINCFG_AF_1);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A,  3,  PINCFG_INPUT  | PINCFG_AF_1 | PINCFG_PULLUP);  // USART1_RX
	conuart.Init(1);

	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_A,  9, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_A, 10, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_SPEED_FAST); // I2C1_SDA

	i2capp.devnum = 1;

  #define I2C_IRQ_NUM                   23
  #define I2C_IRQ_HANDLER   IRQ_Handler_23
}
#endif

#if defined(BOARD_MIBO64_STM32F405)

TGpioPin  led1pin(PORTNUM_C, 13, false); // PC13

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);

	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_B,  6, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_PULLUP); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B,  7, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_PULLUP); // I2C1_SDA

	i2capp.devnum = 1;

  #define I2C_IRQ_NUM                   31
  #define I2C_IRQ_HANDLER   IRQ_Handler_31
}
#endif


#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746)

TGpioPin  led1pin(1, 0, false);
TGpioPin  led2pin(1, 7, false);
TGpioPin  led3pin(1, 14, false);

#define LED_COUNT 3

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

  // USART3: Stlink USB / Serial converter
	hwpinctrl.PinSetup(PORTNUM_D, 8,  PINCFG_OUTPUT | PINCFG_AF_7); // USART3_TX
	hwpinctrl.PinSetup(PORTNUM_D, 9,  PINCFG_INPUT  | PINCFG_AF_7); // USART3_RX
	conuart.Init(3);

	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_B,  8, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_PULLUP); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B,  9, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_PULLUP); // I2C1_SDA

	i2capp.devnum = 1;

  #define I2C_IRQ_NUM                   31
  #define I2C_IRQ_HANDLER   IRQ_Handler_31
}

#endif

#if defined(BOARD_NUCLEO_G474RE)

TGpioPin  led1pin(PORTNUM_A, 5, false);

#define LED_COUNT 1

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

#if 0
  // USART2: Stlink USB / Serial converter
	hwpinctrl.PinSetup(PORTNUM_A, 2,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART2.TX
	hwpinctrl.PinSetup(PORTNUM_A, 3,  PINCFG_INPUT  | PINCFG_AF_7);  // USART2.RX
	conuart.Init(2);
#else
  // USART1: alternative, using external debugger probe
	hwpinctrl.PinSetup(PORTNUM_C, 4,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1.TX
	hwpinctrl.PinSetup(PORTNUM_C, 5,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1.RX
	conuart.Init(1);
#endif

	// I2C1
	// open drain mode have to be used, otherwise it won't work
	// External pull-ups are required !
	hwpinctrl.PinSetup(PORTNUM_B,  8, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_PULLUP); // I2C1_SCL
	hwpinctrl.PinSetup(PORTNUM_B,  9, PINCFG_AF_4 | PINCFG_OPENDRAIN | PINCFG_PULLUP); // I2C1_SDA

	i2capp.devnum = 1;

  #define I2C_IRQ_NUM                   31
  #define I2C_IRQ_HANDLER   IRQ_Handler_31
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

extern "C" void I2C_IRQ_HANDLER()
{
	i2capp.HandleIrq();
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

	//TRACE("hbcounter = %u, systick = %u\r\n", hbcounter, systick);
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
	//mcu_enable_icache(); // enable instruction cache if present

	clockcnt_init();

	// go on with the hardware initializations
	setup_board();

	traces_init();
	tracebuf.waitsend = true;  // start in wait mode

	TRACE("\r\n--------------------------\r\n");
	TRACE("NVCM I2C SLAVE TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	// Setup the I2C Application

	if (!i2capp.Init(0x50, 0x00))
	{
		TRACE("I2C Application Init FAILED!\r\n");
	}
	else
	{
		TRACE("I2C Application Initialized.\r\n");
	}

	// initialize the data with some content
	i2capp.data[0] = 0x33;
	i2capp.data[1] = 0x44;
	i2capp.data[2] = 0x55;
	i2capp.data[3] = 0x66;
	i2capp.data[4] = 0x77;
	i2capp.data[5] = 0x88;
	i2capp.data[6] = 0x99;
	i2capp.data[7] = 0xAA;
	i2capp.data[8] = 0xBB;
	i2capp.data[9] = 0xCC;
	i2capp.data[10] = 0xDD;

	// Enable The I2C IRQ

	setup_irq(I2C_IRQ_NUM);

  TRACE("\r\nStarting main cycle...\r\n");

	tracebuf.waitsend = false;  // go to buffered (fast) mode

	SysTick_Config(SystemCoreClock / 1000);

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

	unsigned t0, t1;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		tracebuf.Run();  // send the buffered characters

		idle_task();

		if (t1-t0 > hbclocks)
		{
			heartbeat_task();
			t0 = t1;
		}
	}
}

// ----------------------------------------------------------------------------
