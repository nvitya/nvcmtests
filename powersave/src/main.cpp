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
 *  file:     main.cpp (blinkled)
 *  brief:    Multi-board blinking led example for NVCM
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwclkctrl.h"
#include "hwpins.h"
#include "cppinit.h"
#include "clockcnt.h"

#if defined(BOARD_NUCLEO_F446) || defined(BOARD_NUCLEO_F746) || defined(BOARD_NUCLEO_H743)

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
}

#endif

#if defined(BOARD_NUCLEO_G474RE)

TGpioPin  led1pin(PORTNUM_A, 5, false);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_DISCOVERY_F746)

TGpioPin  led1pin(PORTNUM_I, 1, false);

#define LED_COUNT 1

void setup_board()
{
	// discovery board led on arduino port CN7 / 6
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_DISCOVERY_F072)

TGpioPin  led1pin(PORTNUM_C, 6, false);
TGpioPin  led2pin(PORTNUM_C, 8, false);
TGpioPin  led3pin(PORTNUM_C, 9, false);
TGpioPin  led4pin(PORTNUM_C, 7, false);

#define LED_COUNT 4
#undef USE_DWT_CYCCNT

void setup_board()
{
	// direction leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif


#if defined(BOARD_EVK_IMXRT1020)

TGpioPin  led1pin(1, 5, false); // GPIO_AD_B0_05 = GPIO_1_5

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_MIBO48_XMC4100)

TGpioPin  led1pin(1, 0, true);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif


#if defined(BOARD_BOOT_XMC1200)

TGpioPin  led1pin(0, 0, true);
TGpioPin  led2pin(0, 2, true);
TGpioPin  led3pin(0, 5, true);
TGpioPin  led4pin(0, 6, true);
TGpioPin  led5pin(0, 7, true);

#define LED_COUNT 5

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led4pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led5pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_F407VG_MIN)

TGpioPin  led1pin(4, 0, true);  // PE0

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_STM32F407ZE)

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

void setup_board()
{
	// nucleo board leds
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_ARDUINO_DUE)

TGpioPin  led1pin(1, 27, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_MIN_F103) || defined(BOARD_MIBO48_STM32F303) || defined(BOARD_MIBO48_STM32G473) || defined(BOARD_MIBO64_STM32F405) || defined(BOARD_MIBO64_STM32F070)

TGpioPin  led1pin(PORTNUM_C, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}
#endif

#if defined(BOARD_MIBO20_STM32F030) || defined(BOARD_MIBO20_STM32F070)

TGpioPin  led1pin(PORTNUM_B, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}
#endif


#if defined(BOARD_XPLAINED_SAME70)

TGpioPin  led1pin(2, 8, false);  // C8

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_MIBO100_ATSAME70)

TGpioPin  led1pin(PORTNUM_D, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
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
}

#endif

#if defined(BOARD_MIBO64_ATSAM4S)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_MIBO64_ATSAME5X)

TGpioPin  led1pin(PORTNUM_A, 1, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_XPRESSO_LPC4337)

TGpioPin  led1pin(3, 5, true);
TGpioPin  led2pin(0, 7, true);
TGpioPin  led3pin(3, 7, true);

#define LED_COUNT 3

void setup_board()
{
	// RGB LED
	hwpinctrl.PinSetup(6,  9, PINCFG_OUTPUT | PINCFG_DRIVE_WEAK | PINCFG_AF_0);  // GPIO_3_5
	hwpinctrl.PinSetup(2,  7, PINCFG_OUTPUT | PINCFG_DRIVE_WEAK | PINCFG_AF_0);  // GPIO_0_7
	hwpinctrl.PinSetup(6, 11, PINCFG_OUTPUT | PINCFG_DRIVE_WEAK | PINCFG_AF_0);  // GPIO_3_7

	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_XPLORER_LPC4330)

TGpioPin  led1pin(1, 12, true); // D2
TGpioPin  led2pin(1, 11, true); // D3

#define LED_COUNT 2

void setup_board()
{
	hwpinctrl.PinSetup(2, 12, PINCFG_OUTPUT | PINCFG_AF_0);  // D2: GPIO_1_12, pad B9
	hwpinctrl.PinSetup(2, 11, PINCFG_OUTPUT | PINCFG_AF_0);  // D3: GPIO_1_11, pad A9

	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_XPRESSO_LPC54608)

TGpioPin  led1pin(2, 2, true);
TGpioPin  led2pin(3, 3, true);
TGpioPin  led3pin(3, 14, true);

#define LED_COUNT 3

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led3pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_MIBO100_LPC546)

TGpioPin  led1pin(1, 3, false);

#define LED_COUNT 1

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_NONE_MK20DN128)

TGpioPin  led1pin(PORTNUM_B, 0, true);
TGpioPin  led2pin(PORTNUM_B, 1, true);

#define LED_COUNT 2

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
}

#endif

#if defined(BOARD_NONE_MKV30F)

TGpioPin  led1pin(PORTNUM_B, 0, true);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
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
}

void cpu_soft_reset()
{
	uint32_t tmp = SCB->AIRCR;
	tmp &= 0x0000FF00;
	tmp |= 0x05FA0004; // write key + reset (SYSRESETREQ)
	SCB->AIRCR = tmp;

	while (1)
	{
		// wait for the reset...
	}
}

void cpu_enter_stop_mode()
{
  /* Select the regulator state in STOP mode */
  //PWR->CR1 |= PWR_CR1_LPDS;
  PWR->CR1 &= ~PWR_CR1_LPDS;

  /* Keep DSTOP mode when Cortex-M7 enter in DEEP-SLEEP */
  PWR->CPUCR &= ~(PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D2 | PWR_CPUCR_PDDS_D3); // stop
  //PWR->CPUCR |= (PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D2 | PWR_CPUCR_PDDS_D3);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /* Ensure that all instructions are done before entering STOP mode */
  __DSB ();
  __ISB ();

  /* Select STOP mode entry */
  /* Request Wait For Interrupt */
  __WFI ();
}

void cpu_enter_standby_mode()
{
  /* Select the regulator state in STOP mode */
  //PWR->CR1 |= PWR_CR1_LPDS;
  PWR->CR1 &= ~PWR_CR1_LPDS;

  /* Keep DSTOP mode when Cortex-M7 enter in DEEP-SLEEP */
  //PWR->CPUCR &= ~(PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D2 | PWR_CPUCR_PDDS_D3); // stop
  PWR->CPUCR |= (PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D2 | PWR_CPUCR_PDDS_D3);  // standby

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /* Ensure that all instructions are done before entering STOP mode */
  __DSB ();
  __ISB ();

  /* Select STOP mode entry */
  /* Request Wait For Interrupt */
  __WFI ();
}

void rtc_writeprotect_disable()
{
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
}

void rtc_writeprotect_enable()
{
  RTC->WPR = 0xFF;
}

#define IRQHANDLER_RTC_WAKEUP  IRQ_Handler_03

extern "C" void IRQHANDLER_RTC_WAKEUP(void)
{
  //HAL_RTCEx_WakeUpTimerIRQHandler(&RTCHandle);

	EXTI_D1->PR1 = EXTI_IMR1_IM19;

  rtc_writeprotect_disable();
	RTC->ISR = 0; // clear the interrupt

  uint32_t tmp;
  tmp = RTC->CR;
  tmp &= ~(RTC_CR_WUTE);    // disable the wakeup timer
  tmp &= ~(RTC_CR_WUTIE);   // disable wakeup interrupt
  RTC->CR = tmp;

  for (int i = 0; i < 100; ++i)
  {
  	if (RTC->ISR) {  }
  }

	cpu_soft_reset();
}

void enter_low_power()
{
	SysTick->CTRL = 0; // stop the systick IRQ

  led1pin.Set0();
  led2pin.Set0();
  led3pin.Set0();

  //cpu_enter_standby_mode();

  PWR->CR1 |= PWR_CR1_DBP;
  rtc_writeprotect_disable();

  uint32_t tmp;
  tmp = RTC->CR;
  tmp &= ~(RTC_CR_WUTE);    // disable the wakeup timer
  tmp &= ~(RTC_CR_WUTIE);   // disable wakeup interrupt
  tmp &= ~(RTC_CR_WUCKSEL); // select RTC/16 for the WU clock
  RTC->CR = tmp;

	RTC->ISR = 0; // clear the wakeup flag

  for (int i = 0; i < 1000; ++i)
  {
  	__NOP();
  }

  NVIC_EnableIRQ(RTC_WKUP_IRQn);

  RTC->WUTR = (4 * 32768/16); // 4s timeout

  EXTI_D1->IMR1 |= EXTI_IMR1_IM19;  // RTC wakeup event
  EXTI->RTSR1 |= EXTI_IMR1_IM19;  // rising edge

  PWR->WKUPCR = 0x3F; // clear all wakeup flags
	EXTI_D1->PR1 = EXTI_IMR1_IM19;

  RTC->CR |= (RTC_CR_WUTIE | RTC_CR_WUTE);  // enable the wakeup timer + IT

  while (1)
  {
  	// wait for the RTC wakeup IRQ
  	cpu_enter_stop_mode();
  	//cpu_enter_standby_mode();
  }
	//__WFI();

	cpu_soft_reset();
}

void rtc_start()
{
  /* Enable write access to Backup domain */
  PWR->CR1 |= PWR_CR1_DBP;

  /* Wait for Backup domain Write protection disable */
  for (int i = 0; i < 10000; ++i)
  {
  	__NOP();
  }

  RCC->BDCR = (0
   | (1 << 15) // RTCEN
   | (1 <<  8) // RTCSEL(2): 1 = LSE
   | (0 <<  6) // LSECSSD
   | (0 <<  5) // LSECSSON
   | (0 <<  3) // LSEDRV(2)
   | (0 <<  2) // LSEBYP
   | (0 <<  1) // LSERDY
   | (1 <<  0) // LSEON
  );

  while (0 == (RCC->BDCR & 1))
  {
  	// wait until LSE + RTC is ready
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

	rtc_start();

	// go on with the hardware initializations
	setup_board();

	mcu_enable_interrupts();

	SysTick_Config(SystemCoreClock / 1000);

  led1pin.Set1();
  led2pin.Set1();
  led3pin.Set1();

	unsigned t0, t1;

	unsigned startsystick = systick;

	t0 = CLOCKCNT;

	// Infinite loop
	while (1)
	{
		t1 = CLOCKCNT;

		if (systick - startsystick > 3000)
		{
			enter_low_power();
		}
	}
}

// ----------------------------------------------------------------------------
