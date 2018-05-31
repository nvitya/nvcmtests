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
 *  file:     main.cpp (ethernet)
 *  brief:    Multi-board ethernet example for NVCM
 *  version:  1.00
 *  date:     2018-05-30
 *  authors:  nvitya
*/

#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "hweth.h"
#include "cppinit.h"
#include "clockcnt.h"

#include "traces.h"

THwUart   conuart;  // console uart

#define ETH_RX_PACKETS  16
#define ETH_TX_PACKETS   4

__attribute__((aligned(32)))
uint8_t   eth_rx_desc_mem[sizeof(HW_ETH_DMA_DESC) * ETH_RX_PACKETS];

__attribute__((aligned(32)))
uint8_t   eth_tx_desc_mem[sizeof(HW_ETH_DMA_DESC) * ETH_TX_PACKETS];

uint8_t   eth_rx_packet_mem[HWETH_MAX_PACKET_SIZE * ETH_RX_PACKETS]   __attribute__((aligned(16)));

THwEth    eth;

#if defined(BOARD_NUCLEO_F746)

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
	// USART3_TX: PD.8
	hwpinctrl.PinSetup(3, 8,  PINCFG_OUTPUT | PINCFG_AF_7);
	// USART3_RX: Pd.9
	hwpinctrl.PinSetup(3, 9,  PINCFG_INPUT  | PINCFG_AF_7);

	conuart.Init(3); // USART3
}

#endif

#if defined(BOARD_XPLAINED_SAME70)

TGpioPin  led1pin(2, 8, false);  // C8

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1 - EDBG
	hwpinctrl.PinSetup(0, 21, PINCFG_INPUT | PINCFG_AF_0);  // USART1_RXD
	MATRIX->CCFG_SYSIO |= (1 << 4); // select PB4 instead of TDI !!!!!!!!!
	hwpinctrl.PinSetup(1,  4, PINCFG_OUTPUT | PINCFG_AF_3); // USART1_TXD
	conuart.Init(0x101); // USART1

	// UART3 - Arduino shield
	//hwpinctrl.PinSetup(3, 28, PINCFG_INPUT | PINCFG_AF_0);  // UART3_RXD
	//hwpinctrl.PinSetup(3, 30, PINCFG_OUTPUT | PINCFG_AF_0); // UART3_TXD
	//uartx2.Init(3); // UART3
}

#endif

#if defined(BOARD_MIBO100_ATSAME70)

TGpioPin  led1pin(PORTNUM_D, 13, false);

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.Init(0);
}

#endif

#if defined(BOARD_DEV_STM32F407VG)

#define SKIP_DTCRAM_EXEC_TEST

TGpioPin  led1pin(4, 0, true);  // PE0

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
}

#endif

#if defined(BOARD_DEV_STM32F407ZE)

#define SKIP_DTCRAM_EXEC_TEST

TGpioPin  led1pin(5, 9, true);  // PF9
TGpioPin  led2pin(5, 10, true);  // PF10

void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	led2pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	// USART1
	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_OUTPUT | PINCFG_AF_7);  // USART1_TX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_INPUT  | PINCFG_AF_7);  // USART1_RX
	conuart.Init(1);
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

	// UART
	hwpinctrl.PinSetup(6, 4, PINCFG_OUTPUT | PINCFG_AF_2);  // UART0_TXD
	hwpinctrl.PinSetup(2, 1, PINCFG_INPUT  | PINCFG_AF_1);  // UART0_RXD
	conuart.Init(0);
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

	hwpinctrl.PinSetup(0, 30, PINCFG_OUTPUT | PINCFG_AF_1); // UART_TX:
	hwpinctrl.PinSetup(0, 29, PINCFG_INPUT  | PINCFG_AF_1); // UART_RX:
	conuart.Init(0);
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
	uint32_t n;
	uint32_t idx;
	uint8_t * pdata;
	uint32_t  datalen;

	eth.PhyStatusPoll(); // must be called regularly

	if (eth.TryRecv(&idx, (void * *)&pdata, &datalen))
	{
		TRACE("Eth frame received, len = %u\r\n", datalen);
#if 0
		for (n = 0; n < datalen; ++n)
		{
			if ((n > 0) && ((n % 15) == 0)) TRACE("\r\n");
			TRACE(" %02X", pdata[n]);
		}
		TRACE("\r\n");
#endif

		eth.ReleaseRxBuf(idx); // must be called !
	}
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

	//conuart.TrySendChar(0x55);
}

void ethernet_init()
{
	uint32_t n;


	/* Ethernet pins configuration ************************************************

	        RMII_REF_CLK ----------------------> PA1
	        RMII_MDIO -------------------------> PA2
	        RMII_MDC --------------------------> PC1
	        RMII_MII_CRS_DV -------------------> PA7
	        RMII_MII_RXD0 ---------------------> PC4
	        RMII_MII_RXD1 ---------------------> PC5
	        RMII_MII_RXER ---------------------> PG2
	        RMII_MII_TX_EN --------------------> PG11
	        RMII_MII_TXD0 ---------------------> PG13
	        RMII_MII_TXD1 ---------------------> PB13
	*/

	uint32_t pinfl = PINCFG_SPEED_FAST | PINCFG_AF_11;

	hwpinctrl.PinSetup(PORTNUM_A,  1, pinfl); // REF CLK
	hwpinctrl.PinSetup(PORTNUM_A,  2, pinfl); // MDIO
	hwpinctrl.PinSetup(PORTNUM_C,  1, pinfl); // MDC
	hwpinctrl.PinSetup(PORTNUM_A,  7, pinfl); // CRS_DV
	hwpinctrl.PinSetup(PORTNUM_C,  4, pinfl); // RXD0
	hwpinctrl.PinSetup(PORTNUM_C,  5, pinfl); // RXD1
	hwpinctrl.PinSetup(PORTNUM_G,  2, pinfl); // RXER
	hwpinctrl.PinSetup(PORTNUM_G, 11, pinfl); // TX_EN
	hwpinctrl.PinSetup(PORTNUM_G, 13, pinfl); // TXD0
	hwpinctrl.PinSetup(PORTNUM_B, 13, pinfl); // TXD1

	/* Enable the Ethernet global Interrupt */
	//HAL_NVIC_SetPriority(ETH_IRQn, 0x7, 0);
	//HAL_NVIC_EnableIRQ(ETH_IRQn);

	eth.phy_address = 0;
	if (!eth.Init(&eth_rx_desc_mem, ETH_RX_PACKETS, &eth_tx_desc_mem, ETH_TX_PACKETS))
	{
		TRACE("ETH INIT FAILED !!!\r\n");
	}
	else
	{
		TRACE("ETH init ok.\r\n");
	}

	// there is no valid rx buffer yet!

	for (n = 0; n < ETH_RX_PACKETS; ++n)
	{
		eth.AssignRxBuf(n, &eth_rx_packet_mem[HWETH_MAX_PACKET_SIZE * n], HWETH_MAX_PACKET_SIZE);
	}

	eth.Start();
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

	TRACE("\r\n--------------------------\r\n");
	TRACE("NVCM ETHERNET TEST\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	ethernet_init();

	TRACE("\r\nStarting main cycle...\r\n");

	SysTick_Config(SystemCoreClock / 1000);

	mcu_enable_interrupts();

	unsigned hbclocks = SystemCoreClock;

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
}

// ----------------------------------------------------------------------------
