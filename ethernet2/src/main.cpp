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

#include "string.h"
#include "platform.h"
#include "hwpins.h"
#include "hwclkctrl.h"
#include "hwuart.h"
#include "hweth.h"
#include "cppinit.h"
#include "clockcnt.h"
#include "stdio.h"

#include "traces.h"

THwUart   conuart;  // console uart

#define ETH_RX_PACKETS  16
#define ETH_TX_PACKETS   4

uint8_t   my_ip_address[4] = {192, 168, 2, 10};

__attribute__((aligned(64)))
uint8_t   eth_rx_desc_mem[sizeof(HW_ETH_DMA_DESC) * ETH_RX_PACKETS];

__attribute__((aligned(64)))
uint8_t   eth_tx_desc_mem[sizeof(HW_ETH_DMA_DESC) * ETH_TX_PACKETS];

__attribute__((aligned(64)))
uint8_t   eth_rx_packet_mem[HWETH_MAX_PACKET_SIZE * ETH_RX_PACKETS];

THwEth    eth;

#if defined(BOARD_NUCLEO_F746)

TGpioPin  pin_led_green(1, 0, false);
TGpioPin  pin_led_blue(1, 7, false);
TGpioPin  pin_led_red(1, 14, false);

#define LED_COUNT 3

void setup_board()
{
	// nucleo board leds
	pin_led_green.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_led_blue.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_led_red.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

  // USART3: Stlink USB / Serial converter
	// USART3_TX: PD.8
	hwpinctrl.PinSetup(3, 8,  PINCFG_OUTPUT | PINCFG_AF_7);
	// USART3_RX: Pd.9
	hwpinctrl.PinSetup(3, 9,  PINCFG_INPUT  | PINCFG_AF_7);

	conuart.Init(3); // USART3

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

	uint32_t pinfl = PINCFG_SPEED_FAST | PINCFG_AF_11;  // do not use PINCFG_SPEED_VERYFAST !!

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

	/* Ethernet pins configuration ************************************************

	        RMII_REF_CLK ----------------------> PD0
	        RMII_MDIO -------------------------> PD9
	        RMII_MDC --------------------------> PD8
	        RMII_MII_CRS_DV -------------------> PD4
	        RMII_MII_RXD0 ---------------------> PD5
	        RMII_MII_RXD1 ---------------------> PD6
	        RMII_MII_RXER ---------------------> PD7
	        RMII_MII_TX_EN --------------------> PD1
	        RMII_MII_TXD0 ---------------------> PD2
	        RMII_MII_TXD1 ---------------------> PD3
	*/

	uint32_t pinfl = PINCFG_SPEED_FAST | PINCFG_AF_0;

	hwpinctrl.PinSetup(PORTNUM_D, 0, pinfl); // REF CLK
	hwpinctrl.PinSetup(PORTNUM_D, 9, pinfl); // MDIO
	hwpinctrl.PinSetup(PORTNUM_D, 8, pinfl); // MDC
	hwpinctrl.PinSetup(PORTNUM_D, 4, pinfl); // CRS_DV
	hwpinctrl.PinSetup(PORTNUM_D, 5, pinfl); // RXD0
	hwpinctrl.PinSetup(PORTNUM_D, 6, pinfl); // RXD1
	hwpinctrl.PinSetup(PORTNUM_D, 7, pinfl); // RXER
	hwpinctrl.PinSetup(PORTNUM_D, 1, pinfl); // TX_EN
	hwpinctrl.PinSetup(PORTNUM_D, 2, pinfl); // TXD0
	hwpinctrl.PinSetup(PORTNUM_D, 3, pinfl); // TXD1

	// Extra PHY Signals
	hwpinctrl.PinSetup(PORTNUM_C, 10, PINCFG_OUTPUT | PINCFG_GPIO_INIT_1); // PHY RESET
	hwpinctrl.PinSetup(PORTNUM_A, 14, PINCFG_INPUT | PINCFG_PULLUP); // PHY INTERRUPT

	eth.phy_address = 0;
}

#endif

#if defined(BOARD_VERTIBO_A)

TGpioPin  led1pin(PORTNUM_A, 29, false);

TGpioPin  pin_fpga_cfg(PORTNUM_C, 9, false);


void setup_board()
{
	led1pin.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_fpga_cfg.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

#if 1
	hwpinctrl.PinSetup(PORTNUM_A,  6,  PINCFG_OUTPUT | PINCFG_AF_1);  // PCK0 = FPGA.CLK_IN

	PMC->PMC_SCER = (1 << 8); // enable PCK0

	PMC->PMC_PCK[0] = 0
		| (1 << 0)  // CSS(3): 1 = MAIN CLK
		| (9 << 4)  // PRES(8): divisor - 1
	;
#endif

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.baudrate = 115200;
	conuart.Init(0);

	/* Ethernet pins configuration ************************************************

	        RMII_REF_CLK ----------------------> PD0
	        RMII_MDIO -------------------------> PD9
	        RMII_MDC --------------------------> PD8
	        RMII_MII_CRS_DV -------------------> PD4
	        RMII_MII_RXD0 ---------------------> PD5
	        RMII_MII_RXD1 ---------------------> PD6
	        RMII_MII_RXER ---------------------> PD7
	        RMII_MII_TX_EN --------------------> PD1
	        RMII_MII_TXD0 ---------------------> PD2
	        RMII_MII_TXD1 ---------------------> PD3
	*/

	uint32_t pinfl = PINCFG_SPEED_FAST | PINCFG_AF_0;

	hwpinctrl.PinSetup(PORTNUM_D, 0, pinfl); // REF CLK
	hwpinctrl.PinSetup(PORTNUM_D, 9, pinfl); // MDIO
	hwpinctrl.PinSetup(PORTNUM_D, 8, pinfl); // MDC
	hwpinctrl.PinSetup(PORTNUM_D, 4, pinfl); // CRS_DV
	hwpinctrl.PinSetup(PORTNUM_D, 5, pinfl); // RXD0
	hwpinctrl.PinSetup(PORTNUM_D, 6, pinfl); // RXD1
	hwpinctrl.PinSetup(PORTNUM_D, 7, pinfl); // RXER       // Tie to the GND !!!
	hwpinctrl.PinSetup(PORTNUM_D, 1, pinfl); // TX_EN
	hwpinctrl.PinSetup(PORTNUM_D, 2, pinfl); // TXD0
	hwpinctrl.PinSetup(PORTNUM_D, 3, pinfl); // TXD1

	eth.phy_address = 1;
}

#endif

#if defined(BOARD_ENEBO_A)

TGpioPin  pin_led_blue(PORTNUM_D, 13, true);
TGpioPin  pin_led_green(PORTNUM_D, 14, true);
TGpioPin  pin_led_red(PORTNUM_A, 20, true);

TGpioPin  pin_eth_reset(PORTNUM_A, 19, false);

#define LED_COUNT 3

void setup_board()
{
	pin_led_blue.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_led_green.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);
	pin_led_red.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	pin_led_red.Set1(); // turn on the RED

	pin_eth_reset.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_0);

	hwpinctrl.PinSetup(PORTNUM_A,  9,  PINCFG_INPUT  | PINCFG_AF_0);  // UART0_RX
	hwpinctrl.PinSetup(PORTNUM_A, 10,  PINCFG_OUTPUT | PINCFG_AF_0);  // UART0_TX
	conuart.baudrate = 115200;
	conuart.Init(0);

	// Ethernet clock output:
	hwpinctrl.PinSetup(PORTNUM_A,  18,  PINCFG_OUTPUT | PINCFG_AF_B);  // PCK2 = Ethernet 25 M Clock

	PMC->PMC_SCER = (1 << 10); // enable PCK2

	PMC->PMC_PCK[2] = 0
		| (2 << 0)  // CSS(3): 2 = PLLA
		| ((12 - 1) << 4)  // PRES(8): divisor - 1
	;

	/* Ethernet pins configuration ************************************************

	        RMII_REF_CLK ----------------------> PD0
	        RMII_MDIO -------------------------> PD9
	        RMII_MDC --------------------------> PD8
	        RMII_MII_CRS_DV -------------------> PD4
	        RMII_MII_RXD0 ---------------------> PD5
	        RMII_MII_RXD1 ---------------------> PD6
	        RMII_MII_RXER ---------------------> PD7
	        RMII_MII_TX_EN --------------------> PD1
	        RMII_MII_TXD0 ---------------------> PD2
	        RMII_MII_TXD1 ---------------------> PD3
	*/

	uint32_t pinfl = PINCFG_SPEED_FAST | PINCFG_AF_0;

	hwpinctrl.PinSetup(PORTNUM_D, 0, pinfl); // REF CLK
	hwpinctrl.PinSetup(PORTNUM_D, 9, pinfl); // MDIO
	hwpinctrl.PinSetup(PORTNUM_D, 8, pinfl); // MDC
	hwpinctrl.PinSetup(PORTNUM_D, 4, pinfl); // CRS_DV
	hwpinctrl.PinSetup(PORTNUM_D, 5, pinfl); // RXD0
	hwpinctrl.PinSetup(PORTNUM_D, 6, pinfl); // RXD1
	hwpinctrl.PinSetup(PORTNUM_D, 7, pinfl); // RXER       // Tie to the GND !!!
	hwpinctrl.PinSetup(PORTNUM_D, 1, pinfl); // TX_EN
	hwpinctrl.PinSetup(PORTNUM_D, 2, pinfl); // TXD0
	hwpinctrl.PinSetup(PORTNUM_D, 3, pinfl); // TXD1

	eth.phy_address = 1;

	delay_us(10);

	pin_eth_reset.Set1(); // start the phy

	delay_us(100);
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

	/* Ethernet pins configuration ************************************************

	        RMII_REF_CLK ----------------------> PD0
	        RMII_MDIO -------------------------> PD9
	        RMII_MDC --------------------------> PD8
	        RMII_MII_CRS_DV -------------------> PD4
	        RMII_MII_RXD0 ---------------------> PD5
	        RMII_MII_RXD1 ---------------------> PD6
	        RMII_MII_RXER ---------------------> PD7
	        RMII_MII_TX_EN --------------------> PD1
	        RMII_MII_TXD0 ---------------------> PD2
	        RMII_MII_TXD1 ---------------------> PD3
	*/

	uint32_t pinfl = PINCFG_SPEED_FAST | PINCFG_AF_0;

	hwpinctrl.PinSetup(PORTNUM_D, 0, pinfl); // REF CLK
	hwpinctrl.PinSetup(PORTNUM_D, 9, pinfl); // MDIO
	hwpinctrl.PinSetup(PORTNUM_D, 8, pinfl); // MDC
	hwpinctrl.PinSetup(PORTNUM_D, 4, pinfl); // CRS_DV
	hwpinctrl.PinSetup(PORTNUM_D, 5, pinfl); // RXD0
	hwpinctrl.PinSetup(PORTNUM_D, 6, pinfl); // RXD1
	hwpinctrl.PinSetup(PORTNUM_D, 7, pinfl); // RXER
	hwpinctrl.PinSetup(PORTNUM_D, 1, pinfl); // TX_EN
	hwpinctrl.PinSetup(PORTNUM_D, 2, pinfl); // TXD0
	hwpinctrl.PinSetup(PORTNUM_D, 3, pinfl); // TXD1


	eth.phy_address = 1;
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

typedef struct TEthernetHeader
{
	uint8_t   dest_mac[6];  /**< Destination node */
	uint8_t   src_mac[6];   /**< Source node */
	uint16_t  ethertype;    /**< Protocol or length */
//
} __attribute__((packed))  TEthernetHeader, * PEthernetHeader;

typedef struct
{
  uint16_t        echead;  // 0x1000 + length

  uint8_t         cmd;
  uint8_t         idx;
  uint16_t        slaveaddr;
  uint16_t        memaddr;
  uint16_t        len_flags;
  uint16_t        irq;
  uint16_t        data;
  uint16_t     	  wkc;
//
} __attribute__((packed))  TecFrame, * PecFrame;

__attribute__((aligned(64)))
uint8_t txbuf[1536];

uint64_t eth_nstime = 0;

bool     rx_led_on = false;
bool     tx_led_on = false;
uint32_t last_rx_time = 0;
uint32_t last_tx_time = 0;

uint8_t ecidx = 1;

void send_test_frame(unsigned tcnt)
{
	PEthernetHeader peth;
	uint32_t idx;

	//TRACE("Sending test frame %i...\r\n", tcnt);

	memset(&txbuf[0], 0, 64);

	peth = PEthernetHeader(&txbuf[0]);
	peth->dest_mac[0] = 0xFF;
	peth->dest_mac[1] = 0xFF;
	peth->dest_mac[2] = 0xFF;
	peth->dest_mac[3] = 0xFF;
	peth->dest_mac[4] = 0xFF;
	peth->dest_mac[5] = 0xFF;
	peth->src_mac[0] = eth.mac_address[0];
	peth->src_mac[1] = eth.mac_address[1];
	peth->src_mac[2] = eth.mac_address[2];
	peth->src_mac[3] = eth.mac_address[3];
	peth->src_mac[4] = eth.mac_address[4];
	peth->src_mac[5] = eth.mac_address[5];
	peth->ethertype = 0xA488; // ethercat

	PecFrame  pecframe = (PecFrame)&txbuf[sizeof(TEthernetHeader)];
	pecframe->echead = 0x1000 + sizeof(TecFrame) - 2;
	pecframe->cmd = 7; // BRD
	pecframe->idx = ecidx;  ++ecidx;
	pecframe->slaveaddr = 0x0000;
	pecframe->memaddr = 0;
	pecframe->len_flags = 2;
	pecframe->irq = 0;
	pecframe->data = 0;
	pecframe->wkc = 0;

	// send the packed
	if (!eth.TrySend(&idx, &txbuf[0], 60))
	{
		TRACE("Packet send failed.\r\n");
	}
	else
	{
		tx_led_on = true;
		last_tx_time = CLOCKCNT;
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
	if (!hwclkctrl.InitCpuClock(MCU_INPUT_FREQ, clockspeed))
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
	TRACE("NVCM ETHERNET TEST2\r\n");
	TRACE("Board: \"%s\"\r\n", BOARD_NAME);
	TRACE("SystemCoreClock: %u\r\n", SystemCoreClock);

	uint32_t n;

	//eth.promiscuous_mode = true;
	//eth.promiscuous_mode = false;
	//eth.hw_ip_checksum = false;
	//eth.loopback = true;
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

	TRACE("Waiting ETH packets...\r\n");

	uint32_t idx;
	uint8_t * pdata;
	uint32_t  datalen;
	PEthernetHeader peh;

	unsigned t0, t1;

	t0 = CLOCKCNT;

	int tcnt = 0;
	unsigned rxcnt = 0;

	bool waslinkup = false;

	while (1)
	{
		t1 = CLOCKCNT;

		eth.PhyStatusPoll(); // must be called regularly

		if (waslinkup != eth.link_up)
		{
			if (eth.link_up)
			{
				TRACE("ETH LINK: UP\r\n");
			}
			else
			{
				TRACE("ETH LINK: DOWN\r\n");
			}
			waslinkup = eth.link_up;
		}

		if (eth.TryRecv(&idx, (void * *)&pdata, &datalen))
		{
			rx_led_on = true;
			last_rx_time = t1;
			++rxcnt;

			peh = PEthernetHeader(pdata);
			uint16_t etype = __REV16(peh->ethertype);

			TRACE("%u. frame len = %u, type = %04X\r\n", rxcnt, datalen, etype);

			TRACE("  dst = %02X:%02X:%02X:%02X:%02X:%02X",
				peh->dest_mac[0],
				peh->dest_mac[1],
				peh->dest_mac[2],
				peh->dest_mac[3],
				peh->dest_mac[4],
				peh->dest_mac[5]
			);
			TRACE(", src = %02X:%02X:%02X:%02X:%02X:%02X",
				peh->src_mac[0],
				peh->src_mac[1],
				peh->src_mac[2],
				peh->src_mac[3],
				peh->src_mac[4],
				peh->src_mac[5]
			);
			TRACE("\r\n");

			#if 1 // display contents
				for (n = 0; n < datalen; ++n)
				{
					if ((n > 0) && ((n % 16) == 0)) TRACE("\r\n");
					TRACE(" %02X", pdata[n]);
				}
				TRACE("\r\n");
			#endif


			eth.ReleaseRxBuf(idx); // must be called !
		}

		if (rx_led_on && (CLOCKCNT - last_rx_time > SystemCoreClock / 10))
		{
			rx_led_on = false;
		}

		if (rx_led_on)
		{
			pin_led_red.Set1();
		}
		else
		{
			pin_led_red.Set0();
		}

		if (tx_led_on && (CLOCKCNT - last_tx_time > SystemCoreClock / 10))
		{
			tx_led_on = false;
		}

		if (tx_led_on)
		{
			pin_led_green.Set1();
		}
		else
		{
			pin_led_green.Set0();
		}


		if (t1 - t0 > SystemCoreClock / 2)
		{
			if (eth.link_up)
			{
				++tcnt;
				send_test_frame(tcnt);
			}

			t0 = t1;
		}

	}
}

// ----------------------------------------------------------------------------
