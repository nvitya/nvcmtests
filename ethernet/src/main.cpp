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

#include "traces.h"

THwUart   conuart;  // console uart

#define ETH_RX_PACKETS  16
#define ETH_TX_PACKETS   4

uint8_t   my_ip_address[4] = {192, 168, 2, 10};

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

typedef struct TEthernetHeader
{
	uint8_t   dest_mac[6];  /**< Destination node */
	uint8_t   src_mac[6];   /**< Source node */
	uint16_t  ethertype;    /**< Protocol or length */
//
} __attribute__((packed))  TEthernetHeader, * PEthernetHeader;

typedef struct TArpHeader
{
	uint16_t  hrd;    /**< Format of hardware address */
	uint16_t  pro;    /**< Format of protocol address */
	uint8_t   hln;    /**< Length of hardware address */
	uint8_t   pln;    /**< Length of protocol address */
	uint16_t  op;     /**< Operation */
	uint8_t   sha[6]; /**< Sender hardware address */
	uint8_t   spa[4]; /**< Sender protocol address */
	uint8_t   tha[6]; /**< Target hardware address */
	uint8_t   tpa[4]; /**< Target protocol address */
//
} __attribute__((packed))  TArpHeader, * PArpHeader;

typedef struct TIpHeader
{
	uint8_t   hl_v;   /**< Header length and version */
	uint8_t   tos;    /**< Type of service */
	uint16_t  len;    /**< Total length */
	uint16_t  id;     /**< Identification */
	uint16_t  off;    /**< Fragment offset field */
	uint8_t   ttl;    /**< Time to live */
	uint8_t   p;      /**< Protocol */
	uint16_t  sum;    /**< Checksum */
	uint8_t   src[4]; /**< Source IP address */
	uint8_t   dst[4]; /**< Destination IP address */
//
} __attribute__((packed))  TIpHeader, * PIpHeader;

typedef struct icmp_echo_header
{
	uint8_t  type;   /**< Type of message */
	uint8_t  code;   /**< Type subcode */
	uint16_t cksum;  /**< 1's complement cksum of struct */
	uint16_t id;     /**< Identifier */
	uint16_t seq;    /**< Sequence number */
//
} __attribute__((packed))  TIcmpHeader, * PIcmpHeader;

uint8_t pbuf[1536];

void answer_arp(uint8_t * pdata)
{
	uint32_t  n;
	PEthernetHeader peth;
	PArpHeader parp;
	uint32_t idx;
	peth = PEthernetHeader(pdata);
	parp = PArpHeader(pdata + sizeof(TEthernetHeader));

	TRACE("ARP request for %u.%u.%u.%u\r\n", parp->tpa[0], parp->tpa[1], parp->tpa[2], parp->tpa[3] );

	if (*(uint32_t *)&(parp->tpa) == *(uint32_t *)&(my_ip_address))
	{
		TRACE("Answering ARP...\r\n");

		// prepare the answer
		memcpy(&pbuf[0], pdata, sizeof(TEthernetHeader) + sizeof(TArpHeader));

		peth = PEthernetHeader(&pbuf[0]);
		parp = PArpHeader(peth + 1);

		parp->op = 0x0200; // ARP Reply (byte swapped)

		// fill the ETH addresses
		for (n = 0; n < 6; ++n)
		{
			peth->dest_mac[n] = peth->src_mac[n];
			parp->tha[n] = parp->sha[n];
			peth->src_mac[n] = eth.mac_address[n];
			parp->sha[n] = eth.mac_address[n];
		}

		// fill the IP addresses
		for (n = 0; n < 4; ++n)
		{
			parp->tpa[n] = parp->spa[n];
			parp->spa[n] = my_ip_address[n];
		}

		// send the packed
		if (!eth.TrySend(&idx, &pbuf[0], sizeof(TEthernetHeader) + sizeof(TArpHeader)))
		{
			TRACE("Packet send failed.\r\n");
		}
	}
}

uint16_t calc_icmp_checksum(void * pdata, uint32_t datalen)
{
	//((uint8_t *)&pdata)[datalen] = 0; // for odd size handling

	uint32_t n;
	uint32_t clen = ((datalen + 1) >> 1);
	uint32_t sum = 0;
	uint16_t * pd16 = (uint16_t *)pdata;

	for (n = 0; n < clen; ++n)
	{
		sum += __REV16(*pd16++);
	}

	sum = (sum & 0xffff) + (sum >> 16);


	return (uint16_t) (~sum);
}


void answer_ip(uint8_t * pdata, uint16_t datalen)
{
	uint32_t n;
	PEthernetHeader  peth;
	PIpHeader        pih;
	uint32_t         idx;
	peth = PEthernetHeader(pdata);
	pih  = PIpHeader(pdata + sizeof(TEthernetHeader));

	uint8_t prot = pih->p;

	TRACE("IP protocol = %u\r\n", prot);

	if (prot == 1) // ICMP
	{
		PIcmpHeader pich = PIcmpHeader(pih + 1);
		if (pich->type == 8) // echo request ?
		{
			TRACE("Echo request detected.\r\n");

			// prepare the answer
			memcpy(&pbuf[0], pdata, datalen);

			peth = PEthernetHeader(&pbuf[0]);
			pih = PIpHeader(peth + 1);
			pich = PIcmpHeader(pih + 1);

			// fill the ETH addresses
			for (n = 0; n < 6; ++n)
			{
				peth->dest_mac[n] = peth->src_mac[n];
				peth->src_mac[n] = eth.mac_address[n];
			}

			// fill the IP addresses
			for (n = 0; n < 4; ++n)
			{
				pih->dst[n] = pih->src[n];
				pih->src[n] = my_ip_address[n];
			}

			pich->type = 0; // ICMP ECHO reply
			pich->code = 0;
			pich->cksum = 0;

			// the ICMP message can contain arbitrary length data, we need to calculate its size first
			uint16_t icmp_len = __REV16(pih->len) - sizeof(TIpHeader);
			pich->cksum = __REV16(calc_icmp_checksum(pich, icmp_len));

			// send the packed
			if (!eth.TrySend(&idx, &pbuf[0], datalen))
			{
				TRACE("ICMP Packet send failed.\r\n");
				return;
			}

			TRACE("ICMP response sent.\r\n");
		}
	}
}

void idle_task()
{
	uint32_t n;
	uint32_t idx;
	uint8_t * pdata;
	uint32_t  datalen;
	PEthernetHeader peh;

	eth.PhyStatusPoll(); // must be called regularly

	if (eth.TryRecv(&idx, (void * *)&pdata, &datalen))
	{
		TRACE("Eth frame received, len = %u\r\n", datalen);
		peh = PEthernetHeader(pdata);
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

		uint16_t etype = __REV16(peh->ethertype);

		TRACE(", type = %04X\r\n", etype);

		if (etype == 0x0806) // ARP ?
		{
			answer_arp(pdata);
		}
		else if (etype == 0x800) // IP
		{
			answer_ip(pdata, datalen);
		}

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
