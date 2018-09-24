// test_spi.cpp

#include "platform.h"
#include "hwpins.h"
#include "hwspi.h"
#include "test_spi.h"
#include "clockcnt.h"

#include "traces.h"

extern TGpioPin led1pin; // defined in main

void test_spi()
{
	TRACE("*** SPI Test ***\r\n");

	THwSpi    spi;
	THwSpi    spi2;
	TGpioPin  pin_cs;

#if defined(MCUF_STM32)

	pin_cs.Assign(PORTNUM_A, 4, false);
	pin_cs.Setup(PINCFG_OUTPUT | PINCFG_GPIO_INIT_1);

	hwpinctrl.PinSetup(PORTNUM_A, 5, PINCFG_AF_0);  // SPI1_SCK
	hwpinctrl.PinSetup(PORTNUM_A, 6, PINCFG_AF_0);  // SPI1_MISO
	hwpinctrl.PinSetup(PORTNUM_A, 7, PINCFG_AF_0);  // SPI1_MOSI

	hwpinctrl.PinSetup(PORTNUM_B, 13, PINCFG_AF_0);  // SPI2_SCK
	hwpinctrl.PinSetup(PORTNUM_B, 14, PINCFG_AF_0);  // SPI2_MISO
	hwpinctrl.PinSetup(PORTNUM_B, 15, PINCFG_AF_0);  // SPI2_MOSI

	// setup clock out
	hwpinctrl.PinSetup(PORTNUM_A, 8, PINCFG_AF_0);  // Clock out
	RCC->CFGR &= ~(7 << 24); // MCO(3): clock out select
	RCC->CFGR |=  (6 << 24); // 4 = system clock, 6 = HSE clock

#endif

	TRACE("Sending 0x5A periodically (1s)...\r\n");

	spi.speed = 2000000;
	spi.Init(1);

	spi2.speed = 8000000;
	spi2.Init(2);


	while (1)
	{
		uint16_t rd;

		pin_cs.Set0();
		spi.TrySendData(0x5A);
		spi.WaitSendFinish();

		spi2.TrySendData(0x5A);
		spi2.WaitSendFinish();

		pin_cs.Set1();

		while (spi.TryRecvData(&rd)) { } // empty the receive side
		while (spi2.TryRecvData(&rd)) { } // empty the receive side

		delay_us(1000000);

		led1pin.Toggle();
	}

}


