// test_dht11.cpp

#include "main.h"
#include "dht11.h"
#include "traces.h"

TDht11  dht11;

void test_dht11()
{
	TRACE("DHT11 test...\r\n");

#if defined(BOARD_VERTIBO_A)
	dht11.pin_dio.Assign(PORTNUM_D, 11, false);
#elif defined(BOARD_MIN_F103)
	dht11.pin_dio.Assign(PORTNUM_B,  5, false);
#else
  #warning "Unknown board for DHT11 test"
	TRACE("The DHT11 test is not available for this board.\r\n");
	return;
#endif

	dht11.Init();

	unsigned t0, t1;
	t0 = CLOCKCNT;

	while (1)
	{
		t1 = CLOCKCNT;
		if (t1 - t0 > SystemCoreClock / 1)
		{
			led1pin.Toggle();

			if (!dht11.Measure())
			{
				TRACE("DHT11 measure failed!\r\n");
			}
			t0 = t1;
		}
	}

}


