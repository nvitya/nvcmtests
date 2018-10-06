// test_keymatrix.cpp

#include "main.h"
#include "keymatrix.h"

TKeyMatrix keymat;

void test_keymatrix()
{
	TRACE("*** Key Matrix test ***\r\n");

#if defined(BOARD_VERTIBO_A)

	keymat.rowpin[0].Assign(PORTNUM_A, 2, false);
	keymat.rowpin[1].Assign(PORTNUM_A, 3, false);
	keymat.rowpin[2].Assign(PORTNUM_A, 4, false);
	keymat.rowpin[3].Assign(PORTNUM_A, 5, false);

	keymat.colpin[0].Assign(PORTNUM_D, 22, false);
	keymat.colpin[1].Assign(PORTNUM_D, 24, false);
	keymat.colpin[2].Assign(PORTNUM_D, 25, false);
	keymat.colpin[3].Assign(PORTNUM_D, 26, false);

	keymat.Init(4, 4);
#else
  #warning "Key matrix test is not implemented for this board"
#endif

	unsigned t0, t1;

	uint32_t prev_keys = 0xF0000000; // some invalid value

	t0 = CLOCKCNT;
	while (1)
	{
		t1 = CLOCKCNT;
		if (t1 - t0 > SystemCoreClock / 2)
		{
			led1pin.Toggle();
			t0 = t1;
		}

		keymat.Run();

		if (prev_keys != keymat.keys)
		{
			TRACE("Keys changed: %08X\r\n", keymat.keys);
			prev_keys = keymat.keys;
		}

		idle_task();
	}
}

