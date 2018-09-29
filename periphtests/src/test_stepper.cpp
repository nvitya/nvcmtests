// test_stepper.cpp

#include "main.h"
#include "hwpins.h"
#include "clockcnt.h"

#include "stepper_4u.h"

#include "traces.h"

TStepper_4u  stepper;

void test_stepper()
{
	TRACE("Stepper motor test\r\n");

#if defined(BOARD_VERTIBO_A)
	stepper.pin[0].Assign(PORTNUM_D, 18, false);
	stepper.pin[1].Assign(PORTNUM_D, 19, false);
	stepper.pin[2].Assign(PORTNUM_D, 20, false);
	stepper.pin[3].Assign(PORTNUM_D, 21, false);
#else
  #warning "Unknown board for stepper test"
	TRACE("The stepper test is not available for this board.\r\n");
	return;
#endif

	stepper.Init();

	unsigned t0, t1;
	t0 = CLOCKCNT;

	int range = 2048;  // exactly one turn
	bool up = true;
	int state = 0;

	while (1)
	{
		t1 = CLOCKCNT;
		if (t1 - t0 > SystemCoreClock / 1)
		{
			led1pin.Toggle();
			t0 = t1;
		}

		if (0 == state)
		{
			if (up)
			{
				stepper.position += range;
			}
			else
			{
				stepper.position -= range;
			}
			up = !up;
			state = 1;
		}
		else if (1 == state)
		{
			if (stepper.position == stepper.actual_pos)
			{
				state = 0;
			}
		}


		stepper.Run();
	}
}



