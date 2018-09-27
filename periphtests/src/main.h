// main.h

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include "hwpins.h"
#include "hwuart.h"
#include "ledandkey.h"
#include "clockcnt.h"
#include "traces.h"

extern TGpioPin    led1pin;
extern THwUart     conuart;
extern TLedAndKey  ledandkey;

void idle_task();

#endif /* SRC_MAIN_H_ */
