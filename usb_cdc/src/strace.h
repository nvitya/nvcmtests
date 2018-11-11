// strace.h

#ifndef STRACE_H_
#define STRACE_H_

#include "hwuart.h"

void strace_init(THwUart * auart, bool abuffered);

void strace_putc(char achar);

void strace_run();

void strace_flush();

void strace(const char* fmt, ...);

#endif /* STRACE_H_ */
