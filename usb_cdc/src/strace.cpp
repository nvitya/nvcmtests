// strace.cpp

#include <stdio.h>
#include <stdarg.h>

#include "strace.h"

#define STRACE_BUF_SIZE  1024 * 4

char strace_buf[STRACE_BUF_SIZE];

int strace_index_write = 0;
int strace_index_read = 0;

THwUart * strace_uart;

bool strace_buffered = false;

void strace_init(THwUart * auart, bool abuffered)
{
	strace_uart = auart;
	strace_buffered = abuffered;
}

void strace_putc(char achar)
{
	if (strace_buffered)
	{
		strace_buf[strace_index_write] = achar;
		++strace_index_write;
		if (strace_index_write >= STRACE_BUF_SIZE)  strace_index_write = 0;
	}
	else
	{
		strace_uart->SendChar(achar);
	}
}

void strace_run()
{
	if (strace_index_write != strace_index_read)
	{
		if (strace_uart->TrySendChar(strace_buf[strace_index_read]))
		{
			++strace_index_read;
			if (strace_index_read >= STRACE_BUF_SIZE)  strace_index_read = 0;
		}
	}
}

void strace_flush()
{
	while (strace_index_write != strace_index_read)
	{
		strace_run();
	}
}

#define FMT_BUFFER_SIZE  256

void strace(const char* fmt, ...)
{
  va_list arglist;
  va_start(arglist, fmt);
  char * pch;

  // allocate format buffer on the stack:
  char fmtbuf[FMT_BUFFER_SIZE];

  pch = &fmtbuf[0];

  *pch = 0;

  vsnprintf(pch, FMT_BUFFER_SIZE, fmt, arglist);

  while (*pch != 0)
  {
  	strace_putc(*pch);
    ++pch;
  }

  va_end(arglist);
}
