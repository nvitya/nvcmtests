// traces.cpp

#include "traces.h"

char trace_buffer[4096];

void traces_init()
{
	tracebuf.Init(&conuart, &trace_buffer[0], sizeof(trace_buffer));
}




