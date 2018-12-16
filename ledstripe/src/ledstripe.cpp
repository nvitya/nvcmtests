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
 *  file:     ledstripe.cpp
 *  brief:    APA 102 led stripe handler
 *  version:  1.00
 *  date:     2018-12-16
 *  authors:  nvitya
 *  note:
 *    Required resources: SPI + DMA
*/

#include <ledstripe.h>

void TLedStripe::Init(uint32_t aledcount)
{
	ledcount = aledcount;

	dmabuf[0] = 0;  // start word

	uint32_t n;
	for (n = 0; n < ledcount; ++n)
	{
		dmabuf[n + 1] = 0;
	}

	dmabuf[ledcount + 2] = 0xFFFFFFFF;  // stop frame

	spi.DmaAssign(true, &dmach);
}

void TLedStripe::StartDisplay()
{
	if (dmach.Active())
	{
		return;
	}

	dmaxfer.bytewidth = 1;
	dmaxfer.srcaddr = &dmabuf[0];
	dmaxfer.count = 4 * ledcount + 8;
	dmaxfer.flags = 0;

	spi.DmaStartSend(&dmaxfer);
}
