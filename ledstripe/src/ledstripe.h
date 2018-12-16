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
 *  file:     ledstripe.h
 *  brief:    APA 102 led stripe handler
 *  version:  1.00
 *  date:     2018-12-16
 *  authors:  nvitya
 *  note:
 *    Required resources: SPI + DMA
*/

#ifndef SRC_LEDSTRIPE_H_
#define SRC_LEDSTRIPE_H_

#include "platform.h"
#include "hwspi.h"
#include "hwdma.h"

#define LEDSTRIPE_MAX_LEDS   150

class TLedStripe
{
public:
  uint32_t       updatecount = 0;

  THwSpi         spi;
  THwDmaChannel  dmach;
  THwDmaTransfer dmaxfer;

  uint32_t       ledcount = 1;

  int            state = 0;

  uint32_t       dmabuf[LEDSTRIPE_MAX_LEDS + 2];

public:
	void           Init(uint32_t aledcount);
	inline void    SetLed(uint16_t aled, uint32_t adata) { dmabuf[aled + 1] = adata; }
	inline void    SetIRGB(uint16_t aled, uint8_t intensity, uint8_t r, uint8_t g, uint8_t b)
	{
		dmabuf[aled + 1] = ((intensity >> 3) | 0xE0) | (b << 8) | (g << 16) | (r << 24);
	}

	void           StartDisplay();
};

#endif /* SRC_LEDSTRIPE_H_ */
