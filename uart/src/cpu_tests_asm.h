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
 *  file:     cpu_tests_asm.h (uart)
 *  brief:    MCU linear code speed tests
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

#ifndef SRC_CPU_TESTS_ASM_H_
#define SRC_CPU_TESTS_ASM_H_

#include "platform.h"

extern "C"
{

#if __CORTEX_M >= 3

  unsigned linear_run_asm();
  unsigned linear_run_asm_itcram();
  unsigned linear_run_asm_dtcram();
  unsigned linear_run_asm_ram2();

#endif

  unsigned linear_run_asm_m0(volatile uint32_t * clockcounter);
  unsigned linear_run_asm_ram_m0(volatile uint32_t * clockcounter);

}

#endif /* SRC_CPU_TESTS_ASM_H_ */
