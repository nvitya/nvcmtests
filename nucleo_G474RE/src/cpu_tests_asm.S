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
 *  file:     cpu_tests_asm.S (uart)
 *  brief:    MCU linear code speed tests
 *  version:  1.00
 *  date:     2018-02-10
 *  authors:  nvitya
*/

.syntax unified
.cpu cortex-m4

//-------------------------------------------------------------------------------------------------
.section .text,"ax",%progbits
.global  linear_run_asm
.thumb_func
linear_run_asm:
	ldr 	 	r3, =0xE0001004		// DWT_CYCCNT address
	ldr         r2, [r3]

	// r2, r3 must kept !

	// test 64 simple instructions

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 16

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 32

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 48

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 64

	// end test

	ldr         r1, [r3]
	sub			r0, r1, r2

	bx		 	lr

//-------------------------------------------------------------------------------------------------
.section .text_ITCRAM,"ax",%progbits
.global  linear_run_asm_itcram
.thumb_func
linear_run_asm_itcram:
	ldr 	 	r3, =0xE0001004		// DWT_CYCCNT address
	ldr         r2, [r3]

	// r2, r3 must kept !

	// test 64 simple instructions

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 16

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 32

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 48

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 64

	// end test

	ldr         r1, [r3]
	sub			r0, r1, r2

	bx		 	lr

//-------------------------------------------------------------------------------------------------
.section .text_DTCRAM,"ax",%progbits
.global  linear_run_asm_dtcram
.thumb_func
linear_run_asm_dtcram:
	ldr 	 	r3, =0xE0001004		// DWT_CYCCNT address
	ldr         r2, [r3]

	// r2, r3 must kept !

	// test 64 simple instructions

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 16

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 32

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 48

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 64

	// end test

	ldr         r1, [r3]
	sub			r0, r1, r2

	bx		 	lr


//-------------------------------------------------------------------------------------------------
.section .text_RAM2,"ax",%progbits
.global  linear_run_asm_ram2
.thumb_func
linear_run_asm_ram2:
	ldr 	 	r3, =0xE0001004		// DWT_CYCCNT address
	ldr         r2, [r3]

	// r2, r3 must kept !

	// test 64 simple instructions

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 16

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 32

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 48

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1
	add			r0, #1

	// 64

	// end test

	ldr         r1, [r3]
	sub			r0, r1, r2

	bx		 	lr

