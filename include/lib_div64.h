/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/*
 * div64.h
 *
 *  Created on: Nov 1, 2013
 *      Author: jxie
 */

#ifndef DIV64_H_
#define DIV64_H_


/*
 * Copyright (C) 2003 Bernardo Innocenti <bernie@develer.com>
 *
 * Based on former lib_do_div() implementation from asm-parisc/div64.h:
 *	Copyright (C) 1999 Hewlett-Packard Co
 *	Copyright (C) 1999 David Mosberger-Tang <davidm@hpl.hp.com>
 *
 *
 * Generic C version of 64bit/32bit division and modulo, with
 * 64bit result and 32bit remainder.
 *
 * The fast case for (n>>32 == 0) is handled inline by lib_do_div().
 *
 * Code generated for this function might be very inefficient
 * for some CPUs. lib_div64_32() can be overridden by linking arch-specific
 * assembly versions such as arch/powerpc/lib/div64.S and arch/sh/lib/div64.S.
 */

/*
 * Copyright (C) 2003 Bernardo Innocenti <bernie@develer.com>
 * Based on former asm-ppc/div64.h and asm-m68knommu/div64.h
 *
 * The semantics of lib_do_div() are:
 *
 * uint32_t lib_do_div(uint64_t *n, uint32_t base)
 * {
 *	uint32_t remainder = *n % base;
 *	*n = *n / base;
 *	return remainder;
 * }
 *
 * NOTE: macro parameter n is evaluated multiple times,
 *       beware of side effects!
 */


uint32_t lib_div64_32(uint64_t *n, uint32_t base)
{
	uint64_t rem = *n;
	uint64_t b = base;
	uint64_t res, d = 1;
	uint32_t high = rem >> 32;

	/* Reduce the thing a bit first */
	res = 0;
	if (high >= base) {
		high /= base;
		res = (uint64_t) high << 32;
		rem -= (uint64_t) (high*base) << 32;
	}

	while ((int64_t)b > 0 && b < rem) {
		b = b+b;
		d = d+d;
	}

	do {
		if (rem >= b) {
			rem -= b;
			res += d;
		}
		b >>= 1;
		d >>= 1;
	} while (d);

	*n = res;
	return rem;
}


/* The unnecessary pointer compare is there
 * to check for type safety (n must be 64bit)
 */
# define lib_do_div(n,base) ({				\
	uint32_t __base = (base);			\
	uint32_t __rem;					\
	(void)(((typeof((n)) *)0) == ((uint64_t *)0));	\
	if (((n) >> 32) == 0) {			\
		__rem = (uint32_t)(n) % __base;		\
		(n) = (uint32_t)(n) / __base;		\
	} else						\
		__rem = lib_div64_32(&(n), __base);	\
	__rem;						\
 })

/* Wrapper for lib_do_div(). Doesn't modify dividend and returns
 * the result, not reminder.
 */
static inline uint64_t lib_lldiv(uint64_t dividend, uint32_t divisor)
{
	uint64_t __res = dividend;
	lib_do_div(__res, divisor);
	return(__res);
}




#endif /* DIV64_H_ */
