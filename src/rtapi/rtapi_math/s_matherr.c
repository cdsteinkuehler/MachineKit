/* @(#)s_matherr.c 5.1 93/09/24 */
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice 
 * is preserved.
 * ====================================================
 */

#if defined(LIBM_SCCS) && !defined(lint)
static char rcsid[] = "$NetBSD: s_matherr.c,v 1.6 1995/05/10 20:47:53 jtc Exp $";
#endif

#include "rtapi_math.h"
#include "mathP.h"

#ifdef __STDC__
	int rtapi_matherr(struct exception *x)
#else
	int rtapi_matherr(x)
	struct exception *x;
#endif
{
	int n=0;
	if(x->arg1!=x->arg1) return 0;
	return n;
}
