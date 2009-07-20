/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... interp.h
 *  Author ............. Christopher Dellin
 *  Creation Date ...... Sept 15, 2008
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

#ifndef BT_INTERP_H
#define BT_INTERP_H

#include <gsl/gsl_interp.h>

/* interp uses the GSL interpolation framework,
 * and relies on many GSL interpolation cspline functions. */
 
enum bt_interp_type {
   BT_INTERP_NATURAL,
   BT_INTERP_SLOPE
};

const gsl_interp_type * bt_interp;

/* Public funtions */

int bt_interp_set_type( gsl_interp * interp,
   enum bt_interp_type ltype, enum bt_interp_type rtype );

int bt_interp_set_slopes( gsl_interp * interp,
   double lslope, double rslope );

#endif /* BT_INTERP_H */
