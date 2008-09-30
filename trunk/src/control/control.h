/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Sam Clanton
 *                       Christopher Dellin
 *  Creation Date ...... Nov 24, 2002
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2002 Nov 24 - TH
 *      File created.
 *    2004 Dec 16 - BZ, SC, TH
 *      Initial port to linux + RTAI
 *    2008 Sept 16 - CD
 *      Ported from btsystem to libbt; btstatecontrol and btcontrol merged
 *
 * ======================================================================== */

/* A controller represents a "space",
 * and can idle/hold a reference position,
 * return the current position,
 * and outputs a joint torque vector */

#ifndef BT_CONTROL_H
#define BT_CONTROL_H

#include <gsl/gsl_vector.h>

/* "Base Class" function pointers */
struct bt_control
{
   const char * name; /* points to the same place for a given type */
   int n; /* number of dimensions to be controlled */
   
   /* Simple state switching */
   int (*idle)(struct bt_control * base);
   int (*hold)(struct bt_control * base);
   int (*is_holding)(struct bt_control * base);
   
   /* Note - these getting/setting functions should be mutex safe! */
   int (*get_position)(struct bt_control * base, gsl_vector * position);
   /* Eventually a get_velocity? */
   int (*set_reference)(struct bt_control * base, gsl_vector * reference);
   
   /* RT - Evaluate, put resulting torque in torque */
   int (*eval)(struct bt_control * base, gsl_vector * jtorque, double time);
};

#endif /* BT_CONTROL_H */
