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
 * and output a joint torque vector */

#ifndef BT_CONTROL_H
#define BT_CONTROL_H

#include <gsl/gsl_vector.h>

struct bt_control;

/* "Base Class" function pointers */
struct bt_control_type
{
   char name[20]; /* points to the same place for a given type */
   
   /* Simple state switching */
   int (*idle)(struct bt_control * c);
   int (*hold)(struct bt_control * c);
   int (*is_holding)(struct bt_control * c);
   
   /* Note - these getting/setting functions should be mutex safe! */
   int (*get_position)(struct bt_control * c, gsl_vector * position);
   /* Eventually a get_velocity? */
   int (*set_reference)(struct bt_control * c, gsl_vector * reference);
   
   /* RT - Evaluate, put resulting torque in torque */
   int (*eval)(struct bt_control * c, gsl_vector * jtorque, double time);
};

struct bt_control {
   struct bt_control_type * type;
   int n; /* number of dimensions to be controlled */
};

#endif /* BT_CONTROL_H */
