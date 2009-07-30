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

/* Bound methods */
#define bt_control_idle(c) ((c)->type->idle(c))
#define bt_control_hold(c) ((c)->type->hold(c))
#define bt_control_is_holding(c) ((c)->type->is_holding(c))
#define bt_control_get_position(c) ((c)->type->get_position(c))
#define bt_control_eval(c,j,t) ((c)->type->eval(c,j,t))

struct bt_control;

/* "Base Class" function pointers */
struct bt_control_type
{
   char name[30]; /* points to the same place for a given type */
   char space[30];
   
   /* Simple state switching */
   int (*idle)(struct bt_control * c);
   int (*hold)(struct bt_control * c);
   int (*is_holding)(struct bt_control * c);
   
   /* Note - these getting/setting functions should be mutex safe! */
   int (*get_position)(struct bt_control * c);
   /* Eventually a get_velocity? */
   
   /* RT - Evaluate, put resulting torque in torque */
   int (*eval)(struct bt_control * c, gsl_vector * jtorque, double time);
};

struct bt_control {
   const struct bt_control_type * type;
   int n; /* number of dimensions to be controlled */
   gsl_vector * position;
   gsl_vector * reference;
};

#endif /* BT_CONTROL_H */
