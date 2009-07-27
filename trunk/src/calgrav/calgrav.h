/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... gravity.h
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

#ifndef BT_CALGRAV_H
#define BT_CALGRAV_H

#include "kinematics.h"

/* btgravity uses gsl :-) */
#include <gsl/gsl_vector.h>

/* gravity compensation
 * only needs a couple of additional
 * parameters in each link*/
struct bt_calgrav {

   /* We need to know about each link's kinematics */
   struct bt_kinematics * kin;
   
   gsl_vector * world_g;
   
   gsl_vector ** g; /* Gravity vector in my frame */
   gsl_vector ** mu;
   gsl_vector ** t; /* my torque vector in my frame */
   gsl_vector ** pt; /* my torque vector in previous frame */
   
};

struct bt_calgrav * bt_calgrav_create( config_setting_t * gravconfig, struct bt_kinematics * kin );
int bt_calgrav_destroy( struct bt_calgrav * grav );

int bt_calgrav_eval( struct bt_calgrav * grav, gsl_vector * jtorque );

#endif /* BT_CALGRAV_H */
