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

#ifndef BT_GRAVITY_H
#define BT_GRAVITY_H

#include "kinematics.h"

/* btgravity uses gsl :-) */
#include <gsl/gsl_vector.h>

/* gravity compensation
 * only needs a couple of additional
 * parameters in each link*/
struct bt_gravity {

   /* We need to know about each link's kinematics */
   struct bt_kinematics * kin;
   
   gsl_vector * world_g;
   
   gsl_vector ** g; /* Gravity vector in my frame */
   gsl_vector ** mu;
   gsl_vector ** t; /* my torque vector in my frame */
   gsl_vector ** pt; /* my torque vector in previous frame */
   
};

struct bt_gravity * bt_gravity_create( config_setting_t * gravconfig, struct bt_kinematics * kin );
int bt_gravity_destroy( struct bt_gravity * grav );

int bt_gravity_eval( struct bt_gravity * grav, gsl_vector * jtorque );

#endif /* BT_GRAVITY_H */
