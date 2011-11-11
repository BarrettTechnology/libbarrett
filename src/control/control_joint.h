/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_joint.h
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

#include "control.h"

#include "dynamics.h"

#include <libconfig.h>

/* This controller controls on joint position by outputing a joint
 * acceleration through the standard Barrett RNEA implementation. */

struct bt_control_joint
{
   /* Include the base */
   struct bt_control base;
   
   /* Our current mode */
   int is_holding;
   
   struct bt_dynamics * dyn;
   
   /* Saved pointers to external vectors we need */
   gsl_vector * jposition;
   gsl_vector * jvelocity;
   
   /* To be computed as intermediate control output */
   gsl_vector * jacceleration;
   
   /* Owned by me, each an n-vector */
   gsl_vector * Kp;
   gsl_vector * Ki;
   gsl_vector * Kd;
   gsl_vector * integrator;
   gsl_vector * temp1;
   gsl_vector * temp2;
   
   /* This is for us to keep track of during real-time evals */
   int last_time_saved;
   double last_time;
   
};

/* The controller-specific create/destroy functions */
int bt_control_joint_create(struct bt_control_joint ** conptr,
                            config_setting_t * config,
                            struct bt_dynamics * dyn,
                            gsl_vector * jposition, gsl_vector * jvelocity);
void bt_control_joint_destroy(struct bt_control_joint * c);
