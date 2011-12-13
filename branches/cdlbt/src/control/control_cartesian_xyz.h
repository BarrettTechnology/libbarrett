/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_cartesian_xyz.h
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

/* This controller controls on Cartesian translational position
 * (X, Y, and Z axes), ignoring rotation.  It uses the RNEA and
 * applies a force a the Cartesian end-tip. */

struct bt_control_cartesian_xyz
{
   /* Include the base */
   struct bt_control base;
   
   /* Our current mode */
   int is_holding;
   
   struct bt_kinematics * kin;
   struct bt_dynamics * dyn;
   
   /* The position is in base.position */
   gsl_vector * cvelocity;
   
   /* Saved pointers to external vectors we need
    (for evaulating RNEA) */
   /*gsl_vector * jposition;
   gsl_vector * jvelocity;*/
   
   /* The linear tool jacobian */
   /*gsl_matrix * tool_jacobian_linear;*/
   
   /* To be computed as intermediate control output */
   gsl_vector * force;
   
   /* Owned by me, each a 3-vector */
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
int bt_control_cartesian_xyz_create(
                           struct bt_control_cartesian_xyz ** conptr,
                           config_setting_t * config,
                           struct bt_kinematics * kin,
                           struct bt_dynamics * dyn);
void bt_control_cartesian_xyz_destroy(struct bt_control_cartesian_xyz * c);