/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_draw.h
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

#include <libbarrett/control.h>

#include <libbarrett/dynamics.h>

#include <libconfig.h>

/* This controller controls on Cartesian translational position
 * (X, Y, and Z axes), ignoring rotation.  It uses the RNEA and
 * applies a force a the Cartesian end-tip. */

struct control_draw
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
   
   gsl_vector * pressure;
   int pressure_onoff;
   
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
struct control_draw * control_draw_create(config_setting_t * config,
   struct bt_kinematics * kin, struct bt_dynamics * dyn);
void control_draw_destroy(struct control_draw * c);

int control_draw_set_pressure_vec(struct control_draw * c, gsl_vector * vec);
