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

#include <libconfig.h>

/* Woo basic independent-PID joint controller! */
struct bt_control_joint
{
   /* Include the base function pointers */
   struct bt_control base;
   
   /* Our current mode */
   int is_holding;
   
   /* Saved pointers to external vectors */
   gsl_vector * jposition;
   gsl_vector * jvelocity;
   
   /* We must maintain places for asynchronous communication;
    * these can be in any format we want */
   /* Note: we don't need position, as we already have jposition to copy from! */
   /*gsl_vector * position;*/ /* Updated on get_position() */
   gsl_vector * reference; /* Saved on set_reference() */
   
   /* Owned by me: */
   gsl_vector * Kp;
   gsl_vector * Ki;
   gsl_vector * Kd;
   gsl_vector * integrator;
   gsl_vector * temp1;
   gsl_vector * temp2;
   
   int last_time_saved;
   double last_time;
   
};

struct bt_control_joint * bt_control_joint_create(config_setting_t * config, gsl_vector * jposition, gsl_vector * jvelocity);
void bt_control_joint_destroy(struct bt_control_joint *);
