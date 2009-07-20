/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_draw2d.h
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

enum control_draw2d_state
{
   CONTROL_DRAW2D_STATE_HOVER,
   CONTROL_DRAW2D_STATE_MOVEIN,
   CONTROL_DRAW2D_STATE_PRESSURE,
   CONTROL_DRAW2D_STATE_MOVEOUT
};

struct control_draw2d
{
   /* Include the base */
   struct bt_control base;
   
   /* Our current mode */
   int is_holding;
   
   struct bt_kinematics * kin;
   struct bt_dynamics * dyn;
   
   /* The local 3d versions of these vectors */
   gsl_vector * position;
   gsl_vector * reference;
   
   
   /* This is how the page is defined */
   gsl_vector * p_topleft;
   gsl_vector * p_topright;
   gsl_vector * p_onpage;
   
   /* We keep a rotation matrix from page coords to word coords
    * This is re-calced whenever p_topleft, p_topright, or p_onpage are chagned */
   gsl_matrix * page_to_world;
   
   /* The state */
   enum control_draw2d_state state;
   
   /* For cartesian position control in page-space */
   gsl_vector * Kp;
   gsl_vector * Ki;
   gsl_vector * Kd;
   gsl_vector * integrator;
   gsl_vector * temp1;
   gsl_vector * temp2;
   
   gsl_vector * temp_set;
   
   /* This is for us to keep track of during real-time evals */
   int last_time_saved;
   double last_time;
   
   gsl_vector * page_force;
   gsl_vector * world_force;
   
   /* Sometimes, in the z direction, we apply pressure;
    * This gives the pressure magnitude */
   double pressure;
   double hover_distance;
};

/* The controller-specific create/destroy functions */
struct control_draw2d * control_draw2d_create(config_setting_t * config,
   struct bt_kinematics * kin, struct bt_dynamics * dyn);
void control_draw2d_destroy(struct control_draw2d * c);

int control_draw2d_set_topleft(struct control_draw2d * c);
int control_draw2d_set_topright(struct control_draw2d * c);
int control_draw2d_set_onpage(struct control_draw2d * c);
int control_draw2d_set_pressure(struct control_draw2d * c, double pressure);
int control_draw2d_set_hover_distance(struct control_draw2d * c, double distance);

int control_draw2d_press(struct control_draw2d * c);
int control_draw2d_hover(struct control_draw2d * c);
