/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_draw2dq.h
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

enum control_draw2dq_state
{
   CONTROL_DRAW2DQ_STATE_HOVER,
   CONTROL_DRAW2DQ_STATE_MOVEIN,
   CONTROL_DRAW2DQ_STATE_PRESSURE,
   CONTROL_DRAW2DQ_STATE_MOVEOUT
};

struct control_draw2dq
{
   /* Include the base */
   struct bt_control base;
   
   /* Our current mode */
   int is_holding;
   
   struct bt_kinematics * kin;
   struct bt_dynamics * dyn;
   
   /* Views into the x,y and vx,vy parts of the position
    * and reference vectors in the base */
   gsl_vector * base_pos_xy;
   gsl_vector * base_pos_vxvy;
   gsl_vector * base_ref_xy;
   gsl_vector * base_ref_vxvy;
   
   /* The local 7d versions of the vectors
    * (like the cartesian_xyz_q controller) */
   gsl_vector * position;
   gsl_vector * reference;
   
   /* These are views into the xyz part and the quaternion part
    * of the above 7d vectors */
   gsl_vector * pos_xy;
   gsl_vector * pos_xyz;
   gsl_vector * pos_quat;
   gsl_vector * ref_xy;
   gsl_vector * ref_xyz;
   gsl_vector * ref_quat;
   
   /* This is how the page is defined */
   gsl_vector * p_topleft;
   gsl_vector * p_topright;
   gsl_vector * p_onpage;
   
   /* We keep a rotation matrix from page coords to word coords
    * This is re-calced whenever p_topleft, p_topright, or p_onpage are chagned */
   gsl_matrix * page_to_world;
   
   /* The state */
   enum control_draw2dq_state state;
   int rot_align_done;
   
   /* For cartesian position control in page-space */
   gsl_vector * Kp;
   gsl_vector * Ki;
   gsl_vector * Kd;
   gsl_vector * integrator;
   gsl_vector * temp1;
   gsl_vector * temp2;
   
   double rot_p;
   double rot_d;
   
   /* A temporary vector used in the rotation matrix setting code */
   gsl_vector * temp_set;
   
   /* For get_position() rotation */
   gsl_matrix * tool_to_page;
   gsl_vector * tool_to_page_y;
   gsl_vector * tool_to_page_z;
   
   /* For computing reference rotation */
   gsl_matrix * ref_tool_to_page;
   gsl_vector * ref_tool_to_page_x;
   gsl_vector * ref_tool_to_page_x_xy;
   gsl_vector * ref_tool_to_page_y;
   /* The local tool z reference vector,
    * the rotation equivalent to the z xyz dimension;
    * expressed in page frame, normally (0,0,1)  */
   gsl_vector * ref_tool_to_page_z;
   
   gsl_vector * temp4vec;
   
   /* This is for us to keep track of during real-time evals */
   int last_time_saved;
   double last_time;
   
   gsl_vector * page_force;
   gsl_vector * world_force;
   
   gsl_vector * world_torque;
   
   /* Sometimes, in the z direction, we apply pressure;
    * This gives the pressure magnitude */
   double pressure;
   double hover_distance;
};

/* The controller-specific create/destroy functions */
struct control_draw2dq * control_draw2dq_create(config_setting_t * config,
   struct bt_kinematics * kin, struct bt_dynamics * dyn);
void control_draw2dq_destroy(struct control_draw2dq * c);

int control_draw2dq_set_topleft(struct control_draw2dq * c);
int control_draw2dq_set_topright(struct control_draw2dq * c);
int control_draw2dq_set_onpage(struct control_draw2dq * c);
int control_draw2dq_set_pressure(struct control_draw2dq * c, double pressure);
int control_draw2dq_set_hover_distance(struct control_draw2dq * c, double distance);

int control_draw2dq_press(struct control_draw2dq * c);
int control_draw2dq_hover(struct control_draw2dq * c);
