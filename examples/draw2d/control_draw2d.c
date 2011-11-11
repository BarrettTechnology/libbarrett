/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... control_draw2d.c
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
#include "control_draw2d.h"

#include <libbarrett/dynamics.h>
#include <libbarrett/gsl.h>

#include <libconfig.h>
#include <syslog.h>
#include <gsl/gsl_blas.h>

/* This is just so we know what to look for in the config file */
static char *str_dimension[] = {"x","y","z"};

/* Define the type */
static int idle(struct bt_control * base);
static int hold(struct bt_control * base);
static int is_holding(struct bt_control * base);
static int get_position(struct bt_control * base);
static int eval(struct bt_control * base, gsl_vector * jtorque, double time);
static const struct bt_control_type control_draw2d_type = {
   "draw2d-space",
   &idle,
   &hold,
   &is_holding,
   &get_position,
   &eval
};
const struct bt_control_type * control_draw2d = &control_draw2d_type;

/* Local static functions */
static int set_transform_matrix(struct control_draw2d * c);

/* Controller-specific functions */
struct control_draw2d * control_draw2d_create(config_setting_t * config,
   struct bt_kinematics * kin, struct bt_dynamics * dyn)
{
   int err;
   struct control_draw2d * c;
   
   /* Create */
   c = (struct control_draw2d *) malloc( sizeof(struct control_draw2d) );
   if (!c)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Initialize */
   c->base.type = control_draw2d;
   c->base.n = 2;
   c->base.position = 0;
   c->base.reference = 0;
   c->is_holding = 0;
   c->kin = kin;
   c->dyn = dyn;
   c->position = 0;
   c->reference = 0;
   c->p_topleft = 0;
   c->p_topright = 0;
   c->p_onpage = 0;
   c->page_to_world = 0;
   c->state = CONTROL_DRAW2D_STATE_HOVER;
   c->Kp = 0;
   c->Ki = 0;
   c->Kd = 0;
   c->integrator = 0;
   c->temp1 = 0;
   c->temp2 = 0;
   c->last_time_saved = 0;
   c->page_force = 0;
   c->world_force = 0;
   c->pressure = 8.0; /* N */
   c->hover_distance = 0.05; /* m */
   
   /* Allocate */
   if (   !(c->base.position  = (gsl_vector *) malloc(sizeof(gsl_vector))  )
       || !(c->base.reference = (gsl_vector *) malloc(sizeof(gsl_vector))  )
       || !(c->position       = gsl_vector_calloc(3)  )
       || !(c->reference      = gsl_vector_calloc(3)  )
       || !(c->p_topleft      = gsl_vector_calloc(3)  )
       || !(c->p_topright     = gsl_vector_calloc(3)  )
       || !(c->p_onpage       = gsl_vector_calloc(3)  )
       || !(c->page_to_world  = gsl_matrix_calloc(3,3))
       || !(c->Kp             = gsl_vector_calloc(3)  )
       || !(c->Ki             = gsl_vector_calloc(3)  )
       || !(c->Kd             = gsl_vector_calloc(3)  )
       || !(c->integrator     = gsl_vector_calloc(3)  )
       || !(c->temp1          = gsl_vector_calloc(3)  )
       || !(c->temp2          = gsl_vector_calloc(3)  )
       || !(c->temp_set       = gsl_vector_calloc(3)  )
       || !(c->page_force     = gsl_vector_calloc(3)  )
       || !(c->world_force    = gsl_vector_calloc(3)  ))
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      control_draw2d_destroy(c);
      return 0;
   }
   
   /* Set up vector views */
   {
      gsl_vector_view view;
      view = gsl_vector_subvector( c->position, 0, 2 );
      *(c->base.position) = view.vector;
      view = gsl_vector_subvector( c->reference, 0, 2 );
      *(c->base.reference) = view.vector;
   }
   
   /* Set default page location
    * (the page defaults in the YZ plane, directly above the WAM) */
   
   gsl_vector_set(c->p_topleft,0,  0.00 );
   gsl_vector_set(c->p_topleft,1, +0.10 );
   gsl_vector_set(c->p_topleft,2,  0.75 );
   
   gsl_vector_set(c->p_topright,0,  0.00 );
   gsl_vector_set(c->p_topright,1, -0.10 );
   gsl_vector_set(c->p_topright,2,  0.75 );
   
   gsl_vector_set(c->p_onpage,0,  0.00 );
   gsl_vector_set(c->p_onpage,1,  0.00 );
   gsl_vector_set(c->p_onpage,2,  0.50 );
   
   /* Set the rotation matrix based on the above defaults */
   err = set_transform_matrix(c);
   if (err)
   {
      syslog(LOG_ERR,"%s: Bad default page location.",__func__);
      control_draw2d_destroy(c);
      return 0;
   }
   
   /* Read in the PID values from the configuration */
   {
      int j;
      config_setting_t * pids;
      /* Make sure the configuration looks good */
      if ( !(pids = config_setting_get_member( config, "pids" ))
                || (config_setting_type(pids)   != CONFIG_TYPE_GROUP) 
                || (config_setting_length(pids) != 3) )
      {
         syslog(LOG_ERR,"%s: The 'pids' configuration is not a 3-element group.",__func__);
         control_draw2d_destroy(c);
         return 0;
      }
      /* Read in the PID values */
      for (j=0; j<3; j++)
      {
         double p,i,d;
         config_setting_t * pid_grp;
         
         pid_grp = config_setting_get_member( pids, str_dimension[j] );
         if (!pid_grp)
         {
            syslog(LOG_ERR,"%s: No member of pids called %s.",__func__,str_dimension[j]);
            control_draw2d_destroy(c);
            return 0;
         }
         if (   bt_gsl_config_get_double(config_setting_get_member( pid_grp, "p" ), &p)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "i" ), &i)
             || bt_gsl_config_get_double(config_setting_get_member( pid_grp, "d" ), &d))
         {
            syslog(LOG_ERR,"%s: No p, i, and/or d value",__func__);
            control_draw2d_destroy(c);
            return 0;
         }
         
         gsl_vector_set(c->Kp,j,p);
         gsl_vector_set(c->Ki,j,i);
         gsl_vector_set(c->Kd,j,d);
      }
   }
   
   return c;
}

void control_draw2d_destroy(struct control_draw2d * c)
{
   /* De-allocate */
   if (c->base.position)  free(c->base.position);
   if (c->base.reference) free(c->base.reference);
   if (c->position)       gsl_vector_free(c->base.position);
   if (c->reference)      gsl_vector_free(c->base.reference);
   if (c->p_topleft)      gsl_vector_free(c->p_topleft);
   if (c->p_topright)     gsl_vector_free(c->p_topright);
   if (c->p_onpage)       gsl_vector_free(c->p_onpage);
   if (c->page_to_world)  gsl_matrix_free(c->page_to_world);
   if (c->Kp)             gsl_vector_free(c->Kp);
   if (c->Ki)             gsl_vector_free(c->Ki);
   if (c->Kd)             gsl_vector_free(c->Kd);
   if (c->integrator)     gsl_vector_free(c->integrator);
   if (c->temp1)          gsl_vector_free(c->temp1);
   if (c->temp2)          gsl_vector_free(c->temp2);
   if (c->temp_set)       gsl_vector_free(c->temp_set);
   if (c->page_force)     gsl_vector_free(c->page_force);
   if (c->world_force)    gsl_vector_free(c->world_force);
   
   /* Destroy */
   free(c);
   return;
}

static int set_transform_matrix(struct control_draw2d * c)
{
   gsl_vector_view x, y, z;
   double len;
   
   /* Make vector views of the page_to_world rotation matrix */
   x = gsl_matrix_column( c->page_to_world, 0);
   y = gsl_matrix_column( c->page_to_world, 1);
   z = gsl_matrix_column( c->page_to_world, 2);
   
   /* x is the unit vector (p_topright - p_topleft) */
   gsl_blas_dcopy(c->p_topright, &x.vector);
   gsl_blas_daxpy(-1, c->p_topleft, &x.vector);
   len = gsl_blas_dnrm2( &x.vector );
   gsl_blas_dscal( 1.0 / len, &x.vector );
   
   /* z is the unit vector (x crossed into (p_onpage - p_topleft)) */
   gsl_blas_dcopy(c->p_onpage, c->temp_set);
   gsl_blas_daxpy(-1, c->p_topleft, c->temp_set);
   bt_gsl_cross( &x.vector, c->temp_set, &z.vector );
   len = gsl_blas_dnrm2( &z.vector );
   gsl_blas_dscal( 1.0 / len, &z.vector );
   
   /* y is (z crossed into x) */
   gsl_vector_set_zero(&y.vector);
   bt_gsl_cross( &z.vector, &x.vector, &y.vector );
   
   return 0;
}

int control_draw2d_set_topleft(struct control_draw2d * c)
{
   gsl_blas_dcopy( c->kin->tool->origin_pos, c->p_topleft );
   return 0;
}

int control_draw2d_set_topright(struct control_draw2d * c)
{
   gsl_blas_dcopy( c->kin->tool->origin_pos, c->p_topright );
   return 0;
}

int control_draw2d_set_onpage(struct control_draw2d * c)
{
   gsl_blas_dcopy( c->kin->tool->origin_pos, c->p_onpage );
   return 0;
}

int control_draw2d_set_pressure(struct control_draw2d * c, double pressure)
{
   c->pressure = pressure;
   return 0;
}

int control_draw2d_set_hover_distance(struct control_draw2d * c, double distance)
{
   c->hover_distance = distance;
   return 0;
}

int control_draw2d_press(struct control_draw2d * c)
{
   switch (c->state)
   {
      case CONTROL_DRAW2D_STATE_PRESSURE:
         break;
      case CONTROL_DRAW2D_STATE_HOVER:
      case CONTROL_DRAW2D_STATE_MOVEIN:
      case CONTROL_DRAW2D_STATE_MOVEOUT:
         c->state = CONTROL_DRAW2D_STATE_MOVEIN;
         break;
   }
   return 0;
}

int control_draw2d_hover(struct control_draw2d * c)
{
   switch (c->state)
   {
      case CONTROL_DRAW2D_STATE_HOVER:
         break;
      case CONTROL_DRAW2D_STATE_PRESSURE:
      case CONTROL_DRAW2D_STATE_MOVEIN:
      case CONTROL_DRAW2D_STATE_MOVEOUT:
         gsl_vector_set(c->integrator,2,0.0);
         gsl_vector_set(c->reference,2, gsl_vector_get(c->position,2) );
         c->state = CONTROL_DRAW2D_STATE_MOVEOUT;
         break;
   }
   return 0;
}




static int idle(struct bt_control * base)
{
   struct control_draw2d * c = (struct control_draw2d *) base;
   /* Do we need to stop doing anything? */
   c->is_holding = 0;
   return 0;
}

static int hold(struct bt_control * base)
{
   struct control_draw2d * c = (struct control_draw2d *) base;
   
   /* Already holding? */
   if (c->is_holding)
      return 1;
   
   /* Convert current world cartesian position to our page reference */
   gsl_blas_dgemv( CblasTrans, -1.0, c->page_to_world,
                   c->p_topleft,
                   0.0, c->reference );
   gsl_blas_dgemv( CblasTrans, 1.0, c->page_to_world,
                   c->kin->tool->origin_pos,
                   1.0, c->reference );
   
   /* Zero the integrators! */
   gsl_vector_set_zero(c->integrator);
   
   /* We're now in hover mode */
   c->state = CONTROL_DRAW2D_STATE_HOVER;
   c->last_time_saved = 0;
   c->is_holding = 1;
   return 0;
}

static int is_holding(struct bt_control * base)
{
   struct control_draw2d * c = (struct control_draw2d *) base;
   return c->is_holding ? 1 : 0;
}

static int get_position(struct bt_control * base)
{
   struct control_draw2d * c = (struct control_draw2d *) base;
   gsl_blas_dgemv( CblasTrans, -1.0, c->page_to_world,
                   c->p_topleft,
                   0.0, c->position );
   gsl_blas_dgemv( CblasTrans, 1.0, c->page_to_world,
                   c->kin->tool->origin_pos,
                   1.0, c->position );
   return 0;
}

/* RT - Evaluate */
static int eval(struct bt_control * base, gsl_vector * jtorque, double time)
{
   struct control_draw2d * c = (struct control_draw2d *) base;
   
   /* Do PID position control with the current reference */
   if (c->is_holding)
   {
      if (!c->last_time_saved)
      {
         c->last_time = time;
         c->last_time_saved = 1;
      }
      
      /* Simple reference control of z direction (i own)
       * for moves */
      switch (c->state)
      {
         case CONTROL_DRAW2D_STATE_HOVER:
         case CONTROL_DRAW2D_STATE_PRESSURE:
            break;
         case CONTROL_DRAW2D_STATE_MOVEIN:
            *(gsl_vector_ptr(c->reference,2)) += 0.1 * (time - c->last_time); /* m/s */
            break;
         case CONTROL_DRAW2D_STATE_MOVEOUT:
            *(gsl_vector_ptr(c->reference,2)) -= 0.1 * (time - c->last_time); /* m/s */
            if (gsl_vector_get(c->reference,2) < - c->hover_distance)
            {
               gsl_vector_set(c->reference,2, - c->hover_distance );
               c->state = CONTROL_DRAW2D_STATE_HOVER;
            }
            break;
      }
      
      /* Get current position
       * (this is guarenteed to happen in the wam loop) */
      
      /* Compute page force */
      gsl_vector_set_zero(c->page_force);
      /* Compute the error */
      gsl_vector_memcpy( c->temp1, c->position );
      gsl_vector_sub( c->temp1, c->reference );
      /* Increment integrator */
      gsl_vector_memcpy( c->temp2, c->temp1 );
      gsl_vector_scale( c->temp2, time - c->last_time );
      c->last_time = time;
      gsl_vector_add( c->integrator, c->temp2 );
      /* Copy in P term */
      gsl_vector_memcpy( c->temp2, c->temp1 );
      gsl_vector_mul( c->temp2, c->Kp );
      gsl_vector_sub( c->page_force, c->temp2 );
      /* Copy in I term */
      gsl_vector_memcpy( c->temp2, c->integrator);
      gsl_vector_mul( c->temp2, c->Ki );
      gsl_vector_sub( c->page_force, c->temp2 );
      /* Copy in D term */
      gsl_vector_memcpy( c->temp2, c->kin->tool_velocity );
      gsl_vector_mul( c->temp2, c->Kd );
      gsl_vector_sub( c->page_force, c->temp2 );
      
      /* Switch from STATE_MOVEIN to STATE_HOVER if necessary */
      switch (c->state)
      {
         case CONTROL_DRAW2D_STATE_HOVER:
         case CONTROL_DRAW2D_STATE_PRESSURE:
         case CONTROL_DRAW2D_STATE_MOVEOUT:
            break;
         case CONTROL_DRAW2D_STATE_MOVEIN:
            if (gsl_vector_get(c->page_force,2) > 2 * c->pressure)
               c->state = CONTROL_DRAW2D_STATE_PRESSURE;
      }
      
      /* If we're in pressure mode, turn the z force to c->pressure */
      if (c->state == CONTROL_DRAW2D_STATE_PRESSURE)
         gsl_vector_set(c->page_force,2, c->pressure);
      
      /* Convert page force to world force */
      gsl_blas_dgemv( CblasNoTrans, 1.0, c->page_to_world,
                      c->page_force,
                      0.0, c->world_force );

      /* Multiply by the Jacobian-transpose at the tool */
      gsl_blas_dgemv( CblasTrans, 1.0, c->kin->tool_jacobian_linear,
                      c->world_force,
                      1.0, jtorque );
   }

   return 0;
}
