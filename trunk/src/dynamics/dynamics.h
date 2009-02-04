/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... dynamics.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... Feb 3, 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

#ifndef BT_DYNAMICS_H
#define BT_DYNAMICS_H

#include <libconfig.h>

/* bt_dynamics uses gsl :-) */
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "kinematics.h"

/* Links are things with attached frames;
 * Link kinematics, we have a base link/frame,
 * and one for each moving link.
 * For now, dynamics does not have a toolplate frame.
 * We'll have to work out the best way to support tooling
 * dynamics later. */
struct bt_dynamics_link {

   /* Doubly-linked for convenience */
   struct bt_dynamics_link * next;
   struct bt_dynamics_link * prev;

   /* Mass of link (kg) */
   double mass;
   
   /* Center of mass of link, in DH link frame */
   gsl_vector * com;
   
   /* Inertia matrix of link, around link center-of-mass */
   gsl_matrix * I;
   
   /* NOTE:
    * Do we need rotor inertia here?? */
   
   /* Next, a place to hold the results from calculations */
   
   /* Vectors expressed in local link frame */
   gsl_vector * omega; /* angular velocity of local frame w.r.t. base frame */
   gsl_vector * alpha; /* angular acceleration of local frame w.r.t. base frame */
   gsl_vector * a;     /* linear acceleration of frame origin */
   
   /* A couple of caches, also expressed in local link frame */
   gsl_vector * omega_prev; /* Previous frame's ang vel in my frame */
   gsl_vector * f_next;     /* Next frame's force in my frame */
   
   gsl_vector * fnet;
   gsl_vector * tnet;
   gsl_vector * f;     /* force exerted on this link by previous link */
   gsl_vector * t;     /* torque exerted on this link by previous link */
};


struct bt_dynamics {

   /* We rely on the kin structure */
   struct bt_kinematics * kin;
   
   int dof;
   int nlinks;
   struct bt_dynamics_link ** link_array;
   
   struct bt_dynamics_link * base;
   struct bt_dynamics_link ** link; /* Moving links array */
   
   /* Temporary Vectors */
   gsl_vector * temp1_v3;
   gsl_vector * temp2_v3;
};


/* Dynamics Functions */
struct bt_dynamics * bt_dynamics_create( config_setting_t * dynconfig, int ndofs, struct bt_kinematics * kin );
int bt_dynamics_destroy( struct bt_dynamics * dyn );

/* Reverse Newton-Euler Algorithm (RNEA)
 * Throw a switch in there for gravity on/off?
 * How to account for base acceleration (even gravity?)
 * NOTE: This takes ~ 152us on PC104 right now. */
int bt_dynamics_eval_inverse( struct bt_dynamics * dyn,
   gsl_vector * jpos, gsl_vector * jvel, gsl_vector * jacc, gsl_vector * jtor );

#endif /* BT_DYNAMICS_H */
