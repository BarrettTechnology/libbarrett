/** Implementation of bt_interp, GSL interpolator type with support for
 *  2-point trajectories, as well as natural or slope-drive endpoints.
 *
 * \file interp.c
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

#include <gsl/gsl_interp.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_linalg.h>

#include <barrett/cdlbt/interp.h>

/* Implementation notes:
 * OK, here's the way this works.
 * We use a modified GSL cspline algorithm for sizes > 3.
 * For sizes of 2 and 3, all bets are off ... */


struct state_2p
{
   double a, b, c, d;
};

struct state_3p
{
   double aL, bL, cL, dL;
   double aR, bR, cR, dR;
};

struct state_t
{
   enum bt_interp_type ltype;
   enum bt_interp_type rtype;
   
   /* We add two more entries to the state, just for use by the init method */
   double alpha;
   double beta;
   
   /* stuff for systems of 2 and 3 points */
   struct state_2p * s_2p;
   struct state_3p * s_3p;
   
   /* stuff for things with at least 4 points (like csplines) */
   struct {
      double * c;
      double * g;
      double * diag;
      double * offdiag;
   } * cstate;
};


static void * alloc(size_t size)
{
   struct state_t * state;
   
   state = (struct state_t *) malloc( sizeof(struct state_t) );
   
   /* Set defaults */
   state->ltype = BT_INTERP_NATURAL;
   state->rtype = BT_INTERP_NATURAL;
   state->alpha = 0.0;
   state->beta = 0.0;
   state->s_2p = 0;
   state->s_3p = 0;
   state->cstate = 0;
   
   /* Allocate size for the size-specific coefficients */
   switch (size)
   {
      case 2:
         state->s_2p = (struct state_2p *) malloc( sizeof(struct state_2p) );
         if (!state->s_2p)
         {
            free(state);
            return 0;
         }
         break;
      case 3:
         state->s_3p = (struct state_3p *) malloc( sizeof(struct state_3p) );
         if (!state->s_3p)
         {
            free(state);
            return 0;
         }
         break;
      default:
         state->cstate = gsl_interp_cspline->alloc(size);
         if (!state->cstate)
         {
            free(state);
            return 0;
         }
   }
   return state;
}


static int init(void * vstate, const double xa[], const double ya[], size_t size)
{
   struct state_t * state = (struct state_t *) vstate;
   
   switch (size)
   {
      case 2:
      {
         double A[16];
         double x[4];
         double b[4];
         
         /* Set up the system */
         {
            int i;
            double h = xa[1] - xa[0];
            for (i=0; i<16; i++)
               A[i] = 0.0;
            
            /* Zeroth-order, all points have the same value */
            /* a = y0 */
            A[0*4+0] = 1.0;
            b[0] = ya[0];
            /* a, b, c, d = y1 */
            A[1*4+0] = 1.0;
            A[1*4+1] = h;
            A[1*4+2] = h*h;
            A[1*4+3] = h*h*h;
            b[1] = ya[1];
            
            /* The last two equations depend on the endpoint types */
            if (state->ltype == BT_INTERP_SLOPE)
            {
               /* b = alpha */
               A[2*4+1] = 1.0;
               b[2] = state->alpha;
            }
            else /* state->ltype == WAM_INTERP_NATURAL */
            {
               /* c = 0 */
               A[2*4+2] = 2.0;
               b[2] = 0.0;
            }
            if (state->rtype == BT_INTERP_SLOPE)
            {
               /* b, c, d = beta */
               A[3*4+1] = 1.0;
               A[3*4+2] = 2*h;
               A[3*4+3] = 3*h*h;
               b[3] = state->beta;
            }
            else /* state->ltype == WAM_INTERP_NATURAL */
            {
               /* c, d = 0 */
               A[3*4+2] = 2.0;
               A[3*4+3] = 6*h;
               b[3] = 0.0;
            }
         }
         
         /* Solve the system */
         {
            gsl_matrix_view A_view = gsl_matrix_view_array(A,4,4);
            gsl_vector_view b_view = gsl_vector_view_array(b,4);
            gsl_vector_view x_view = gsl_vector_view_array(x,4);
            gsl_permutation * p = gsl_permutation_alloc(4);
            int s;
            
            gsl_linalg_LU_decomp( &A_view.matrix, p, &s);
            gsl_linalg_LU_solve( &A_view.matrix, p, &b_view.vector, &x_view.vector );
            
            gsl_permutation_free(p);
         }
         
         /* Save the solved values */
         state->s_2p->a = x[0];
         state->s_2p->b = x[1];
         state->s_2p->c = x[2];
         state->s_2p->d = x[3];
      } break;
      case 3:
      {
         double A[64];
         double x[8];
         double b[8];
         
         /* Set up the system */
         {
            int i;
            double hL = xa[1] - xa[0];
            double hR = xa[2] - xa[1];
            for (i=0; i<64; i++)
               A[i] = 0.0;
            
            /* Zeroth-order, all points have the same value */
            /* aL = y0 */
            A[0*8+0] = 1.0;
            b[0] = ya[0];
            /* aL, bL, cL, dL = y1 */
            A[1*8+0] = 1.0;
            A[1*8+1] = hL;
            A[1*8+2] = hL*hL;
            A[1*8+3] = hL*hL*hL;
            b[1] = ya[1];
            /* aR = y1 */
            A[2*8+4] = 1.0;
            b[2] = ya[1];
            /* aR, bR, cR, dR = y2 */
            A[3*8+4] = 1.0;
            A[3*8+5] = hR;
            A[3*8+6] = hR*hR;
            A[3*8+7] = hR*hR*hR;
            b[3] = ya[2];
            
            /* First-order, the slopes are the same at the midpoint */
            /* bL, cL, dL, -bR = 0 */
            A[4*8+1] = 1.0;
            A[4*8+2] = 2*hL;
            A[4*8+3] = 3*hL*hL;
            A[4*8+5] = -1.0;
            b[4] = 0.0;
            
            /* Second-order, the concavity is the same at the midpoint */
            /* cL, dL, -cR = 0 */
            A[5*8+2] = 2.0;
            A[5*8+3] = 6*hL;
            A[5*8+6] = -2.0;
            b[5] = 0.0;
            
            /* The last two equations depend on the endpoint types */
            if (state->ltype == BT_INTERP_SLOPE)
            {
               /* bL = alpha */
               A[6*8+1] = 1.0;
               b[6] = state->alpha;
            }
            else /* state->ltype == WAM_INTERP_NATURAL */
            {
               /* cL = 0 */
               A[6*8+2] = 2.0;
               b[6] = 0.0;
            }
            if (state->rtype == BT_INTERP_SLOPE)
            {
               /* bR, cR, dR = beta */
               A[7*8+5] = 1.0;
               A[7*8+6] = 2*hR;
               A[7*8+7] = 3*hR*hR;
               b[7] = state->beta;
            }
            else /* state->rtype == WAM_INTERP_NATURAL */
            {
               /* cR, dR = 0 */
               A[7*8+6] = 2.0;
               A[7*8+7] = 6*hL;
               b[7] = 0.0;
            }
         }
         
         /* Solve the system */
         {
            gsl_matrix_view A_view = gsl_matrix_view_array(A,8,8);
            gsl_vector_view b_view = gsl_vector_view_array(b,8);
            gsl_vector_view x_view = gsl_vector_view_array(x,8);
            gsl_permutation * p = gsl_permutation_alloc(8);
            int s;
            
            gsl_linalg_LU_decomp( &A_view.matrix, p, &s);
            gsl_linalg_LU_solve( &A_view.matrix, p, &b_view.vector, &x_view.vector );
            
            gsl_permutation_free(p);
         }
         
         /* Save the solved values */
         state->s_3p->aL = x[0];
         state->s_3p->bL = x[1];
         state->s_3p->cL = x[2];
         state->s_3p->dL = x[3];
         state->s_3p->aR = x[4];
         state->s_3p->bR = x[5];
         state->s_3p->cR = x[6];
         state->s_3p->dR = x[7];
      } break;
      default:
      {
         /* spline calculation with natural boundary conditions
          * or with defined first and/or last derivatives
          * see [Engeln-Mullges + Uhlig, p. 258]
          */
         /* Note - this is mostly duplication. Oh well. */
         size_t i;
         size_t num_points = size;
         size_t max_index = num_points - 1;  /* Engeln-Mullges + Uhlig "n" */
         size_t sys_size = max_index - 1;    /* linear system is sys_size x sys_size */

         /* Note - moved outer c setting to below */
         
         /* Set up the system for the inner c's */
         for (i = 0; i < sys_size; i++)
         {
            const double h_i   = xa[i + 1] - xa[i];
            const double h_ip1 = xa[i + 2] - xa[i + 1];
            const double ydiff_i   = ya[i + 1] - ya[i];
            const double ydiff_ip1 = ya[i + 2] - ya[i + 1];
            const double g_i = (h_i != 0.0) ? 1.0 / h_i : 0.0;
            const double g_ip1 = (h_ip1 != 0.0) ? 1.0 / h_ip1 : 0.0;
            state->cstate->offdiag[i] = h_ip1;
            /* added in here ######### */
            if (state->ltype == BT_INTERP_SLOPE && i==0)
            {
               state->cstate->diag[i] = 1.5 * h_i + 2.0 * h_ip1;
               state->cstate->g[i] = 3.0 * (ydiff_ip1 * g_ip1 - 0.5 * ( 3.0*(ydiff_i*g_i) - state->alpha ) );
               continue;
            }
            if (state->rtype == BT_INTERP_SLOPE && i == sys_size-1)
            {
               state->cstate->diag[i] = 2.0 * h_i + 1.5 * h_ip1;
               state->cstate->g[i] = 3.0 * ( 0.5 * ( 3.0*(ydiff_ip1*g_ip1) - state->beta ) - ydiff_i * g_i );
               continue;
            }
            /* ####################### */
            state->cstate->diag[i] = 2.0 * (h_ip1 + h_i);
            state->cstate->g[i] = 3.0 * (ydiff_ip1 * g_ip1 - ydiff_i * g_i);
         }
         
         /* Solve the system for the inner c's */
         {
            gsl_vector_view g_vec = gsl_vector_view_array(state->cstate->g, sys_size);
            gsl_vector_view diag_vec = gsl_vector_view_array(state->cstate->diag, sys_size);
            gsl_vector_view offdiag_vec = gsl_vector_view_array(state->cstate->offdiag, sys_size - 1);
            gsl_vector_view solution_vec = gsl_vector_view_array ((state->cstate->c) + 1, sys_size);
            
            int status = gsl_linalg_solve_symm_tridiag(&diag_vec.vector, 
                                                       &offdiag_vec.vector, 
                                                       &g_vec.vector, 
                                                       &solution_vec.vector);
            if (status != GSL_SUCCESS) return status;
         }
         
         /* Set the outer c's.  */
         if (state->ltype == BT_INTERP_SLOPE)
         {
            const double h_0 = xa[1] - xa[0];
            const double ydiff_0 = ya[1] - ya[0];
            state->cstate->c[0] = 1.0/(2.0*h_0) * ( (3.0/h_0)*ydiff_0 - 3.0*state->alpha - state->cstate->c[1]*h_0 );
         }
         else
            state->cstate->c[0] = 0.0;
         if (state->rtype == BT_INTERP_SLOPE)
         {
            const double h_nm1 = xa[max_index] - xa[max_index-1];
            const double ydiff_nm1 = ya[max_index] - ya[max_index-1];
            
            state->cstate->c[max_index] = - 1.0/(2.0*h_nm1) * ( (3.0/h_nm1)*ydiff_nm1 - 3.0*state->beta + state->cstate->c[max_index-1]*h_nm1 );
         }
         else
            state->cstate->c[max_index] = 0.0;
      } break;
   }
   
   return GSL_SUCCESS;
}


static int eval(const void * vstate,
   const double x_array[], const double y_array[], size_t size,
   double x, gsl_interp_accel * acc, double * result)
{
   struct state_t * state = (struct state_t *) vstate;
   switch (size)
   {
      case 2:
         {
            double delx = x - x_array[0];
            *result = state->s_2p->a + delx * (state->s_2p->b + delx * (state->s_2p->c + delx * state->s_2p->d));
            return GSL_SUCCESS;
         }
      case 3:
         if (x < x_array[1])
         {
            double delx = x - x_array[0];
            *result = state->s_3p->aL + delx * (state->s_3p->bL + delx * (state->s_3p->cL + delx * state->s_3p->dL));
            return GSL_SUCCESS;
         }
         else
         {
            double delx = x - x_array[1];
            *result = state->s_3p->aR + delx * (state->s_3p->bR + delx * (state->s_3p->cR + delx * state->s_3p->dR));
            return GSL_SUCCESS;
         }
      default:
         return gsl_interp_cspline->eval(state->cstate,x_array,y_array,size,x,acc,result);
   }
}


static int eval_deriv(const void * vstate,
   const double x_array[], const double y_array[], size_t size,
   double x, gsl_interp_accel * acc, double * result)
{
   struct state_t * state = (struct state_t *) vstate;
   switch (size)
   {
      case 2:
         {
            double delx = x - x_array[0];
            *result = state->s_2p->b + delx * (2 * state->s_2p->c + delx * 3 * state->s_2p->d);
            return GSL_SUCCESS;
         }
      case 3:
         if (x < x_array[1])
         {
            double delx = x - x_array[0];
            *result = state->s_3p->bL + delx * (2 * state->s_3p->cL + delx * 3 * state->s_3p->dL);
            return GSL_SUCCESS;
         }
         else
         {
            double delx = x - x_array[1];
            *result = state->s_3p->bR + delx * (2 * state->s_3p->cR + delx * 3 * state->s_3p->dR);
            return GSL_SUCCESS;
         }
      default:
         return gsl_interp_cspline->eval_deriv(state->cstate,x_array,y_array,size,x,acc,result);
   }
}


static int eval_deriv2(const void * vstate,
   const double x_array[], const double y_array[], size_t size,
   double x, gsl_interp_accel * acc, double * result)
{
   struct state_t * state = (struct state_t *) vstate;
   switch (size)
   {
      case 2:
         {
            double delx = x - x_array[0];
            *result = 2 * state->s_2p->c + 6 * delx * state->s_2p->d;
            return GSL_SUCCESS;
         }
      case 3:
         if (x < x_array[1])
         {
            double delx = x - x_array[0];
            *result = 2 * state->s_3p->cL + 6 * delx * state->s_3p->dL;
            return GSL_SUCCESS;
         }
         else
         {
            double delx = x - x_array[1];
            *result = 2 * state->s_3p->cR + 6 * delx * state->s_3p->dR;
            return GSL_SUCCESS;
         }
      default:
         return gsl_interp_cspline->eval_deriv2(state->cstate,x_array,y_array,size,x,acc,result);
   }
}


static int eval_integ(const void * vstate,
   const double x_array[], const double y_array[], size_t size,
   gsl_interp_accel * acc, double a, double b, double * result)
{
   struct state_t * state = (struct state_t *) vstate;
   switch (size)
   {
      case 2:
      case 3:
         GSL_ERROR("wam_interp: integrating with size=2,3 not yet implemented.",GSL_EINVAL);
      default:
         return gsl_interp_cspline->eval_integ(state->cstate,x_array,y_array,size,acc,a,b,result);
   }
}


static void w_free(void * vstate)
{
   struct state_t * state = (struct state_t *) vstate;
   
   if (state->s_2p)   free(state->s_2p);
   if (state->s_3p)   free(state->s_3p);
   if (state->cstate) gsl_interp_cspline->free(state->cstate);
   
   free(state);
   return;
}


/* Make a GSL interp object to integrate with the framework */
static const gsl_interp_type bt_interp_type = 
{
  "wam-interp", 
  2,
  &alloc,
  &init,
  &eval,
  &eval_deriv,
  &eval_deriv2,
  &eval_integ,
  &w_free
};


const gsl_interp_type * bt_interp = &bt_interp_type;


int bt_interp_set_type( gsl_interp * interp,
   enum bt_interp_type ltype, enum bt_interp_type rtype )
{
   struct state_t * state;
   
   /* Make sure its our type */
   if (interp->type != bt_interp)
      return -1;
   
   state = (struct state_t *) interp->state;
   
   state->ltype = ltype;
   state->rtype = rtype;
   
   return 0;
}


int bt_interp_set_slopes( gsl_interp * interp,
   double lslope, double rslope )
{
   struct state_t * state;
   
   /* Make sure its our type */
   if (interp->type != bt_interp)
      return -1;
   
   state = (struct state_t *) interp->state;
   
   state->alpha = lslope;
   state->beta = rslope;
   
   return 0;
}
