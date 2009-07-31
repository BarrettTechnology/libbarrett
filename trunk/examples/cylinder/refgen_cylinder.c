
#include <stdlib.h>
#include <math.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include <syslog.h>

#include "refgen_cylinder.h"
#include "libbarrett/gsl.h"

#define RES (0.001)

/* compute the unit vector and h_max */
static int compute_unit(struct refgen_cylinder * r);

#define RADIUS_H (0.0010)
#define RADIUS_CIRCUM (0.0010)
#define BINARY_DEPTH (8)

static int destroy(struct bt_refgen * base);
static int teach_init(struct bt_refgen * base);
static int teach_flush(struct bt_refgen * base);
static int teach_end(struct bt_refgen * base);
static int teach_start(struct bt_refgen * base);
static int teach_trigger(struct bt_refgen * base, double time,
                         gsl_vector * cur_position);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, double time, gsl_vector * ref);
static const struct bt_refgen_type refgen_cylinder_type = {
   "cylinder",
   0, /* create */
   &destroy,
   &teach_init,
   &teach_flush,
   &teach_end,
   &teach_start,
   &teach_trigger,
   0, 0, /* load/save functions */
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval,
};
const struct bt_refgen_type * refgen_cylinder = &refgen_cylinder_type;

/* Generic parameterization */
void func(struct refgen_cylinder * r, double h, double theta, gsl_vector * pos)
{
   double rad;
   rad = gsl_interp_eval( r->interp, r->hs, r->rs, h, r->acc );
   
   gsl_vector_memcpy( pos, r->bottom );
   
   gsl_blas_daxpy( h, r->unit, pos );
   
   gsl_blas_daxpy( rad * cos(theta), r->e1, pos );
   gsl_blas_daxpy( rad * sin(theta), r->e2, pos );
}

/* Functions */
struct refgen_cylinder * refgen_cylinder_create(gsl_vector * cpos)
{
   struct refgen_cylinder * r;
   r = (struct refgen_cylinder *) malloc(sizeof(struct refgen_cylinder));
   if (!r) return 0;
   r->base.type = refgen_cylinder;
   
   r->temp = gsl_vector_alloc(3);
   r->start = gsl_vector_calloc(3);
   
   /* save the cartesian position,
    * set on refgen creation */
   r->cpos = cpos;
   
   /* These are allocated and set using set_top() and set_bottom(),
    * and set using compute_unit() */
   r->top = 0;
   r->bottom = 0;
   r->unit = gsl_vector_calloc(3);
   r->h_max = 0;
   r->e1 = gsl_vector_calloc(3);
   r->e2 = gsl_vector_calloc(3);
   
   /* These are allocated using compute_unit(),
    * and trimmed using init() */
   r->func_sz = 0;
   r->hs = 0;
   r->rs = 0;
   r->rs_set = 0;
   
   /* These are created using init() */
   r->interp = 0;
   r->acc = 0;
   
   return r;
}

/* Functions for setting the top and bottom positions */
int refgen_cylinder_set_top(struct refgen_cylinder * r)
{
   if (r->top)
      return -1;
   
   r->top = gsl_vector_calloc(3);
   gsl_vector_memcpy(r->top,r->cpos);
   
   if (r->bottom)
      return compute_unit(r);
   
   return 0;
}
int refgen_cylinder_set_bottom(struct refgen_cylinder * r)
{
   if (r->bottom)
      return -1;
   
   r->bottom = gsl_vector_calloc(3);
   gsl_vector_memcpy(r->bottom,r->cpos);
   
   if (r->top)
      return compute_unit(r);
   
   return 0;
}
static int compute_unit(struct refgen_cylinder * r)
{
   int i;
   double len;
   
   /* Compute r->unit and r->h_max */
   gsl_blas_dcopy( r->top, r->unit );
   gsl_blas_daxpy( -1.0, r->bottom, r->unit );
   r->h_max = gsl_blas_dnrm2(r->unit);
   gsl_blas_dscal( 1.0/r->h_max, r->unit );
   
   /* Compute e1 and e2 */
   gsl_vector_set_zero(r->e2);
   gsl_vector_set_zero(r->temp);
   gsl_vector_set( r->temp, 0, 1.0 );
   
   bt_gsl_cross( r->unit, r->temp, r->e2 );
   len = gsl_blas_dnrm2(r->e2);
   gsl_blas_dscal( 1.0/len, r->e2 );
   
   gsl_vector_set_zero(r->e1);
   bt_gsl_cross( r->e2, r->unit, r->e1 );
   
   /* Allocate memory for r->hs and r->rs */
   r->func_sz = (int) floor(r->h_max / RES) + 1;
   r->hs = (double *) malloc( r->func_sz * sizeof(double) );
   r->rs = (double *) malloc( r->func_sz * sizeof(double) );
   r->rs_set = (char *) malloc( r->func_sz );
   
   /* Fill in r->hs and r->rs_set */
   for (i=0; i<r->func_sz; i++)
   {
      r->hs[i] = (RES/2) + (i * RES);
      r->rs_set[i] = 0;
   }
   
   return 0;
}

/* Generic refgen functions */
static int destroy(struct bt_refgen * base)
{
   struct refgen_cylinder * r = (struct refgen_cylinder *) base;
   if (r->top) gsl_vector_free(r->top);
   if (r->bottom) gsl_vector_free(r->bottom);
   gsl_vector_free(r->e1);
   gsl_vector_free(r->e2);
   if (r->hs) free(r->hs);
   if (r->rs) free(r->rs);
   gsl_vector_free(r->temp);
   if (r->interp) gsl_interp_free(r->interp);
   if (r->acc) gsl_interp_accel_free(r->acc);
   gsl_vector_free(r->start);
   free(r);
   return 0;
}


static int teach_init(struct bt_refgen * base)
{
   return 0;
}
static int teach_flush(struct bt_refgen * base)
{
   return 0;
}
static int teach_end(struct bt_refgen * base)
{
   int oi; /* old index */
   int ni; /* old index */
   struct refgen_cylinder * r = (struct refgen_cylinder *) base;

   if (r->interp)
      return -1;
   
   /* Eliminate any un-set values in the arrays */
   ni = 0;
   for (oi=0; oi<r->func_sz; oi++)
   {
      if (r->rs_set[oi])
      {
         /* Copy the values */
         r->hs[ni] = r->hs[oi];
         r->rs[ni] = r->rs[oi];
         ni++;
      }
   }
   r->func_sz = ni;
   free(r->rs_set);
   r->rs_set = 0;
   r->hs = (double *) realloc( r->hs, r->func_sz * sizeof(double) );
   r->rs = (double *) realloc( r->rs, r->func_sz * sizeof(double) );
   
   /* Create a gsl interpolator for the h -> r function */
   r->interp = gsl_interp_alloc( gsl_interp_cspline, r->func_sz );
   gsl_interp_init( r->interp, r->hs, r->rs, r->func_sz );
   r->acc = gsl_interp_accel_alloc();
   
   /* Compute the start vector */
   func(r, r->h_max, 0.0, r->start);
   
   return 0;
}
static int teach_start(struct bt_refgen * base)
{
   return 0;
}
static int teach_trigger(struct bt_refgen * base, double time,
                         gsl_vector * cur_position)
{
   struct refgen_cylinder * r;
   double h;
   double bb; /* length of b squared */
   double rad;
   int i;
   
   r = (struct refgen_cylinder *) base;
   
   if (r->interp)
      return -1;
   
   /* Get the current h and r */
   gsl_blas_dcopy( r->cpos, r->temp );
   gsl_blas_daxpy( -1.0, r->bottom, r->temp );
   gsl_blas_ddot( r->temp, r->unit, &h);
   gsl_blas_ddot( r->temp, r->temp, &bb );
   rad = sqrt( bb - h*h );
   
   /* Ignore if h is out of range */
   if (h < 0 || r->h_max < h)
      return 0;
   
   /* Lookup the index for h */
   i = (int) floor(h / RES);
   if ( !r->rs_set[i] )
   {
      r->rs[i] = rad;
      r->rs_set[i] = 1;
   }
   else if ( r->rs[i] > rad )
      r->rs[i] = rad;
   
   return 0;
}


static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct refgen_cylinder * r = (struct refgen_cylinder *) base;
   (*start) = r->start;
   return 0;
}
static int get_total_time(struct bt_refgen * base, double * time)
{
   (*time) = 0.0;
   return 0;
}
static int get_num_points(struct bt_refgen * base, int * points)
{
   (*points) = 1;
   return 0;
}
static int start(struct bt_refgen * base)
{
   struct refgen_cylinder * r = (struct refgen_cylinder *) base;
   r->h = r->h_max;
   r->theta = 0.0;
   return 0;
}
static int eval(struct bt_refgen * base, double time, gsl_vector * ref)
{
   int i;
   double h, theta;
   double error;
   double new_h_error, new_theta_error;
   double radius_h, radius_theta;
   double rad;
   struct refgen_cylinder * r = (struct refgen_cylinder *) base;
   /* We have r->cpos, the current position */
   
   /* Our initial best guess is h, theta. */
   h = r->h;
   theta = r->theta;
   
   /* Get current radius */
   rad = gsl_interp_eval( r->interp, r->hs, r->rs, h, r->acc );
   
   radius_h = RADIUS_H;
   radius_theta = RADIUS_CIRCUM / rad;
   
   /* Do a binary search through parameters (h,theta) */
   for (i=0; i<BINARY_DEPTH; i++)
   {
      /* Evaluate current error */
      func(r,h,theta, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      error = gsl_blas_dnrm2( r->temp );
      
      /* Test a */
      func(r,h+radius_h,theta, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_h_error = gsl_blas_dnrm2( r->temp );
      
      /* Test b */
      func(r,h,theta+radius_theta, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_theta_error = gsl_blas_dnrm2( r->temp );
      
      radius_h /= 2;
      radius_theta /= 2;
      
      /* Adjust parameters */
      if (new_h_error < error) h += radius_h;
      else                     h -= radius_h;
      
      if (new_theta_error < error) theta += radius_theta;
      else                         theta -= radius_theta;
   }
   
   /* Respect bounds on h, theta */
   if (h < 0)        h = 0;
   if (h > r->h_max) h = r->h_max;
   if (theta < 0)        theta += 2 * M_PI;
   if (theta > 2 * M_PI) theta -= 2 * M_PI;
   
   func(r,h,theta,ref);
   
   r->h = h;
   r->theta = theta;
   
   return 0;
}
