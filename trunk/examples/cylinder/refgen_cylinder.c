
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include "refgen_cylinder.h"

#define SURF_A_RADIUS (0.0005)
#define SURF_B_RADIUS (0.0005)

static int destroy(struct bt_refgen * base);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, gsl_vector * ref);
static int trigger(struct bt_refgen * base);
static const struct bt_refgen_type refgen_cylinder_type = {
   "cylinder",
   &destroy,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval,
   &trigger
};
const struct bt_refgen_type * refgen_cylinder = &refgen_cylinder_type;

/* Generic parameterization */
void func(double a, double b, gsl_vector * pos)
{
   double r2;
   r2 = a*a + b*b;
   gsl_vector_set(pos,0,a+0.5);
   gsl_vector_set(pos,1,b);
   if (r2 < 0.04)
      gsl_vector_set(pos,2, -4*r2);
   else
      gsl_vector_set(pos,2, -4*0.04);
}

/* Functions */
struct refgen_cylinder * refgen_cylinder_create(gsl_vector * cpos)
{
   struct refgen_cylinder * r;
   r = (struct refgen_cylinder *) malloc(sizeof(struct refgen_cylinder));
   if (!r) return 0;
   r->base.type = refgen_cylinder;
   
   /* save the cartesian position */
   r->cpos = cpos;
   
   r->start = gsl_vector_calloc(3);
   func( 0.0, 0.0, r->start );
   
   r->temp = gsl_vector_alloc(3);
   
   return r;
}

/* Generic refgen functions */
static int destroy(struct bt_refgen * base)
{
   struct refgen_cylinder * r = (struct refgen_cylinder *) base;
   gsl_vector_free(r->start);
   gsl_vector_free(r->temp);
   free(r);
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
   r->a = 0.0;
   r->b = 0.0;
   return 0;
}
static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   int i;
   double a, b;
   double error;
   double new_a_error, new_b_error;
   double a_radius, b_radius;
   struct refgen_cylinder * r = (struct refgen_cylinder *) base;
   /* We have r->cpos, the current position */
   
   /* Our initial best guess is a, b. */
   a = r->a;
   b = r->b;
   
   a_radius = SURF_A_RADIUS;
   b_radius = SURF_B_RADIUS;
   
   /* Do a binary search through parameters (a,b) */
   for (i=0; i<5; i++)
   {
      /* Evaluate current error */
      func(a,b, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      error = gsl_blas_dnrm2( r->temp );
      
      /* Test a */
      func(a+a_radius,b, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_a_error = gsl_blas_dnrm2( r->temp );
      
      /* Test b */
      func(a,b+b_radius, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_b_error = gsl_blas_dnrm2( r->temp );
      
      a_radius /= 2;
      b_radius /= 2;
      
      /* Adjust parameters */
      if (new_a_error < error) a += a_radius;
      else                     a -= a_radius;
      
      if (new_b_error < error) b += b_radius;
      else                     b -= b_radius;
   }
     
   func(a,b,ref);
   
   r->a = a;
   r->b = b;
   
   return 0;
}

static int trigger(struct bt_refgen * base)
{
   return -1;
}
