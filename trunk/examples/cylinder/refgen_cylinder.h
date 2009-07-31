/* This is a custom refgen that traces a two-dimensional
 * surface with cylindrical symmetry. */

#include <libbarrett/refgen.h>

#include <gsl/gsl_interp.h>

/* Here we define our own refgen */
struct refgen_cylinder
{
   /* Include the base */
   struct bt_refgen base;
   gsl_vector * cpos; /* current, saved */
   
   gsl_vector * top;
   gsl_vector * bottom;
   gsl_vector * unit; /* Unit vector from bottom -> top */
   double h_max; /* max height, from bottom -> top */
   gsl_vector * e1;
   gsl_vector * e2;
   
   int func_sz;
   double * hs;
   double * rs;
   char * rs_set;
   
   gsl_interp * interp;
   gsl_interp_accel * acc;
   
   gsl_vector * start; /* mine, constant */
   gsl_vector * temp; /* test 3-vector to try */
   
   /* 2-D parameters */
   double h;
   double theta;
};

/* Custom functions */
struct refgen_cylinder * refgen_cylinder_create(gsl_vector * cpos);

int refgen_cylinder_set_top(struct refgen_cylinder * r);
int refgen_cylinder_set_bottom(struct refgen_cylinder * r);
