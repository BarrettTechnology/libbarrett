/* This is a custom refgen that traces a two-dimensional
 * surface with cylindrical symmetry. */

#include <libbt/refgen.h>

/* Here we define our own refgen */
struct refgen_cylinder
{
   /* Include the base */
   struct bt_refgen base;
   gsl_vector * cpos; /* current, saved */
   
   gsl_vector * start; /* mine, constant */
   gsl_vector * temp; /* test 3-vector to try */
   
   /* 2-D parameters */
   double a;
   double b;
};

/* Custom functions */
struct refgen_cylinder * refgen_cylinder_create(gsl_vector * cpos);
