/* This is a custom refgen that does trimeshes from
 * Solidworks-generated VRML 1.0 ascii files*/

#include <libbarrett/refgen.h>

#include <gsl/gsl_vector.h>

struct refgen_mastermaster_loc
{
   struct bt_refgen base;
   gsl_vector * jpos; /* current, saved */
   
   double power;
   
   gsl_vector * start;
   gsl_vector * temp;
   
   int started;
   
   /* The other refgen */
   struct refgen_mastermaster_loc * other;
};

struct refgen_mastermaster_loc * refgen_mastermaster_loc_create(gsl_vector * jpos);
int refgen_mastermaster_loc_set_power(struct refgen_mastermaster_loc * r, double power);
int refgen_mastermaster_loc_set_other(struct refgen_mastermaster_loc * r, struct refgen_mastermaster_loc * other);
