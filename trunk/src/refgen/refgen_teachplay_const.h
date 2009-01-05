#include "refgen.h"

#include "log.h"

#include "spline.h"
#include "profile.h"

#include <gsl/gsl_vector.h>

/* Publicize the type */
const struct bt_refgen_type * bt_refgen_teachplay_const;

struct bt_refgen_teachplay_const
{
   /* Include the base */
   struct bt_refgen base;
   
   /* The location of the elapsed time */
   double * elapsed_time;
   
   int n;
   gsl_vector * start;
   
   /* a bt_refgen_teachplay has a log object */
   double time; /* Location used for teaching */
   char * filename;
   struct bt_log * log;
   
   struct bt_spline * spline;
   struct bt_profile * profile;
   
};

/* refgen-specific creation function */
struct bt_refgen_teachplay_const * bt_refgen_teachplay_const_create(
   double * elapsed_time, gsl_vector * cur_position, char * filename);
int bt_refgen_teachplay_const_trigger(struct bt_refgen_teachplay_const * t, double time);
int bt_refgen_teachplay_const_flush(struct bt_refgen_teachplay_const * t);
int bt_refgen_teachplay_const_save(struct bt_refgen_teachplay_const * t);
