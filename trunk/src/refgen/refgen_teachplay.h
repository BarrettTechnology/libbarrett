#include "refgen.h"

#include "log.h"

#include "spline.h"
#include "profile.h"

#include <gsl/gsl_vector.h>

/* Publicize the type */
const struct bt_refgen_type * bt_refgen_teachplay;

struct bt_refgen_teachplay
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
   
};

/* refgen-specific creation function */
struct bt_refgen_teachplay * bt_refgen_teachplay_create(
   double * elapsed_time, gsl_vector * cur_position, char * filename);
int bt_refgen_teachplay_flush(struct bt_refgen_teachplay * t);
int bt_refgen_teachplay_save(struct bt_refgen_teachplay * t);
