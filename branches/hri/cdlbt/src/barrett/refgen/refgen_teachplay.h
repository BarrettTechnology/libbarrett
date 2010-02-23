#include "refgen.h"

#include "../log/log.h"

#include "../spline/spline.h"
#include "../profile/profile.h"

#include <gsl/gsl_vector.h>

/* Publicize the type */
const struct bt_refgen_type * bt_refgen_teachplay;

struct bt_refgen_teachplay
{
   /* Include the base */
   struct bt_refgen base;

   /* filled on teach_init() or load()
    * or if created by custom creation function */
   int n;
   gsl_vector * start;
   
   /* for teaching */
   gsl_vector * position; /* Location used for teaching log object */
   double time; /* Location used for teaching log object */
   struct bt_log * log;

   /* created by teach_end() or load() */   
   struct bt_spline * spline;
};
