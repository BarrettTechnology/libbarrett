#include "refgen.h"
#include "refgen_teachplay_const.h"

#include <string.h>

#include <syslog.h>

#include <gsl/gsl_vector.h>

/* Define the type (see refgen.h for details) */
static int destroy(struct bt_refgen * base);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, gsl_vector * ref);
static int trigger(struct bt_refgen * bas);
static const struct bt_refgen_type bt_refgen_teachplay_const_type = {
   "teachplay_const",
   &destroy,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval,
   &trigger
};
const struct bt_refgen_type * bt_refgen_teachplay_const = &bt_refgen_teachplay_const_type;

/* Trajectory-specific create function */
struct bt_refgen_teachplay_const * bt_refgen_teachplay_const_create(
   double * elapsed_time, gsl_vector * cur_position, char * filename)
{
   struct bt_refgen_teachplay_const * t;
   int i;
   t = (struct bt_refgen_teachplay_const *) malloc( sizeof(struct bt_refgen_teachplay_const) );
   if (!t) return 0;
   
   /* Set the type */
   t->base.type = bt_refgen_teachplay_const;
   
   /* Save the elapsed time */
   t->elapsed_time = elapsed_time;
   
   /* Set the dof */
   t->n = cur_position->size;
   t->start = gsl_vector_calloc(t->n);
   
   /* No spline or profile yet */
   t->spline = 0;
   t->profile = 0;
   
   /* Create a log object (a time field, plus a field for each dimension) */
   t->filename = (char *) malloc( strlen(filename) + 1 );
   strcpy(t->filename,filename);
   t->log = bt_log_create(1 + cur_position->size);
   bt_log_addfield( t->log, &t->time, 1, BT_LOG_DOUBLE, "time" );
   for (i=0; i<cur_position->size; i++)
      bt_log_addfield( t->log, gsl_vector_ptr(cur_position,i), 1, BT_LOG_DOUBLE, "pos" );
   bt_log_init( t->log, 1000, filename );
   
   return t;
}

int bt_refgen_teachplay_const_flush(struct bt_refgen_teachplay_const * t)
{
   return bt_log_flush(t->log);
}

int bt_refgen_teachplay_const_save(struct bt_refgen_teachplay_const * t)
{
   char * filename_csv;
   struct bt_log_read * logread;
   gsl_vector * position;
   int i;
   
   /* Destroy the logger */
   bt_log_destroy(t->log);
   t->log = 0;
   
   /* Convert the file to csv */
   filename_csv = (char *) malloc( strlen(t->filename) + 4 + 1 );
   strcpy(filename_csv,t->filename);
   strcat(filename_csv,".csv");
   bt_log_decode( t->filename, filename_csv, 1, 0 );
   
   /* Create an interable log reader from the CSV file */
   position = gsl_vector_calloc( t->n );
   logread = bt_log_read_create( t->n + 1 );
   bt_log_read_addfield( logread, &t->time, 1, BT_LOG_DOUBLE );
   for (i=0; i<t->n; i++)
      bt_log_read_addfield( logread, gsl_vector_ptr(position,i), 1, BT_LOG_DOUBLE );
   bt_log_read_init( logread, filename_csv, 1);
   
   /* Get the first point, create the spline  */
   bt_log_read_get( logread, 0 );
   gsl_vector_memcpy( t->start, position );
   t->spline = bt_spline_create( position, BT_SPLINE_MODE_ARCLEN );
   
   /* Iterate through every subsequent point; don't mind the time, for now */
   while ( !bt_log_read_get( logread, 0 ) )
      bt_spline_add( t->spline, position, 0.0 ); /* The 0.0 is meaningless for ARCLEN type */
   
   /* Initialize the spline */
   bt_spline_init( t->spline, 0, 0 );
   
   /* Make a new profile */
   t->profile = bt_profile_create(0.5, 0.5, 0.0, t->spline->length);
   
   /* Clean up after outselves ... */
   bt_log_read_destroy( logread );
   gsl_vector_free(position);
   free(filename_csv);
   
   return 0;
}

/* Interface functions (from refgen.h) */
static int destroy(struct bt_refgen * base)
{
   struct bt_refgen_teachplay_const * t;
   t = (struct bt_refgen_teachplay_const *) base;
   if (t->log) bt_log_destroy(t->log);
   if (t->spline) bt_spline_destroy(t->spline);
   if (t->profile) bt_profile_destroy(t->profile);
   free(t->filename);
   gsl_vector_free(t->start);
   free(t);
   return 0;
}

static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct bt_refgen_teachplay_const * t;
   t = (struct bt_refgen_teachplay_const *) base;
   
   (*start) = t->start;
   
   return 0;
}

static int get_total_time(struct bt_refgen * base, double * time)
{
   struct bt_refgen_teachplay_const * t;
   t = (struct bt_refgen_teachplay_const *) base;
   if (!t->profile) return -1;
   (*time) = t->profile->time_end;
   return 0;
}

static int get_num_points(struct bt_refgen * base, int * points)
{
   (*points) = 1337; /* Not Implemented (yet!) */
   return 0;
}



static int start(struct bt_refgen * base)
{
   return -1; /* Not Implemented */
}

static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   struct bt_refgen_teachplay_const * t;
   double s;
   t = (struct bt_refgen_teachplay_const *) base;
   
   if ( *(t->elapsed_time) > t->profile->time_end )
      return 1; /* finished */
   
   if (!t->profile) return -1;
   if (!t->spline) return -1;
   
   bt_profile_get( t->profile, &s, *(t->elapsed_time) );
   bt_spline_get( t->spline, ref, s );
   
   return 0;
}

static int trigger(struct bt_refgen * base)
{
   struct bt_refgen_teachplay_const * t;
   t = (struct bt_refgen_teachplay_const *) base;
   /* Copy in the time */
   t->time = *(t->elapsed_time);
   /* Trigger the logger */
   return bt_log_trigger(t->log);
}
