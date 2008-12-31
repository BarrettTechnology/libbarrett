#include "trajectory.h"
#include "trajectory_teachplay.h"

#include <string.h>

#include <syslog.h>

#include <gsl/gsl_vector.h>

/* Define the type */
static int destroy(struct bt_trajectory * base);
static int get_num_points(struct bt_trajectory * base);
static int get_total_time(struct bt_trajectory * base, double * time);
static int get_start(struct bt_trajectory * base, gsl_vector ** start);
static int get_reference(struct bt_trajectory * base, gsl_vector * ref, double time);
static const struct bt_trajectory_type bt_trajectory_teachplay_type = {
   "teachplay",
   &destroy,
   &get_num_points,
   &get_total_time,
   &get_start,
   &get_reference
};
const struct bt_trajectory_type * bt_trajectory_teachplay = &bt_trajectory_teachplay_type;

/* Trajectory-specific create function */
struct bt_trajectory_teachplay * bt_trajectory_teachplay_create(
   gsl_vector * cur_position, char * filename)
{
   struct bt_trajectory_teachplay * t;
   int i;
   t = (struct bt_trajectory_teachplay *) malloc( sizeof(struct bt_trajectory_teachplay) );
   if (!t) return 0;
   
   /* Set the type */
   t->base.type = bt_trajectory_teachplay;
   
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

int bt_trajectory_teachplay_trigger(struct bt_trajectory_teachplay * t, double time)
{
   /* Copy in the time */
   t->time = time;
   /* Trigger the logger */
   return bt_log_trigger(t->log);
}

int bt_trajectory_teachplay_flush(struct bt_trajectory_teachplay * t)
{
   return bt_log_flush(t->log);
}

int bt_trajectory_teachplay_save(struct bt_trajectory_teachplay * t)
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
   t->spline = bt_spline_create( position );
   
   /* Iterate through every subsequent point; don't mind the time, for now */
   while ( !bt_log_read_get( logread, 0 ) )
      bt_spline_add( t->spline, position );
   
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

/* Interface functions (from trajectory.h) */
static int destroy(struct bt_trajectory * base)
{
   struct bt_trajectory_teachplay * t;
   t = (struct bt_trajectory_teachplay *) base;
   if (t->log) bt_log_destroy(t->log);
   if (t->spline) bt_spline_destroy(t->spline);
   if (t->profile) bt_profile_destroy(t->profile);
   free(t->filename);
   gsl_vector_free(t->start);
   free(t);
   return 0;
}

static int get_num_points(struct bt_trajectory * base)
{
   return 0;
}

static int get_total_time(struct bt_trajectory * base, double * time)
{
   struct bt_trajectory_teachplay * t;
   t = (struct bt_trajectory_teachplay *) base;
   if (!t->profile) return -1;
   (*time) = t->profile->time_end;
   return 0;
}

static int get_start(struct bt_trajectory * base, gsl_vector ** start)
{
   struct bt_trajectory_teachplay * t;
   t = (struct bt_trajectory_teachplay *) base;
   
   (*start) = t->start;
   
   return 0;
}

static int get_reference(struct bt_trajectory * base, gsl_vector * ref, double time)
{
   struct bt_trajectory_teachplay * t;
   double s;
   t = (struct bt_trajectory_teachplay *) base;
   
   if (!t->profile) return -1;
   if (!t->spline) return -1;
   
   bt_profile_get( t->profile, &s, time );
   bt_spline_get( t->spline, ref, s );
   
   return 0;
}
