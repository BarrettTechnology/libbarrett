#include "refgen.h"
#include "refgen_teachplay.h"

#include "gsl.h"

#include <string.h>

#include <syslog.h>

#include <gsl/gsl_vector.h>

/* Define the type (see refgen.h for details) */
static int create(struct bt_refgen ** refgenptr, int n);
static int destroy(struct bt_refgen * base);

static int teach_init(struct bt_refgen * base);
static int teach_flush(struct bt_refgen * base);
static int teach_end(struct bt_refgen * base);
static int teach_start(struct bt_refgen * base);
static int teach_trigger(struct bt_refgen * base, double time,
                         gsl_vector * cur_position);

static int load(struct bt_refgen * base, config_setting_t * setting);
static int save(struct bt_refgen * base, config_setting_t * setting);

static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, double time, gsl_vector * ref);

static const struct bt_refgen_type bt_refgen_teachplay_type = {
   "teachplay",
   &create,
   &destroy,
   &teach_init,
   &teach_flush,
   &teach_end,
   &teach_start,
   &teach_trigger,
   &load,
   &save,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval
};
const struct bt_refgen_type * bt_refgen_teachplay = &bt_refgen_teachplay_type;


static int create(struct bt_refgen ** refgenptr, int n)
{
   struct bt_refgen_teachplay * r;

   (*refgenptr) = 0;
   r = (struct bt_refgen_teachplay *) malloc( sizeof(struct bt_refgen_teachplay) );
   if (!r) return -1;
   
   /* Set the type */
   r->base.type = bt_refgen_teachplay;

   /* Initialize */
   r->n = n;
   r->start = 0;
   r->position = 0;
   r->log = 0;
   r->spline = 0;

   r->start = gsl_vector_calloc(r->n);
   r->position = gsl_vector_calloc(r->n);

   (*refgenptr) = (struct bt_refgen *)r;
   return 0;
}


static int destroy(struct bt_refgen * base)
{
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;
   
   if (r->log) bt_log_destroy(r->log);
   if (r->spline) bt_spline_destroy(r->spline);
   if (r->start) gsl_vector_free(r->start);
   if (r->position) gsl_vector_free(r->position);
   free(r);
   
   return 0;
}


static int teach_init(struct bt_refgen * base)
{
   int i;
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;

   /* Make sure we're ready to initialize teaching */

   /* Create a log object (a time field, plus a field for each dimension) */
   bt_log_create(&r->log, 1 + r->position->size);
   bt_log_addfield(r->log, &r->time, 1, BT_LOG_DOUBLE, "time");
   for (i=0; i<r->position->size; i++)
      bt_log_addfield(r->log, gsl_vector_ptr(r->position,i), 1, BT_LOG_DOUBLE, "pos");
   bt_log_init( r->log, 1000, 0 ); /* Use a temp file */

   return 0;
}


static int teach_flush(struct bt_refgen * base)
{
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;

   return bt_log_flush(r->log);
}


static int teach_end(struct bt_refgen * base)
{
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;
   
   /* OK, we're done teaching. Flush and finish the log file */
   bt_log_flush(r->log);
   bt_log_finish(r->log);

   /* Start a decode */
   bt_log_decode_start(r->log);

   /* Get the first point, create the spline  */
   bt_log_decode_next(r->log, 0);
   gsl_vector_memcpy(r->start, r->position);
   bt_spline_create(&r->spline,r->start, BT_SPLINE_MODE_EXTERNAL);
   
   /* Iterate through every subsequent point; don't mind the time, for now */
   while ( !bt_log_decode_next(r->log, 0) )
      bt_spline_add(r->spline, r->position, r->time);
   
   /* Initialize the spline */
   bt_spline_init(r->spline, 0, 0);

   return 0;
}


static int teach_start(struct bt_refgen * base)
{
   return 0;
}


static int teach_trigger(struct bt_refgen * base, double time,
                         gsl_vector * cur_position)
{
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;
   
   /* Copy in the time */
   r->time = time;

   /* Copy in current position */
   gsl_vector_memcpy(r->position, cur_position);
   
   /* Trigger the logger */
   return bt_log_trigger(r->log);
}


static int load(struct bt_refgen * base, config_setting_t * setting)
{
   int record_num;
   int i;
   struct bt_refgen_teachplay * r;
   config_setting_t * data;
   
   r = (struct bt_refgen_teachplay *) base;

   /* Make sure we don't have a log or a spline
    * (that we didn't teach with this object,
    *  and that we haven't already loaded) */
   if (r->log || r->spline)
      return -1;

   data = config_setting_get_member(setting,"data");
   if (!data)
   {
      syslog(LOG_ERR,"No data found in configuration.");
      return -1;
   }

   for (record_num=0; record_num<config_setting_length(data); record_num++)
   {
      config_setting_t * record;
      record = config_setting_get_elem(data,record_num);
      if (!record)
      {
         syslog(LOG_ERR,"Error reading record index %d.",record_num);
         if (r->spline)
         {
            bt_spline_destroy(r->spline);
            r->spline = 0;
            return -1;
         }
      }

      r->time = config_setting_get_float_elem(record,0);

      for (i=0; i<r->n; i++)
         gsl_vector_set(r->position,i,config_setting_get_float_elem(record,i+1));

      if (!record_num)
      {
         gsl_vector_memcpy(r->start, r->position);
         bt_spline_create(&r->spline, r->start, BT_SPLINE_MODE_EXTERNAL);
      }
      else
      {
         bt_spline_add(r->spline, r->position, r->time);
      }
   }

   /* Initialize the spline */
   bt_spline_init( r->spline, 0, 0 );
   
   return 0;
}


static int save(struct bt_refgen * base, config_setting_t * setting)
{
   struct bt_refgen_teachplay * r;
   config_setting_t * data;
   
   r = (struct bt_refgen_teachplay *) base;
   
   /* Make sure we have a log and a spline
    * (that we taught with this object, and it's done) */
   if (!r->log || !r->spline)
      return -1;

   /* Make a new list called data */
   data = config_setting_add(setting,"data",CONFIG_TYPE_LIST);

   /* Go back to the beginning of our log */
   bt_log_decode_start(r->log);

   /* Go through each vector */
   while ( !bt_log_decode_next(r->log, 0) )
   {
      int i;
      config_setting_t * record;

      record = config_setting_add(data,0,CONFIG_TYPE_ARRAY);

      /* Add the time */
      config_setting_set_float_elem(record,-1,r->time);

      /* Add the vector components */
      for (i=0; i<r->n; i++)
         config_setting_set_float_elem(record,-1,gsl_vector_get(r->position,i));
   }

   return 0;
}


/* Interface functions (from refgen.h) */
static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;
   
   (*start) = r->start;
   
   return 0;
}


static int get_total_time(struct bt_refgen * base, double * time)
{
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;
   if (!r->spline) return -1;
   (*time) = r->spline->length;
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


static int eval(struct bt_refgen * base, double time, gsl_vector * ref)
{
   struct bt_refgen_teachplay * r;
   r = (struct bt_refgen_teachplay *) base;
   
   if ( time > r->spline->length )
      return 1; /* finished */
   
   if (!r->spline) return -1;
   
   bt_spline_get(r->spline, ref, time);
   
   return 0;
}
