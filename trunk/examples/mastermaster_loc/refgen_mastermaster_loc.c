#include <sys/socket.h> /* For sockets */
#include <fcntl.h> /* To change socket to nonblocking mode */
#include <arpa/inet.h> /* For inet_aton() */

#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <syslog.h>
#include "refgen_mastermaster_loc.h"

#define PORT (5555)

static int destroy(struct bt_refgen * base);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, gsl_vector * ref);
static int trigger(struct bt_refgen * base);
static const struct bt_refgen_type refgen_mastermaster_loc_type = {
   "mastermaster-loc",
   &destroy,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval,
   &trigger
};
const struct bt_refgen_type * refgen_mastermaster_loc = &refgen_mastermaster_loc_type;

/* Functions */
struct refgen_mastermaster_loc * refgen_mastermaster_loc_create(gsl_vector * jpos)
{
   struct refgen_mastermaster_loc * r;

   /* Create */
   r = (struct refgen_mastermaster_loc *) malloc(sizeof(struct refgen_mastermaster_loc));
   if (!r) return 0;
   
   /* Initialize */
   syslog(LOG_ERR,"Uninit Type: %d",(int)(r->base.type));
   r->base.type = refgen_mastermaster_loc;
   syslog(LOG_ERR,"Type: %d",(int)(r->base.type));
   r->power = 0.5;
   r->jpos = jpos;
   r->start = 0;
   r->temp = 0;
   r->started = 0;
   
   /* Allocate start vector */
   r->start = gsl_vector_calloc(jpos->size);
   if (!r->start)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   r->temp = gsl_vector_calloc(jpos->size);
   if (!r->temp)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }

   syslog(LOG_ERR,"Type: %d",(int)(r->base.type));
   return r;
}

int refgen_mastermaster_loc_set_power(struct refgen_mastermaster_loc * r, double power)
{
   r->power = power;
   return 0;
}

int refgen_mastermaster_loc_set_other(struct refgen_mastermaster_loc * r, struct refgen_mastermaster_loc * other)
{
   r->other = other;
   return 0;
}

static int destroy(struct bt_refgen * base)
{
   struct refgen_mastermaster_loc * r = (struct refgen_mastermaster_loc *) base;
   if (r->start) gsl_vector_free(r->start);
   if (r->temp) gsl_vector_free(r->temp);
   free(r);
   return 0;
}

static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct refgen_mastermaster_loc * r = (struct refgen_mastermaster_loc *) base;
   syslog(LOG_ERR,"%s called ...",__func__);
   
   /* The start location is always at zero position. */
   (*start) = r->start;
   
   syslog(LOG_ERR,"%s returning ...",__func__);
   return 0;
}

static int get_total_time(struct bt_refgen * base, double * time)
{ return 0; }
static int get_num_points(struct bt_refgen * base, int * points)
{ return 0; }

static int start(struct bt_refgen * base)
{
   struct refgen_mastermaster_loc * r = (struct refgen_mastermaster_loc *) base;
   r->started = 1;
   return 0;
}

static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   struct refgen_mastermaster_loc * r = (struct refgen_mastermaster_loc *) base;
   
   /* If we have no other refgen, stay at start location */
   if (!r->other)
   {
      gsl_vector_memcpy(ref,r->start);
      return 0;
   }
   
   /* It the other one is started, then go! */
   if (r->other->started)
   {
      gsl_vector_memcpy( r->temp, r->jpos );
      gsl_blas_dscal( r->power, r->temp );
      gsl_blas_daxpy( 1.0 - r->power, r->other->jpos, r->temp );
   }
   
   /* Copy this into the reference */
   gsl_vector_memcpy( ref, r->temp );

   return 0;
}

/* We don't teach ... */
static int trigger(struct bt_refgen * base)
{ return -1; }
