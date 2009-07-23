#include <math.h> /* for fabs() */

#include <unistd.h> /* for close() */

#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <syslog.h>

#include <libbarrett/gsl.h>

#include "refgen_mastermaster.h"

#define PORT (5555)
#define J5GAIN (100)

static int destroy(struct bt_refgen * base);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, gsl_vector * ref);
static int trigger(struct bt_refgen * base);
static const struct bt_refgen_type refgen_mastermaster_type = {
   "mastermaster",
   &destroy,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval,
   &trigger
};
const struct bt_refgen_type * refgen_mastermaster = &refgen_mastermaster_type;

/* Functions */
struct refgen_mastermaster * refgen_mastermaster_create(char * sendtohost,
                                                        struct bt_kinematics * kin,
                                                        gsl_vector * jpos,
                                                        gsl_vector * jtrq)
{
   struct refgen_mastermaster * r;
   int err;
   long flags;
   int buflen;
   unsigned int buflenlen;
   struct sockaddr_in bind_addr;
   struct sockaddr_in their_addr;

   /* Create */
   r = (struct refgen_mastermaster *) malloc(sizeof(struct refgen_mastermaster));
   if (!r) return 0;
   
   /* save the joint position vector,
    * set on refgen creation */
   r->base.type = refgen_mastermaster;
   r->power = 0.5;
   r->jpos = jpos;
   r->jtrq = jtrq;
   r->kin = kin;
   
   /* Initialize */
   r->start = 0;
   r->sendbuf = 0;
   r->recvbuf = 0;
   r->sock = -1;
   r->num_missed = 1000;
   r->locked = 0;
   r->wrist_locking = 0;
   r->wrist_j5_locked = 0;
   r->wrist_j6_locked = 0;
   r->temp3 = 0;
   r->g_unit = 0;
   
   /* Allocate start vector */
   r->start = gsl_vector_calloc(jpos->size);
   if (!r->start)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   
   /* Allocate send buffer */
   r->sendbuf = gsl_vector_calloc(7);
   if (!r->sendbuf)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   if (r->sendbuf->stride != 1)
   {
      syslog(LOG_ERR,"%s: sendbuf vector stride (%d) is not 1!",__func__,r->recvbuf->stride);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   
   /* Allocate receive buffer */
   r->recvbuf = gsl_vector_calloc(7);
   if (!r->recvbuf)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   if (r->recvbuf->stride != 1)
   {
      syslog(LOG_ERR,"%s: recvbuf vector stride (%d) is not 1!",__func__,r->recvbuf->stride);
      destroy((struct bt_refgen *)r);
      return 0;
   }

   /* Allocate temp3 vector */
   r->temp3 = gsl_vector_calloc(3);
   if (!r->temp3)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }

   /* Allocate g_unit vector */
   r->g_unit = gsl_vector_calloc(3);
   if (!r->g_unit)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   gsl_vector_set(r->g_unit,2,-1.0);
   
   /* Create socket */
   r->sock = socket(PF_INET, SOCK_DGRAM, 0);
   if (r->sock == -1)
   {
      syslog(LOG_ERR,"%s: Could not create socket.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   
   /* Set socket to non-blocking, set flag associated with open file */
   flags = fcntl(r->sock, F_GETFL, 0);
   if (flags < 0)
   {
      syslog(LOG_ERR,"%s: Could not get socket flags.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   flags |= O_NONBLOCK;
   err = fcntl(r->sock, F_SETFL, flags);
   if (err < 0)
   {
      syslog(LOG_ERR,"%s: Could not set socket flags.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   
   /* Maybe set UDP buffer size? */
   buflenlen = sizeof(buflen);
   err = getsockopt(r->sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);
   
   buflenlen = sizeof(buflen);
   buflen = 5 * 7 * sizeof(double);
   err = setsockopt(r->sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not set output buffer size.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   
   buflenlen = sizeof(buflen);
   err = getsockopt(r->sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);
   
   /* Set up the bind address */
   bind_addr.sin_family = AF_INET;
   bind_addr.sin_port = htons(PORT);
   bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   err = bind(r->sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
   if (err == -1)
   {
      syslog(LOG_ERR,"%s: Could not bind to socket on port %d.",__func__,PORT);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   
   /* Set up the other guy's address */
   their_addr.sin_family = AF_INET;
   their_addr.sin_port = htons(PORT);
   err = ! inet_pton(AF_INET, sendtohost, &their_addr.sin_addr);
   if (err)
   {
      syslog(LOG_ERR,"%s: Bad IP argument '%s'.",__func__,sendtohost);
      destroy((struct bt_refgen *)r);
      return 0;
   }
   
   /* Call "connect" to set datagram destination */
   err = connect(r->sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
   if (err)
   {
      syslog(LOG_ERR,"%s: Could not set datagram destination.",__func__);
      destroy((struct bt_refgen *)r);
      return 0;
   }

   return r;
}

int refgen_mastermaster_set_power(struct refgen_mastermaster * r, double power)
{
   r->power = power;
   return 0;
}

static int destroy(struct bt_refgen * base)
{
   struct refgen_mastermaster * r = (struct refgen_mastermaster *) base;
   
   if (r->sock != -1) close(r->sock);
   if (r->recvbuf) gsl_vector_free(r->recvbuf);
   if (r->sendbuf) gsl_vector_free(r->sendbuf);
   if (r->start) gsl_vector_free(r->start);
   if (r->temp3) gsl_vector_free(r->temp3);
   if (r->g_unit) gsl_vector_free(r->g_unit);
   free(r);
   return 0;
}

static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct refgen_mastermaster * r = (struct refgen_mastermaster *) base;
   
   /* For now, start at the current position (??) */
   (*start) = r->start;
   
   return 0;
}

static int get_total_time(struct bt_refgen * base, double * time)
{ return 0; }
static int get_num_points(struct bt_refgen * base, int * points)
{ return 0; }

static int start(struct bt_refgen * base)
{
   struct refgen_mastermaster * r = (struct refgen_mastermaster *) base;
   r->num_missed = 1000;
   r->locked = 0;
   return 0;
}

static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   int i;
   struct refgen_mastermaster * r = (struct refgen_mastermaster *) base;
   
   /* We have r->jpos, the current position */
   /* Copy the vector into the send buffer */
   for (i=0; i<r->jpos->size; i++)
      *(gsl_vector_ptr(r->sendbuf,i)) = *(gsl_vector_ptr(r->jpos,i));

   /* Send the datagram */
   send(r->sock, r->sendbuf->data, 7*sizeof(double), 0);
   
   /* Read as many datagrams as we can ... */
   if (r->num_missed < 1000)
      r->num_missed++;
   while (recv(r->sock, r->recvbuf->data, 7*sizeof(double), 0) == 7*sizeof(double))
   {
      r->locked = 1;
      r->num_missed = 0;
   }
   
   /* If we've missed a lot, we're done */
   if (r->locked && r->num_missed > 100)
      return 1; /* finished */
   
   /* If we aren't locked yet, stay at the zero position */
   if (!r->locked)
   {
      gsl_vector_memcpy(ref,r->start);
      return 0;
   }
   
   /* Take the average of sendbuf and recvbuf (using sendbuf as temp) */
   gsl_blas_dscal( r->power, r->sendbuf );
   gsl_blas_daxpy( 1.0 - r->power, r->recvbuf, r->sendbuf );

   /* If we're wrist_locking (or locked), compute the J5 torque */
   if (r->wrist_locking)
   {
      double j5torque;
      
      gsl_vector_set_zero(r->temp3);
      /* r->temp = z5 cross g */
      bt_gsl_cross( r->kin->link[4]->axis_z, r->g_unit, r->temp3 );
      /* j5torque = r->temp dot z4 */
      gsl_blas_ddot( r->kin->link[3]->axis_z, r->temp3, &j5torque );
      /* Add in the gain */
      j5torque *= J5GAIN;

      /* If this torque is small enough, lock it! */
      if (!r->wrist_j5_locked && fabs(j5torque) < 1.0)
         r->wrist_j5_locked = 1;

      /* If we're J5 locked, apply the torque */
      if (r->wrist_j5_locked)
         *(gsl_vector_ptr(r->jtrq,4)) += j5torque;

      /* Apply the J6 lock if we're close to 0 */
      if (!r->wrist_j6_locked && (fabs(gsl_vector_get(r->jpos,5)) < 0.01 ) )
         r->wrist_j6_locked = 1;

      /* If we're J6 locked, zero the reference position */
      if (r->wrist_j6_locked)
         gsl_vector_set(r->sendbuf,5,0.0);
   }
   
   /* Copy this into the reference */
   for (i=0; i<ref->size; i++)
      *(gsl_vector_ptr(ref,i)) = *(gsl_vector_ptr(r->sendbuf,i));

   return 0;
}

/* We don't teach ... */
static int trigger(struct bt_refgen * base)
{ return -1; }
