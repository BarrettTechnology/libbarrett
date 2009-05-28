/* This is a custom refgen that does trimeshes from
 * Solidworks-generated VRML 1.0 ascii files*/

#include <libbarrett/refgen.h>
#include <libbarrett/kinematics.h>

#include <gsl/gsl_vector.h>

struct refgen_mastermaster
{
   struct bt_refgen base;
   gsl_vector * jpos; /* current, saved */
   gsl_vector * jtrq; /* saved */
   struct bt_kinematics * kin;
   
   double power;
   
   gsl_vector * start;
   
   gsl_vector * sendbuf; /* This is always a 7-vector */
   gsl_vector * recvbuf; /* This is always a 7-vector */
   
   int sock;
   
   int num_missed; /* Number of eval() cycles since we got a packet */
   int locked; /* WAM joints locked */

   int wrist_locking;
   int wrist_j5_locked;
   int wrist_j6_locked;

   gsl_vector * g_unit; /* gravity unit vector, probably <0,0,-1> */
   gsl_vector * temp3; /* temporary 3vector used for wrist locking code */
   
};

struct refgen_mastermaster * refgen_mastermaster_create(char * sendtohost,
                                                        struct bt_kinematics * kin,
                                                        gsl_vector * jpos,
                                                        gsl_vector * jtrq);
int refgen_mastermaster_set_power(struct refgen_mastermaster * r, double power);
