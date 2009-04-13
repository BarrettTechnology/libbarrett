/* This is a custom refgen that does trimeshes from
 * Solidworks-generated VRML 1.0 ascii files*/

#include <libbarrett/refgen.h>

#include <gsl/gsl_vector.h>

struct refgen_mastermaster
{
   struct bt_refgen base;
   gsl_vector * jpos; /* current, saved */
   
   double amp;
   
   gsl_vector * start;
   
   gsl_vector * sendbuf; /* This is always a 7-vector */
   gsl_vector * recvbuf; /* This is always a 7-vector */
   
   int sock;
   
   int num_missed; /* Number of eval() cycles since we got a packet */
   int locked;
};

struct refgen_mastermaster * refgen_mastermaster_create(char * sendtohost, gsl_vector * jpos, double amp);
int refgen_mastermaster_set_amp(struct refgen_mastermaster * r, double amp);
