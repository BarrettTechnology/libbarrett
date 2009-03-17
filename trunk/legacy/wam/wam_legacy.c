/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... wam_legacy.c
 *  Author ............. Sam Clanton
 *                       Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2004 Q3
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2004-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    a high-level asynchronous non-realtime interface to the WAM
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, split from btwam into wam and wambot
 *
 * ======================================================================== */

#include <stdio.h>
#include <string.h>
#include <syslog.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "wam_internal.h"
#include "wam_legacy.h"


/* #################### MATH STUFF #################### */

/* Private funtion prototypes */
static char * math_sprint(char * buf, struct bt_wam_legacy_math * math);
static struct bt_wam_legacy_math * math_create_from_gsl(void * gsl_ptr, enum bt_wam_legacy_math_type type);
static int math_destroy(vect_n * vec);

vect_n * new_vn(int size)
{
   vect_n * vec;
   vec = (vect_n *) malloc(sizeof(vect_n));
   if (!vec)
      return 0;
   vec->type = BT_WAM_LEGACY_MATH_TYPE_VECT;
   vec->gsl_ptr = (void *) gsl_vector_calloc(size);
   if (!vec->gsl_ptr)
   {
      free(vec);
      return 0;
   }
   return vec;
}

int destroy_vn(vect_n ** p)
{
   gsl_vector_free((gsl_vector *)((*p)->gsl_ptr));
   free(*p);
   (*p) = 0;
   return 0;
}

int len_vn(vect_n* src)
{
   switch (src->type)
   {
      gsl_vector * vec;
      case BT_WAM_LEGACY_MATH_TYPE_VECT:
         vec = (gsl_vector *) src->gsl_ptr;
         return vec->size;
      case BT_WAM_LEGACY_MATH_TYPE_MATR:
         return 16;
      default:
         syslog(LOG_ERR,"%s: Unknown bt math size!",__func__);
         return 0;
   }
}

char * sprint_vn(char *dest,vect_n* src)
{
   return math_sprint(dest,src);
}

double getval_vn(vect_n * src, int i)
{
   switch (src->type)
   {
      gsl_vector * vec;
      case BT_WAM_LEGACY_MATH_TYPE_VECT:
         vec = (gsl_vector *) src->gsl_ptr;
         return gsl_vector_get(vec,i);
      default:
         syslog(LOG_ERR,"%s: Can't get vector value for this type!",__func__);
   }
   return 0.0;
}

vect_3 * new_v3()
{
   return new_vn(3);
}

vect_3 * const_v3(vect_3 * vec, double a, double b, double c)
{
   return 0;
}

matr_n * new_mh()
{
   matr_n * matr;
   matr = (matr_n *) malloc(sizeof(matr_n));
   if (!matr)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   matr->type = BT_WAM_LEGACY_MATH_TYPE_MATR;
   matr->gsl_ptr = (void *) gsl_matrix_calloc(4,4);
   if (!matr->gsl_ptr)
   {
      free(matr);
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   return matr;
}

/* Private functions */
static char * math_sprint(char * buf, struct bt_wam_legacy_math * math)
{
   int j;
   switch (math->type)
   {
      gsl_vector * vec;
      /*gsl_matrix * mat;*/
      case BT_WAM_LEGACY_MATH_TYPE_VECT:
         vec = (gsl_vector *) math->gsl_ptr;
         buf[0] = '<';
         buf[1] = 0;
         for(j=0; j<vec->size; j++)
            sprintf(buf+strlen(buf),"%8.4f,",gsl_vector_get(vec,j));
         buf[strlen(buf)-1] = 0;
         strcat(buf,">");
         break;
      default:
         syslog(LOG_ERR,"%s: Type not yet supported.",__func__);
         sprintf(buf,"Not supported.");
   }
   return buf;
}

static struct bt_wam_legacy_math * math_create_from_gsl(void * gsl_ptr, enum bt_wam_legacy_math_type type)
{
   struct bt_wam_legacy_math * math;
   math = (struct bt_wam_legacy_math *) malloc(sizeof(struct bt_wam_legacy_math));
   if (!math)
      return 0;
   math->type = type;
   math->gsl_ptr = gsl_ptr;
   return math;
}

static int math_destroy(struct bt_wam_legacy_math * math)
{
   free(math);
   return 0;
}


/* #################### HAPTICS STUFF #################### */

int new_bthaptic_scene(bthaptic_scene *bth, int size)
{return 0;}

void init_bulletproofwall(bteffect_bulletproofwall *wall, double Boffset, double K2, double K2offset, double K1, double Bin, double Bout)
{return;}

int bulletproofwall_nf(struct bthaptic_object_struct *obj, btreal depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{return 0;}

int wickedwall_nf(struct bthaptic_object_struct *obj, double depth, vect_n *norm, vect_n *vel, vect_n *acc, vect_n *force)
{return 0;}

void init_state_btg(btgeom_state *bts, double samplerate, double cutoffHz)
{return;}

int init_bx_btg( btgeom_box *box,vect_3 *pt1, vect_3 *pt2, vect_3 *pt3, double thk, double dir1, double dir2,int inside)
{return 0;}


/* #################### OS / THREAD STUFF #################### */

int btrt_mutex_init(btrt_mutex * m)
{
   m->mutex = bt_os_mutex_create(BT_OS_RT);
   if (!m->mutex)
      return -1;
   return 0;
}

int test_and_log(int return_val, const char *str)
{
   if (return_val != 0)
   {
      syslog(LOG_ERR, "%s: %d", str, return_val);
      return return_val;
   }
   else
      return 0;
}

int btrt_thread_create(btrt_thread_struct * thd, char * thdname, int priority, void (* thd_func)(void *), void * data)
{
   /* Create the thread */
   thd->thread = bt_os_thread_create(BT_OS_RT, thdname, priority,
                                     (void (*)(struct bt_os_thread *))thd_func, data);
   if (!thd->thread)
   {
      syslog(LOG_ERR,"%s: Could not create realtime thread '%s'.",__func__,thdname);
      return -1;
   }
   
   /* Put the data in the thread (not actually used yet) */
   
   
   return 0;
}

/* We let people set done to 1, instead of calling stop() correctly */
int btrt_thread_done(btrt_thread_struct * thd)
{
   if (thd->done || bt_os_thread_done(thd->thread))
      return 1;
   else
      return 0;
}

int btrt_thread_exit(btrt_thread_struct * thd)
{
   bt_os_thread_exit(thd->thread);
   return 0;
}


/* #################### SYSTEM STUFF #################### */

int InitializeSystem()
{
   return 0;
}

int CloseSystem()
{
   return 0;
}

int ReadSystemFromConfig(char * conffile, int * buscount)
{
   (*buscount) = 1;
   return 0;
}

int setSafetyLimits(int bus, double a, double b, double c)
{
   return 0;
}

int setProperty(int bus, int id, int prop, int check, long value)
{
   return 0;
}


/* #################### CONTROL STUFF #################### */

int setmode_bts(btstatecontrol *sc, enum scstate mode)
{
   switch (mode)
   {
      case SCMODE_IDLE:
         bt_control_idle(sc->control);
         return 0;
      case SCMODE_POS:
         bt_control_hold(sc->control);
         return 0;
      case SCMODE_TORQUE:
      case SCMODE_TRJ:
         syslog(LOG_ERR,"%s: You can't set the mode to TORQUE or TRJ!",__func__);
   }
   return -1;
}

enum scstate getmode_bts(btstatecontrol *sc)
{
   if (bt_control_is_holding(sc->control))
      return SCMODE_POS;
   else
      return SCMODE_IDLE;
}

int moveparm_bts(btstatecontrol *sc, double vel, double acc)
{
   bt_wam_set_velocity(&sc->my_wam->local->base, vel);
   bt_wam_set_acceleration(&sc->my_wam->local->base, acc);
   return 0;
}

int start_trj_bts(btstatecontrol *sc)
{
   return 0;
}

int stop_trj_bts(btstatecontrol *sc)
{
   return 0;
}

/* We should reach into the refgen for this ... */
enum trjstate get_trjstate_bts(btstatecontrol *sc)
{
   return BTTRAJ_STOPPED;
}

int pause_trj_bts(btstatecontrol *sc,double period)
{
   return 0;
}

int unpause_trj_bts(btstatecontrol *sc,double period)
{
   return 0;
}

vect_n * btt_reset(struct bts_btt * btt)
{
   return 0;
}


/* #################### TRAJECTORY STUFF #################### */

via_trj_array * new_vta(int a, int b)
{
   via_trj_array * vta;
   vta = (via_trj_array *) malloc(sizeof(via_trj_array));
   if (!vta)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   vta->a = a;
   vta->b = b;
   return vta;
}

int register_vta(btstatecontrol *sc, via_trj_array * vta)
{
   return 0;
}


/* #################### ROBOT STUFF #################### */

int apply_tool_force_bot(btrobot * robot, vect_3 * point, vect_3 * force, vect_3 * torque)
{
   return 0;
}


/* #################### WAM STUFF #################### */

wam_struct * OpenWAM(char * conffile, int which)
{
   wam_struct * wam;
   struct bt_wam * bt_wam;
   
   /* Create the legacy structure */
   wam = (wam_struct *) malloc(sizeof(wam_struct));
   if (!wam)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   /* Create the bt_wam, but don't start the loop immediately */
   bt_wam = bt_wam_create_opt(conffile, BT_WAM_OPT_NO_LOOP_START);
   if (!bt_wam)
   {
      free(wam);
      return 0;
   }
   
   if (bt_wam->type == BT_WAM_LOCAL)
      wam->local = (struct bt_wam_local *) bt_wam;
   else
   {
      syslog(LOG_ERR,"%s: We don't support proxy WAMs in legacy mode.",__func__);
      bt_wam_destroy(&wam->local->base);
      free(wam);
      return 0;
   }
   
   /* Add in the name, dof */
   wam->name = wam->local->name;
   wam->dof = wam->local->kin->dof;
   
   /* Add in some vectors */   
   wam->Jpos = math_create_from_gsl(wam->local->jposition,BT_WAM_LEGACY_MATH_TYPE_VECT);
   if (!wam->Jpos)
   {
      bt_wam_destroy(&wam->local->base);
      free(wam);
      return 0;
   }
   wam->Jtrq = math_create_from_gsl(wam->local->jtorque,BT_WAM_LEGACY_MATH_TYPE_VECT);
   if (!wam->Jtrq)
   {
      math_destroy(wam->Jpos);
      bt_wam_destroy(&wam->local->base);
      free(wam);
      return 0;
   }
   wam->Gtrq = new_vn(len_vn(wam->Jpos));
   if (!wam->Gtrq)
   {
      math_destroy(wam->Jpos);
      math_destroy(wam->Jtrq);
      bt_wam_destroy(&wam->local->base);
      free(wam);
      return 0;
   }  
   wam->torq_limit = new_vn(len_vn(wam->Jpos));
   if (!wam->torq_limit)
   {
      destroy_vn(&wam->Gtrq);
      math_destroy(wam->Jpos);
      math_destroy(wam->Jtrq);
      bt_wam_destroy(&wam->local->base);
      free(wam);
      return 0;
   }  
   wam->Cpos = math_create_from_gsl(wam->local->cposition,BT_WAM_LEGACY_MATH_TYPE_VECT);
   if (!wam->Cpos)
   {
      destroy_vn(&wam->torq_limit);
      destroy_vn(&wam->Gtrq);
      math_destroy(wam->Jpos);
      math_destroy(wam->Jtrq);
      bt_wam_destroy(&wam->local->base);
      free(wam);
      return 0;
   }
   wam->HMpos = math_create_from_gsl(wam->local->kin->tool->trans_to_world,BT_WAM_LEGACY_MATH_TYPE_MATR);
   if (!wam->HMpos)
   {
      destroy_vn(&wam->torq_limit);
      destroy_vn(&wam->Gtrq);
      math_destroy(wam->Cpos);
      math_destroy(wam->Jpos);
      math_destroy(wam->Jtrq);
      bt_wam_destroy(&wam->local->base);
      free(wam);
      return 0;
   }
   
   /* Add in the controllers */
   wam->Jsc.control = (struct bt_control *) wam->local->con_joint_legacy;
   wam->Jsc.my_wam = wam;
   wam->Jsc.btt.reset = &btt_reset;
   wam->Csc.control = (struct bt_control *) wam->local->con_cartesian_xyz;
   wam->Csc.my_wam = wam;
   wam->Csc.btt.reset = &btt_reset;
   
   /* Some things are unimplemented */
   wam->Cforce = 0;
   wam->Ctrq = 0;
   wam->Cpoint = 0;
   /*wam->robot;*/
   
   return wam;
}

void WAMControlThread(struct bt_os_thread * thd)
{
   wam_struct * wam;
   wam = thd->data;
   
   bt_wam_loop_start(&wam->local->base);
   
   while (!bt_os_thread_done(thd)){
      bt_os_usleep(10000);
   }
   
   bt_wam_loop_stop(&wam->local->base);
   
   bt_os_thread_exit(thd);
}

int registerWAMcallback(wam_struct * wam, int (* thd_func)(wam_struct *))
{
   return 0;

}

int MoveSetup(wam_struct * wam, double vel, double acc)
{
   bt_wam_set_velocity(&wam->local->base, vel);
   bt_wam_set_acceleration(&wam->local->base, acc);
   return 0;
}

int MoveWAM(wam_struct * wam, vect_n * dest)
{
   return 0;
}

int SetGravityComp(wam_struct * wam, double gcompval)
{
   if (gcompval == 0.0)
   {
      bt_wam_setgcomp(&wam->local->base, 0);
      return 0;
   }
   else if (gcompval == 1.0)
   {
      bt_wam_setgcomp(&wam->local->base, 1);
      return 0;
   }
   return -1;
}


/* #################### SERIAL STUFF #################### */

int serialOpen(PORT * port, char * device)
{
   return 0;
}

int serialSetBaud(PORT * port, int rate)
{
   return 0;
}




