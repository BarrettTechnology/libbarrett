/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... bus.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 15 Feb 2003
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2003-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt, renamed from btsystem.h to bus.h
 *                                                                          *
 * ======================================================================== */

/* Note - this library is all buses wth pucks and actuators. */

/** \file sys.h
  \brief Access and use a collection of pucks
  
  btsystem is meant to give an api to a random set of pucks and motors on multiple busses.
  these are abstracted as "actuators"
  
  \internal !ToDo! much of the bus information should be re-written to be dynamically 
  allocated.
  
*/

#ifndef BT_BUS_H
#define BT_BUS_H

#include "os.h"

/* btwam uses libconfig */
#include <libconfig.h>

/* Everything on the same BUS must have the same property defs */

#define SAFETY_PUCK_ID (10)

/* Todo - prefix these names */

/*! bcastGroup */
enum {
   WHOLE_ARM = 0,
   LOWER_ARM = -1,
   UPPER_ARM = -2
};

enum {
   ROLE_TATER = 0,
   ROLE_GIMBALS = 1,
   ROLE_SAFETY = 2,
   ROLE_WRAPTOR = 3
};

/*! Control_mode states */
enum {
   MODE_IDLE = 0,
   MODE_DUTY = 1,
   MODE_TORQUE = 2,
   MODE_PID = 3,
   MODE_VELOCITY = 4,
   MODE_TRAPEZOIDAL = 5
};

enum {
   STATUS_OFFLINE = -1,
   STATUS_RESET = 0,
   STATUS_ERR = 1,
   STATUS_READY = 2
};

enum {
   DEG = 0,        /* 0-360        */
   RAD = 1,        /* 0-6.28       */
   GRAD = 2,       /* 0-400        */
   PERCENT = 3,    /* 0-100        */
   NATIVE =4     /* 0-CTS*RATIO  */
};

enum {
   SAVED_ERR = 7,
   IGNORE_ERR = 8,
   IS_ACTIVE = 9
};

enum {
   BUS_ERROR = -1,
   BUS_OFF = 0,
   BUS_ON = 1
};

enum {
   ERR_NONE = 0,
   ERR_READ_ONLY = 0,
   ERR_OUT_OF_RANGE = 1
};



struct bt_bus_safety_puck
{
   int id;
};

struct bt_bus_puck
{
   int id;
   
   /* Read from the puck each boot: */
   int counts_per_rev;
   double puckI_per_Nm;
   int gid;
   int order; /* PIDX-1 (0,1,2,3) */
   
   /* Last literal values: */
   long int puck_position;
   int puck_acceleration;
   int puck_torque;
   /*int index;
   int zero;*/
   
   /* Converted values, kept up-to-date by integration */
   double position;
   double velocity;
   double acceleration;
   double torque;
   
   /* For differentiation */
   double position_last;
   double velocity_last;

};

/* Forward declaration */
struct bt_bus_properties;

struct bt_bus_group
{
   struct bt_bus_puck * puck[4];
};

enum bt_bus_update_type
{
   bt_bus_UPDATE_POS_ONLY, /* only update position */
   bt_bus_UPDATE_POS_DIFF, /* differentiate to get pos, vel, acc */
   bt_bus_UPDATE_ACCPOS /* alternate acceleration and position */
};

struct bt_bus
{
   int port; /* For now, we only know about CAN.
              * We can do ethernet later. */
   
   /* Note - eventually, this can be a union
    *        of different device types */
   struct can_device * dev;
   
   /* A bus has a number of actuator pucks */
   int num_pucks;
   int pucks_size;
   struct bt_bus_puck ** puck;
   struct bt_bus_safety_puck * safety_puck;
   struct bt_bus_properties * p;
   int first_pos;
   int first_acc;
   
   /* Groups */
   int groups_size;
   struct bt_bus_group ** group;
   
   /* Update type */
   enum bt_bus_update_type update_type;
   int update_count;
   bt_os_rtime update_last;
};

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

struct bt_bus * bt_bus_create( config_setting_t * busconfig, enum bt_bus_update_type update_type );
int bt_bus_destroy( struct bt_bus * bus );

int bt_bus_update( struct bt_bus * bus );
int bt_bus_set_torques( struct bt_bus * bus );

int bt_bus_set_property(struct bt_bus * bus, int id, int property, int verify, long value);
int bt_bus_get_property(struct bt_bus * bus, int id, int property, long *reply);
int bt_bus_can_clearmsg(struct bt_bus * bus);

/* Wooo property definitions! */
struct bt_bus_properties
{
   int VERS;
   int ROLE;
   int SN;
   int ID;
   int ERROR;
   int STAT;
   int ADDR;
   int VALUE;
   int MODE;
   int D;
   int TORQ;
   int MD;
   int V;
   int B;
   int P;
   int P2;
   int E;
   int E2;
   int MT;
   int MV;
   int MCV;
   int MOV;
   int MOFST;
   int IOFST;
   int PTEMP;
   int UPSECS;
   int OD;
   int MDS;
   int AP;
   int AP2;
   int MECH;
   int MECH2;
   int CTS;
   int CTS2;
   int DP;
   int DP2;
   int OT;
   int OT2;
   int CT;
   int CT2;
   int BAUD;
   int TEMP;
   int OTEMP;
   int _LOCK;
   int DIG0;
   int DIG1;
   int ANA0;
   int ANA1;
   int THERM;
   int VBUS;
   int IMOTOR;
   int VLOGIC;
   int ILOGIC;
   int GRPA;
   int GRPB;
   int GRPC;
   int PIDX;
   int ZERO;
   int SG;
   int HSG;
   int LSG;
   int _DS;
   int IVEL;
   int IOFF;
   int MPE;
   int EN;
   int TSTOP;
   int KP;
   int KD;
   int KI;
   int SAMPLE;
   int ACCEL;
   int TENSION;
   int UNITS;
   int RATIO;
   int LOG;
   int DUMP;
   int LOG1;
   int LOG2;
   int LOG3;
   int LOG4;
   int GAIN1;
   int GAIN2;
   int GAIN3;
   int OFFSET1;
   int OFFSET2;
   int OFFSET3;
   int PEN;
   int SAFE;
   int SAVE;
   int LOAD;
   int DEF;
   int VL1;
   int VL2;
   int TL1;
   int TL2;
   int VOLTL1;
   int VOLTL2;
   int VOLTH1;
   int VOLTH2;
   int MAXPWR;
   int PWR;
   int IFAULT;
   int IKP;
   int IKI;
   int IKCOR;
   int VNOM;
   int TENST;
   int TENSO;
   int JIDX;
   int IPNM;
   int HALLS;
   int HALLH;
   int HALLH2;
   int POLES;
   int ECMAX;
   int ECMIN;
   int ISQ;
   int TETAE;
   int FIND;
   int LCV;
   int LCVC;
   int LFV;
   int LFS;
   int LFAP;
   int LFDP;
   int LFT;
   int VALUE32;
   int PROP_END;

   int LOCK;
   int FET0;
   int FET1;
   int CMD;
   int X0;
   int X1;
   int X2;
   int X3;
   int X4;
   int X5;
   int X6;
   int X7;
   int COMMON_END;
   int SAFETY_END;
   int T;
   int M;
   int M2;
   int IOFF2;
   int HOLD;
   int TIE;
   int LFLAGS;
   int LCTC;
};

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* BT_BUS_H */
 

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2003-2008 Barrett Technology, Inc.            *
 *                        625 Mount Auburn St                           *
 *                    Cambridge, MA  02138,  USA                        *
 *                                                                      *
 *                        All rights reserved.                          *
 *                                                                      *
 *  ******************************************************************  *
 *                            DISCLAIMER                                *
 *                                                                      *
 *  This software and related documentation are provided to you on      *
 *  an as is basis and without warranty of any kind.  No warranties,    *
 *  express or implied, including, without limitation, any warranties   *
 *  of merchantability or fitness for a particular purpose are being    *
 *  provided by Barrett Technology, Inc.  In no event shall Barrett     *
 *  Technology, Inc. be liable for any lost development expenses, lost  *
 *  lost profits, or any incidental, special, or consequential damage.  *
 *======================================================================*/
 
