/** Definition of bt_bus, a library for communicating with a set of
 *  Barrett Puck(TM) motor controllers on a CAN bus.
 *
 * \file bus.h
 * \author Traveler Hauptman
 * \author Brian Zenowich
 * \author Sam Clanton
 * \author Christopher Dellin
 * \date 2003-2009
 */

/* Copyright 2003, 2004, 2005, 2006, 2007, 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

/** \file bus.h
 *
 * \section sec_intro Introduction
 *
 * A bt_bus is a set of Barrett Puck(TM) motor controllers on a single CAN
 * bus.  A bus can be created in one of many update modes, which allows an
 * update to fetch positions or positions/accelerations for Pucks that
 * support it (presently, no Pucks support this feature).
 *
 * Once the bus is created, the functions defined in this file can be used
 * to update the positions/velocities/accelerations, send motor torques,
 * and set/get properties.
 */

#ifndef BT_BUS_H
#define BT_BUS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <libconfig.h>
   
#include "../os/os.h"


/** CAN Broadcast Groups */
enum bt_broadcast_groups
{
   WHOLE_BUS_GRP = 0,  // everything but the safety puck

   WAM_GRP = 4,  // the whole WAM (pucks 1-7)
   LOWER_WAM_GRP = 1,  // a packed-torque group (pucks 1-4)
   UPPER_WAM_GRP = 2,  // a packed-torque group (pucks 5-7)

   HAND_GRP = 5,  // the whole hand (pucks 11-14)

   // When responding to position requests, pucks send to group 3 so the safety puck can listen.
   POSITION_FEEDBACK_GRP = 3,
   // When responding to non-position requests, pucks send to group 6.
   OTHER_FEEDBACK_GRP = 6
};

/** Puck status values */
enum bt_puck_status
{
   STATUS_OFFLINE = -1,
   STATUS_RESET = 0,
   STATUS_ERR = 1,
   STATUS_READY = 2
};

/** Puck control mode states */
enum bt_control_modes
{
   MODE_IDLE = 0,
   MODE_DUTY = 1,
   MODE_TORQUE = 2,
   MODE_PID = 3,
   MODE_VELOCITY = 4,
   MODE_TRAPEZOIDAL = 5
};

/** Static IDs for particular types of Puck */
enum bt_bus_puck_id
{
   BT_BUS_PUCK_ID_WAMSAFETY = 10,
   BT_BUS_PUCK_ID_FT = 8
};


/** A WAM safety Puck need only have an ID. */
struct bt_bus_safety_puck
{
   int id;
};

/** Data associated with a motor Puck, including static info read from
 *  the puck on initialization and dynamic data such as present position.
 */
struct bt_bus_puck
{
   int id; /**< The Puck's ID */
   
   /** \name Values read from the Puck on bus creation:
    *  \{ */
   int vers;
   int counts_per_rev;
   double puckI_per_Nm;
   int gid;
   int order; /**< PIDX-1 (0,1,2,3) */
   /*  \} */
   
   /** \name Most recent values read from Puck:
    *  \{ */
   long int puck_position;
   int puck_acceleration;
   int puck_torque;
   /*  \} */
   
   /** \name Converted, calculated, or buffered values:
    *  \{ */
   double position;
   double velocity;
   double acceleration;
   double torque;
   /*  \} */
   
   /** \name Variables used for differentiation:
    *  \{ */
   double position_last;
   double velocity_last;
   /* \} */
};


/** A set of four Pucks in a group, each with a distinct order [0-3]. */
struct bt_bus_group
{
   struct bt_bus_puck * puck[4];
};


/** The update type; presently, Pucks do not support retrieving acceleration.
 */
enum bt_bus_update_type
{
   bt_bus_UPDATE_POS_ONLY, /**< only update position */
   bt_bus_UPDATE_POS_DIFF, /**< differentiate to get pos, vel, acc */
   bt_bus_UPDATE_ACCPOS    /**< alternate acceleration and position */
};


/* Forward declaration of the huge bt_bus_properties list */
struct bt_bus_properties;


/** All the data related to a bus of Pucks.
 *
 * A bt_bus has a CAN device, a list of Pucks (and groups of Pucks, etc),
 * and information about the update type.
 */
struct bt_bus
{
   /** \name CAN-specific data:
    *  \{ */
   int port;
   struct bt_bus_can * dev;
   /*  \} */
   
   /** \name Lists and information about the Pucks found on the bus:
    *  \{ */
   int num_pucks;
   int pucks_size;
   struct bt_bus_puck ** puck;
   struct bt_bus_safety_puck * safety_puck;
   struct bt_bus_properties * p;
   int groups_size;              /**< Length of the group array */
   struct bt_bus_group ** group; /**< Indexed array of groups */
   /* \} */
   
   /** \name Information about performing updates:
    *  \{ */
   enum bt_bus_update_type update_type;
   int first_pos; /**< Have we received the first position yet? */
   int first_acc; /**< Have we received the first acceleration yet? */
   int update_count;
   bt_os_rtime update_last; /**< Time of last update */
   /*  \} */
};


/** Create a bt_bus given a configuration and an update type.
 *
 * \param[out] busptr The bt_bus object on success, or 0 on failure
 * \param[in] busconfig Bus configuration, from libconfig
 * \param[in] update_type Update type
 * \retval 0 Success
 */
int bt_bus_create(struct bt_bus ** busptr, config_setting_t * busconfig,
                  enum bt_bus_update_type update_type);


/** Destroy a bt_bus object.
 *
 * This function destroys a bt_bus object created by bt_bus_create().
 *
 * \param[in] bus The bt_bus object to destroy
 * \retval 0 Success
 */
int bt_bus_destroy(struct bt_bus * bus);


/** Update a bt_bus object.
 *
 * This function retrieves an update from the Pucks on the bus.  Depending
 * on the update type specified in bt_bus_create(), this may ask the Pucks
 * for positions or accelerations.  The resulting values will be saved in
 * the bt_bus object.
 *
 * \param[in] bus The bt_bus object to update
 * \retval 0 Success
 */
int bt_bus_update(struct bt_bus * bus);


/** Set torques to Pucks in a bt_bus object.
 *
 * This function send the present values of the Pucks's torque value to all
 * of the motor Pucks on the bus.  This function will use Puck groups to send
 * the torques to up to 4 Pucks at once, preserving CAN bandwidth.
 *
 * \param[in] bus The bt_bus object to set torques to
 * \retval 0 Success
 */
int bt_bus_set_torques(struct bt_bus * bus);


/** Get a property value from a Puck.
 *
 * This function gets a property value from a given Puck.
 *
 * \param[in] bus The bt_bus object to use
 * \param[in] id The ID of the Puck
 * \param[in] property The property to get; se bt_bus_properties for a list
 * \param[out] reply The location to save the value
 * \retval 0 Success
 * \retval 1 The property is beyond the maximum in the properties list
 * \return For other return values, see bt_bus_can_get_property() in
 *         bus_can.h
 */
int bt_bus_get_property(struct bt_bus * bus, int id, int property,
                        long * reply);


/** Set a property value on a Puck.
 *
 * This function sets a property to a given value on a given Puck.  If the
 * verify argument is true, this will also perform a get property request to
 * ensure that the value was changed.
 *
 * \param[in] bus The bt_bus object to use
 * \param[in] id The ID of the Puck
 * \param[in] property The property to set; se bt_bus_properties for a list
 * \param[in] value The property's value to set
 * \retval 0 Success
 * \retval 1 The property is beyond the maximum in the properties list
 * \return For other return values, see bt_bus_can_set_property() in
 *         bus_can.h
 */
int bt_bus_set_property(struct bt_bus * bus, int id, int property, long value);


/** A list of properties available on each Puck on the bus.
 *
 * This structure is presently filled with the correct values as the bt_bus
 * is initialized, based on the version of the first puck found.
 *
 * \note This properties list is presently unique to a bus, instead of
 *       specific to each Puck.  Therefore, if different Pucks have different
 *       versions, things might break.
 */
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
#endif
#endif /* BT_BUS_H */
