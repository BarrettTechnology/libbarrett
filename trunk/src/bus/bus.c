/** Implementation of bt_bus, a library for communicating with a set of
 *  Barrett Puck(TM) motor controllers on a CAN bus.
 *
 * \file bus.c
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

#include <stdlib.h>
#include <syslog.h>
#include <string.h>

#include "bus.h"
#include "bus_can.h"
#include "os.h"

#define MAXPUCKID (20)
#define TWOPI (2*3.141592653589793238462643383)
#define BORDER(Value,Min,Max) \
   ((Value)<(Min))?(Min):(((Value)>(Max))?(Max):(Value))


/** CAN Broadcast Groups */
enum broadcast_groups
{
   WHOLE_ARM = 0,
   LOWER_ARM = -1,
   UPPER_ARM = -2
};

/** Puck status values */
enum puck_status
{
   STATUS_OFFLINE = -1,
   STATUS_RESET = 0,
   STATUS_ERR = 1,
   STATUS_READY = 2
};

/** Puck control mode states */
enum control_modes
{
   MODE_IDLE = 0,
   MODE_DUTY = 1,
   MODE_TORQUE = 2,
   MODE_PID = 3,
   MODE_VELOCITY = 4,
   MODE_TRAPEZOIDAL = 5
};


/** Create a bt_bus_properties list given a firmware version */
static struct bt_bus_properties * prop_defs_create(long firmwareVersion);

/** Destroy a bt_bus_properties list */
static int prop_defs_destroy(struct bt_bus_properties * p);

/** Query and receive a packed CAN packet of Puck accelerations */
static int retrieve_puck_accelerations(struct bt_bus * bus);

/** Query and receive a packed CAN packet of Puck positions */
static int retrieve_puck_positions(struct bt_bus * bus);


/** Define two pointer-array setting functions for setting values in
 *  pointer arrays.
 *
 * A pointer array is an array of points to objects, where the index is
 * meaningful.  For example, the data structure that holds pointers to Pucks
 * in a bt_bus is a pointer array:
 * \code
 * struct bt_bus
 * {
 *    ...
 *    struct bt_bus_puck ** puck;
 *    ...
 * }
 * \endcode
 *
 * The generated function parr_set_puck(bus->puck,&size,6,mypuck) will set
 * element 6 of the puck pointer array to mypuck, increasing the size of the
 * pointer array if needed, and setting any pointer without a value to 0.
 */
#define PARR_SET_FUNC(FNAME,TYPE) \
static int parr_set_ ## FNAME ( TYPE *** rp, int * sz, int i, TYPE * x ) \
{                                                                        \
   if (i >= *sz)                                                         \
   {                                                                     \
      int j;                                                             \
      if (*sz == 0) *rp = (TYPE **) malloc (    (i+1)*sizeof(TYPE *));   \
      else          *rp = (TYPE **) realloc(*rp,(i+1)*sizeof(TYPE *));   \
      if(!*rp) return -1;                                                \
      for (j=*sz; j<i; j++) (*rp)[j] = 0;                                \
      *sz = i+1;                                                         \
   }                                                                     \
   (*rp)[i] = x;                                                         \
   return 0;                                                             \
}
PARR_SET_FUNC(puck,struct bt_bus_puck)
PARR_SET_FUNC(group,struct bt_bus_group)


int bt_bus_create(struct bt_bus ** busptr, config_setting_t * busconfig,
                  enum bt_bus_update_type update_type)
{
   struct bt_bus * bus;

   (*busptr) = 0;
   
   /* Create */
   bus = (struct bt_bus *) malloc(sizeof(struct bt_bus));
   if (!bus)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      (*busptr) = 0;
      return -1;
   }
   
   /* Initialize */
   bus->dev = 0;
   bus->port = 0;
   bus->num_pucks = 0;
   bus->pucks_size = 0;
   bus->puck = 0;
   bus->safety_puck = 0;
   bus->p = 0;
   bus->first_pos = 1;
   bus->first_acc = 1;
   bus->groups_size = 0;
   bus->group = 0;
   bus->update_type = update_type;
   bus->update_count = 0;
   
   /* OK, first parse the config into the bus structure */
   {
      config_setting_t * setting;
      
      /* Bus address */
      setting = config_setting_get_member( busconfig, "port" );
      if (   setting == NULL
          || config_setting_type(setting) != CONFIG_TYPE_INT )
      {
         printf("no port\n");
         bt_bus_destroy(bus);
         return -1;
      }
      bus->port = config_setting_get_int(setting);
   }
   
   /* Initialize CAN on the port */
   bt_bus_can_create(&(bus->dev), bus->port);
   if(!(bus->dev))
   {
      syslog(LOG_ERR, "Could not initialize can bus port %d", bus->port);
      bt_bus_destroy(bus);
      return -1;
   }
   
   /* Wake all the pucks on the bus */
   syslog(LOG_ERR, "Waking all pucks");
   /* Must use '5' for STAT*/
   bt_bus_can_set_property(bus->dev, BT_BUS_CAN_GROUPID(WHOLE_ARM), 5, 0,
                           STATUS_READY);
   bt_os_usleep(300000); /* Wait 300ms for puck to initialize*/
   
   /* Iterate through all the pucks */
   {
      int id;
      int status;
      bt_bus_can_iterate_start(bus->dev);
      while (bt_bus_can_iterate_next(bus->dev,&id,&status))
      {
         /* Initialize proprty definitions */
         if (!(bus->p))
         {
            long fw_vers;
            /* Get the firmware version*/
            bt_bus_can_get_property(bus->dev, id, 0, &fw_vers);
            bus->p = prop_defs_create(fw_vers);
            if (!bus->p)
            {
               syslog(LOG_ERR,
                      "%s: Could not initialize property definitions.",
                      __func__);
               bt_bus_destroy(bus);
               return -1;
            }
         }
         if (id == BT_BUS_PUCK_ID_WAMSAFETY)
         {
            struct bt_bus_safety_puck * puck;
            if (bus->safety_puck)
            {
               syslog(LOG_ERR,"%s: More than one safety puck found!.",
                      __func__);
               bt_bus_destroy(bus);
               return -1;
            }
            if (status != STATUS_READY)
            {
               syslog(LOG_ERR, "%s: The safety module on port %d is not "
                      "functioning properly",__func__,bus->port);
               break;
            }
            /* Make a new safety puck */
            syslog(LOG_ERR,"Found a safety puck!\n");
            puck = (struct bt_bus_safety_puck *)
               malloc(sizeof(struct bt_bus_safety_puck));
            if (!puck)
            {
               syslog(LOG_ERR,"%s: Out of memory.",__func__);
               bt_bus_destroy(bus);
               return -1;
            }
            puck->id = id;
            bus->safety_puck = puck;
         }
         else if (status == STATUS_RESET || status == STATUS_READY)
         {
            long reply;
            struct bt_bus_puck * puck;
            /* Wake it up if it's not ready yet */
            if (status == STATUS_RESET)
            {
               syslog(LOG_ERR, "Waking puck %d", id);
               /*wakePuck(bus->dev, id);*/
               /* Must use '5' for STAT*/
               bt_bus_can_set_property(bus->dev, id, 5, 0, STATUS_READY);
               bt_os_usleep(300000); /* Wait 300ms for puck to initialize*/
            }
            /* Make a new puck */
            puck = (struct bt_bus_puck *) malloc(sizeof(struct bt_bus_puck));
            if (!puck)
            {
               syslog(LOG_ERR,"%s: Out of memory.",__func__);
               bt_bus_destroy(bus);
               return -1;
            }
            puck->id = id;
            /* Make sure the puck is in IDLE mode */
            bt_bus_can_set_property(bus->dev, id, bus->p->MODE, 0,MODE_IDLE);
            bt_os_usleep(200);
            /* Fill the puck structure */
            bt_bus_can_get_property(bus->dev, id, bus->p->VERS, &reply);
            puck->vers = reply;
            bt_bus_can_get_property(bus->dev, id, bus->p->CTS, &reply);
            puck->counts_per_rev = reply;
            bt_bus_can_get_property(bus->dev, id, bus->p->IPNM, &reply);
            puck->puckI_per_Nm = reply; /* Aah, this is a double! */
            bt_bus_can_get_property(bus->dev, id, bus->p->PIDX, &reply);
            puck->order = reply-1;
            bt_bus_can_get_property(bus->dev, id, bus->p->GRPB, &reply);
            puck->gid = reply;
            
            syslog(LOG_ERR,"Puck: ID=%d CTS=%d IPNM=%.2f PIDX=%d GRPB=%d",
                   puck->id, puck->counts_per_rev, puck->puckI_per_Nm,
                   puck->order, puck->gid);
            
            /* Set other things to their defaults */
            puck->puck_position = 0;
            puck->puck_acceleration = 0;
            puck->puck_torque = 0;
            /*puck->index = 0;
            puck->zero = 0;*/
            puck->position = 0.0;
            puck->velocity = 0.0;
            puck->acceleration = 0.0;
            puck->torque = 0.0;
            puck->position_last = 0.0;
            puck->velocity_last = 0.0;
            
            /* Save the puck */
            parr_set_puck(&(bus->puck), &(bus->pucks_size), id, puck);
            bus->num_pucks++;
            
            /* Make a new group if necessary */
            if ( (puck->gid >= bus->groups_size)
                || !(bus->group[puck->gid]) )
            {
               struct bt_bus_group * group;
               group = (struct bt_bus_group *)
                  malloc(sizeof(struct bt_bus_group));
               if (!group)
               {
                  syslog(LOG_ERR,"%s: Out of memory.",__func__);
                  bt_bus_destroy(bus);
                  return -1;
               }
               parr_set_group(&(bus->group), &(bus->groups_size), puck->gid,
                              group );
            }
            
            /* Add the puck to the group */
            bus->group[puck->gid]->puck[puck->order] = puck;
            
            /* Set max torque (eventually) */
         }
      }
   }
   
   /* Did we not get a safety puck? */
   
   /* CPU cycle time (to time things? huh?) */

   (*busptr) = bus;
   return 0;
}


int bt_bus_destroy(struct bt_bus * bus)
{
   int i;
   if (bus->p) prop_defs_destroy(bus->p);
   if (bus->safety_puck) free(bus->safety_puck);
   for (i=0; i<bus->groups_size; i++)
      if (bus->group[i]) free(bus->group[i]);
   if (bus->group) free(bus->group);
   for (i=0; i<bus->pucks_size; i++)
      if (bus->puck[i]) free(bus->puck[i]);
   if (bus->puck) free(bus->puck);
   if (bus->dev) bt_bus_can_destroy(bus->dev);
   free(bus);
   return 0;
}


int bt_bus_update(struct bt_bus * bus)
{
   int i;
   bt_os_rtime rtime;
   struct bt_bus_puck * p;
   double dt;
   
   /* Clear CAN bus of any unwanted messages */
   bt_bus_can_clearmsg( bus->dev );
   
   /* Retrieve the time */
   rtime = bt_os_rt_get_time();
   dt = 1e-9 * (rtime - bus->update_last);

   /* Step the integrators */
   switch (bus->update_type)
   {
      case bt_bus_UPDATE_POS_ONLY:
         retrieve_puck_positions(bus);
         break;
      case bt_bus_UPDATE_POS_DIFF:
         retrieve_puck_positions(bus);
         /* Differentiate to get velocity */
         for (i=0; i<bus->pucks_size; i++) if ((p = bus->puck[i]))
         {
            /*p->velocity_last = p->velocity;*/
            p->velocity = (p->position - p->position_last) / dt;
            /*p->acceleration = (p->velocity - p->velocity_last) / dt;*/
         }
         break;
      case bt_bus_UPDATE_ACCPOS:
         if (bus->update_count % 2)
            retrieve_puck_accelerations(bus);
         else
         {
            retrieve_puck_positions(bus);
            /* Differentiate to get velocity */
            for (i=0; i<bus->pucks_size; i++) if ((p = bus->puck[i]))
            {
               /*p->velocity_last = p->velocity;*/
               p->velocity = (p->position - p->position_last) / dt;
               /*p->acceleration = (p->velocity - p->velocity_last) / dt;*/
            }
         }
         break;
   }
   
   bus->update_last = rtime;
   bus->update_count++;
   return 0;
}


int bt_bus_set_torques(struct bt_bus * bus)
{
   int gid,i;
   int data[4];
   
   /* Convert to puck units */
   for (i=0; i<bus->pucks_size; i++) if (bus->puck[i])
   {
      double tmp;
      tmp = bus->puck[i]->torque * bus->puck[i]->puckI_per_Nm;
      bus->puck[i]->puck_torque = BORDER(tmp, -8191, 8191);
   }
   
   /* Send a setTorques packet for each group */
   for (gid=0; gid<bus->groups_size; gid++) if (bus->group[gid])
   {
      for (i=0; i<4; i++)
      {
         if (bus->group[gid]->puck[i])
            data[i] = bus->group[gid]->puck[i]->puck_torque;
         else
            data[i] = 0;
      }
      bt_bus_can_set_torques(bus->dev, gid, data, bus->p->TORQ);
   }
   
   return 0;
}


int bt_bus_get_property(struct bt_bus * bus, int id, int property,
                        long * reply)
{
   if (property > bus->p->PROP_END)
      return 1;
   return bt_bus_can_get_property(bus->dev, id, property, reply);
}


int bt_bus_set_property(struct bt_bus * bus, int id, int property,
                        int verify, long value)
{
   if (property > bus->p->PROP_END)
      return 1;
   return bt_bus_can_set_property(bus->dev, id, property, verify, value);
}


static int retrieve_puck_positions( struct bt_bus * bus )
{
   int i;
   struct bt_bus_puck * p;
   
   /* Data must be bigger than the max ID (puck id is index) */
   /* Arbitrary for now -- allocate dynamically on startup! */
   long int data[MAXPUCKID];
   
   /* Fill with illegal values */
   for(i = 0; i < 20; i++)
      data[i] = 0x80000000;
   
   /* Perform the btcan broadcast to get positions (group 0) */
   bt_bus_can_get_packed(bus->dev, 0, bus->num_pucks, data, bus->p->AP);
   
   /* Unpack from data into the pucks array */
   for (i=0; i<bus->pucks_size; i++) if ( (p = bus->puck[i]) )
   {
      /* Make sure the puck's data is good */
      if ( !(bus->first_pos) )
      {
         if (data[i] == 0x80000000)
         {
            syslog(LOG_ERR,"puck %d invalid position: %ld\n",i,data[i]);
            continue;
         }
         else if( abs(data[i] - p->puck_position) > 1000)
         {
            syslog(LOG_ERR,"puck %d insane position diff: %ld\n",i,
                   data[i] - p->puck_position);
            continue;
         }
      }
      
      /* Copy the position */
      p->puck_position = data[i];
      
      /* Copy to old, and convert to radians */
      p->position_last = p->position;
      p->position = TWOPI * p->puck_position / p->counts_per_rev;
   }
   
   /* OK, this is no longer the first position */
   bus->first_pos = 0;
   
   return 0;
}


static int retrieve_puck_accelerations( struct bt_bus * bus )
{
   int i;
   struct bt_bus_puck * p;
   
   /* Data must be bigger than the max ID (puck id is index) */
   /* Arbitrary for now -- allocate dynamically on startup! */
   long int data[MAXPUCKID]; 
   
   /* Fill with illegal values */
   for(i = 0; i < 20; i++)
      data[i] = 0x80000000;
   
   /* Update acceleration w/ broadcast */
   /* CHANGE TO 'A' */
   bt_bus_can_get_packed(bus->dev, 0, bus->num_pucks, data, bus->p->MECH);
   
   /* Unpack from data into the pucks array */
   for (i=0; i<bus->pucks_size; i++) if ((p = bus->puck[i]))
   {
      /* Make sure the puck's data is good */
      if ( !(bus->first_acc) )
      {
         if (data[i] == 0x80000000)
         {
            syslog(LOG_ERR,"puck %d invalid acceleration: %ld\n",i,data[i]);
            continue;
         }
#if 0
         else if( abs(data[i] - bus->puck[i]->puck_acceleration) > 100000)
         {
            syslog(LOG_ERR,"puck %d insane acceleration diff: %ld\n",i,
                   data[i] - bus->puck[i]->puck_acceleration);
            continue;
         }
#endif
      }
      
      /* Copy the acceleration */
      p->puck_acceleration = data[i];
      
      /* Convert to radians/sec/sec */
      p->acceleration = (5000.0 * 5000.0) * TWOPI * p->puck_acceleration /
           p->counts_per_rev / (1<<18);
   }
   
   /* OK, this is no longer the first acceleration */
   bus->first_acc = 0;
   
   return 0;
}


static struct bt_bus_properties * prop_defs_create(long firmwareVersion)
{
   struct bt_bus_properties * p;
   int i;
   
   p = (struct bt_bus_properties *) malloc(sizeof(struct bt_bus_properties));
   if (!p)
   {
      syslog(LOG_ERR,"%s: Out of memory.",__func__);
      return 0;
   }
   
   i = 0;
   if(firmwareVersion < 40)
   {
      p->VERS = i++;
      p->ROLE = i++;
      p->SN = i++;
      p->ID = i++;
      p->ERROR = i++;
      p->STAT = i++;
      p->ADDR = i++;
      p->VALUE = i++;
      p->MODE = i++;
      p->D = i++;
      p->TORQ = i++;
      p->P = i++;
      p->V = i++;
      p->E = i++;
      p->B = i++;
      p->MD = i++;
      p->MT = i++;
      p->MV = i++;
      p->MCV = i++;
      p->MOV = i++;
      p->MOFST = i++;
      p->IOFST = i++;
      p->PTEMP = i++;
      p->UPSECS = i++;
      p->OD = i++;
      p->MDS = i++;
      p->AP = i++;
      p->AP2 = i++;
      p->MECH = i++;
      p->MECH2 = i++;
      p->CTS = i++;
      p->CTS2 = i++;
      p->DP = i++;
      p->DP2 = i++;
      p->OT = i++;
      p->OT2 = i++;
      p->CT = i++;
      p->CT2 = i++;
      p->BAUD = i++;
      p->TEMP = i++;
      p->OTEMP = i++;
      p->_LOCK = i++;
      p->DIG0 = i++;
      p->DIG1 = i++;
      p->ANA0 = i++;
      p->ANA1 = i++;
      p->THERM = i++;
      p->VBUS = i++;
      p->IMOTOR = i++;
      p->VLOGIC = i++;
      p->ILOGIC = i++;
      p->GRPA = i++;
      p->GRPB = i++;
      p->GRPC = i++;
      p->PIDX = i++;
      p->ZERO = i++;
      p->SG = i++;
      p->HSG = i++;
      p->LSG = i++;
      p->_DS = i++;
      p->IVEL = i++;
      p->IOFF = i++;
      p->MPE = i++;
      p->EN = i++;
      p->TSTOP = i++;
      p->KP = i++;
      p->KD = i++;
      p->KI = i++;
      p->SAMPLE = i++;
      p->ACCEL = i++;
      p->TENSION = i++;
      p->UNITS = i++;
      p->RATIO = i++;
      p->LOG = i++;
      p->DUMP = i++;
      p->LOG1 = i++;
      p->LOG2 = i++;
      p->LOG3 = i++;
      p->LOG4 = i++;
      p->GAIN1 = i++;
      p->GAIN2 = i++;
      p->GAIN3 = i++;
      p->OFFSET1 = i++;
      p->OFFSET2 = i++;
      p->OFFSET3 = i++;
      p->PEN = i++;
      p->SAFE = i++;
      p->SAVE = i++;
      p->LOAD = i++;
      p->DEF = i++;
      p->VL1 = i++;
      p->VL2 = i++;
      p->TL1 = i++;
      p->TL2 = i++;
      p->VOLTL1 = i++;
      p->VOLTL2 = i++;
      p->VOLTH1 = i++;
      p->VOLTH2 = i++;
      p->MAXPWR = i++;
      p->PWR = i++;
      p->IFAULT = i++;
      p->IKP = i++;
      p->IKI = i++;
      p->IKCOR = i++;
      p->VNOM = i++;
      p->TENST = i++;
      p->TENSO = i++;
      p->JIDX = i++;
      p->IPNM = i++;

      p->PROP_END = i++;

      p->T = p->TORQ;
      p->FET0 = p->B;
      p->FET1 = p->TENSION;
      /*
      p->HALLS = i++;
      p->HALLH = i++;
      p->HALLH2 = i++;
      p->POLES = i++;
      p->ECMAX = i++;
      p->ECMIN = i++;
      p->ISQ = i++;
      p->TETAE = i++;
      p->FIND = i++;
      p->LCV = i++;
      p->LCVC = i++;
      p->LFV = i++;
      p->LFS = i++;
      p->LFAP = i++;
      p->LFDP = i++;
      p->LFT = i++;
      p->VALUE32 = i++;
      */
   }
   else
   {
      /* Common */
      p->VERS = i++;
      p->ROLE = i++; /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
      p->SN = i++;
      p->ID = i++;
      p->ERROR = i++;
      p->STAT = i++;
      p->ADDR = i++;
      p->VALUE = i++;
      p->MODE = i++;
      p->TEMP = i++;
      p->PTEMP = i++;
      p->OTEMP = i++;
      p->BAUD = i++;
      p->_LOCK = i++;
      p->DIG0 = i++;
      p->DIG1 = i++;
      p->FET0 = i++;
      p->FET1 = i++;
      p->ANA0 = i++;
      p->ANA1 = i++;
      p->THERM = i++;
      p->VBUS = i++;
      p->IMOTOR = i++;
      p->VLOGIC = i++;
      p->ILOGIC = i++;
      p->SG = i++;
      p->GRPA = i++;
      p->GRPB = i++;
      p->GRPC = i++;
      p->CMD = i++; /* For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,
                     * HI,IC,IO,TC,TO,C,O,T */
      p->SAVE = i++;
      p->LOAD = i++;
      p->DEF = i++;
      p->FIND = i++;
      p->X0 = i++;
      p->X1 = i++;
      p->X2 = i++;
      p->X3 = i++;
      p->X4 = i++;
      p->X5 = i++;
      p->X6 = i++;
      p->X7 = i++;

      p->COMMON_END = i++;

      /* Safety */
      i = p->COMMON_END;
      p->ZERO = i++;
      p->PEN = i++;
      p->SAFE = i++;
      p->VL1 = i++;
      p->VL2 = i++;
      p->TL1 = i++;
      p->TL2 = i++;
      p->VOLTL1 = i++;
      p->VOLTL2 = i++;
      p->VOLTH1 = i++;
      p->VOLTH2 = i++;
      p->PWR = i++;
      p->MAXPWR = i++;
      p->IFAULT = i++;
      p->VNOM = i++;

      p->SAFETY_END = i++;

      /* Tater */
      i = p->COMMON_END;
      p->T = i++;
      p->MT = i++;
      p->V = i++;
      p->MV = i++;
      p->MCV = i++;
      p->MOV = i++;
      p->P = i++; /* 32-Bit Present Position */
      p->P2 = i++;
      p->DP = i++; /* 32-Bit Default Position */
      p->DP2 = i++;
      p->E = i++; /* 32-Bit Endpoint */
      p->E2 = i++;
      p->OT = i++; /* 32-Bit Open Target */
      p->OT2 = i++;
      p->CT = i++; /* 32-Bit Close Target */
      p->CT2 = i++;
      p->M = i++; /* 32-Bit Move command for CAN*/
      p->M2 = i++;
      p->_DS = i++;
      p->MOFST = i++;
      p->IOFST = i++;
      p->UPSECS = i++;
      p->OD = i++;
      p->MDS = i++;
      p->MECH = i++; /* 32-Bit */
      p->MECH2 = i++;
      p->CTS = i++; /* 32-Bit */
      p->CTS2 = i++;
      p->PIDX = i++;
      p->HSG = i++;
      p->LSG = i++;
      p->IVEL = i++;
      p->IOFF = i++; /* 32-Bit */
      p->IOFF2 = i++;
      p->MPE = i++;
      p->EN = i++;
      p->TSTOP = i++;
      p->KP = i++;
      p->KD = i++;
      p->KI = i++;
      p->ACCEL = i++;
      p->TENST = i++;
      p->TENSO = i++;
      p->JIDX = i++;
      p->IPNM = i++;
      p->HALLS = i++;
      p->HALLH = i++; /* 32-Bit */
      p->HALLH2 = i++;
      p->POLES = i++;
      p->IKP = i++;
      p->IKI = i++;
      p->IKCOR = i++;
      p->HOLD = i++;
      p->TIE = i++;
      p->ECMAX = i++;
      p->ECMIN = i++;
      p->LFLAGS = i++;
      p->LCTC = i++;
      p->LCVC = i++;

      p->PROP_END = i++;

      p->AP = p->P; /* Handle parameter name change*/
      p->TENSION = p->FET1;
   }
   
   return p;
}


static int prop_defs_destroy(struct bt_bus_properties * p)
{
   free(p);
   return 0;
}
