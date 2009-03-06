/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... can_peakesd.c
 *  Author ............. Brian Zenowich
 *                       Traveler Hauptman
 *                       Lauren White
 *                       Christopher Dellin
 *  Creation Date ...... 24 Mar 2003
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2003-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *    This file must be linked against a closed-source proprietary driver
 *    library (libntcan) from esd electronics (http://esd-electronics.com).
 *
 *  REVISION HISTORY:
 *    2003 Mar 24 - BZ
 *      File created & documented.
 *    2004 Dec 16 - BZ, TH
 *      Initial port to linux + RTAI
 *    2007 Aug 6 - LW, BZ
 *      Change code to fit Xenomai, ISA Bus
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *                                                                          *
 * ======================================================================== */


/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#if 0
# ifdef S_SPLINT_S
#  include <err.h>
# endif
#endif

#include <stdio.h>
#include <errno.h>

/*#include <pthread.h>*/
#include <syslog.h>
#include <linux/version.h>
#include <signal.h>

#if 0
#ifdef XENOMAI
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#else
#include <inttypes.h>
#endif
#endif

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)
# include <libpcan.h>
#endif

#ifdef CANTYPE_ESD
# include "ntcan.h"
#endif

#include "can.h"
#include "os.h"


/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (50)
#define RX_TIMEOUT          (50)

#define MAX_BUS             (4)

/*#define HW_ISA_SJA          (9) // use this also for PC/104
//#define HW_PCI              (10) // PCI carries always SJA1000 chips
//#define PCI                 (1)
//#define ISA                 (0)*/


#define DEBUG(x)

/*==============================*
 * PRIVATE MACRO definitions    *
 *==============================*/
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

#define Border(Value,Min,Max)  (Value<Min)?Min:((Value>Max)?Max:Value)

/*==============================*
 * PRIVATE typedefs and structs *
 *==============================*/
#ifdef CANTYPE_ESD
typedef unsigned long DWORD;
#endif
#define MAX_FILTERS (3)

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/

struct can_device
{
   HANDLE handle;
   bt_os_mutex * mutex;
   int iterator;
};

static int accept[MAX_FILTERS];
static int mask[MAX_FILTERS];

static int max_property;
void can_set_max_property(int prop)
{
   max_property = prop;
}

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
static int compile(int property, long longVal, unsigned char *data, int *dataLen);
static int parseMessage(int id, int len, unsigned char *messageData, int *node, int *property, int *ispackedpos, long *value);
static int canReadMsg(struct can_device * dev, int *id, int *len, unsigned char *data, int blocking);
static int canSendMsg(struct can_device * dev, int id, char len, unsigned char *data, int blocking);

/*==============================*
 * Functions                    *
 *==============================*/
#ifdef CANTYPE_ESD
static void allowMessage(struct can_device * dev, int id, int mask)
{
#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)
   /*Allows all messages*/
   CAN_ResetFilter(dev->handle);

#else

   int i;
   for(i = 0; i < 2048; i++)
      if((i & ~mask) == id)
         canIdAdd(dev->handle, i);
#endif
}
#endif




struct can_device * can_create(int port)
{
   long  retvalue;
#ifdef CANTYPE_PEAKISA
   long  pPort;
   int   pIrq;
#endif
   /*int   i;*/

   /*btrt_mutex_create(&commMutex);
   //btrt_mutex_init(&commMutex[bus]);*/
   struct can_device * dev;
   dev = (struct can_device *) malloc(sizeof(struct can_device));
    
   dev->mutex = bt_os_mutex_create(BT_OS_RT);
   
   /*assign ports and irqs to buses *needs to be updated to read ports from cat /proc/pcan/*/
#ifdef CANTYPE_PEAKISA
   if(port == 0)
   {
      pPort = 0x300;
      pIrq = 7;
   }
   else if (port == 1)
   {
      pPort = 0x320;
      pIrq = 5;
   }
   else
   {
      syslog(LOG_ERR, "initCAN: incorrect bus number, cannot open port %d", port);
      return NULL;
   }

   dev->handle = CAN_Open(HW_ISA_SJA, pPort, pIrq);
   if (!dev->handle)
   {
      syslog(LOG_ERR, "initCAN(): CAN_Open(): cannot open device with");
      syslog(LOG_ERR, "type=isa, port=%ld, irq=%d", pPort, pIrq);
      bt_os_mutex_destroy(dev->mutex);
      free(dev);
      return NULL;
   }
#endif

#ifdef CANTYPE_PEAKPCI
   dev->handle = CAN_Open(HW_PCI, (port + 1));
   if (dev->handle)
   {
      syslog(LOG_ERR, "initCAN(): CAN_Open(): cannot open device with");
      syslog(LOG_ERR, "type=pci, port=%d", port);
      bt_os_mutex_destroy(dev->mutex);
      free(dev);
      return NULL;
   }
#endif
      
#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)

   /* Clear Status */
   CAN_Status(dev->handle);

   retvalue = CAN_Init(dev->handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
   if (retvalue)
   {
      syslog(LOG_ERR, "initCAN(): CAN_Init() failed with %d", errno);
      bt_os_mutex_destroy(dev->mutex);
      free(dev);
      return NULL;
   }
   
   CAN_ResetFilter(dev->handle);
   CAN_MsgFilter(dev->handle, 0x0000, 0x053F, MSGTYPE_STANDARD);
   
#endif /* CANTYPE_PEAK */

#ifdef CANTYPE_ESD
   /*Opening can for esd.*/
   retvalue = canOpen(port, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, TX_TIMEOUT, RX_TIMEOUT, &(dev->handle));
   /*retvalue = canOpen(bus, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, TX_TIMEOUT, RX_TIMEOUT, &canDev[bus]);*/
   if(retvalue != NTCAN_SUCCESS)
   {
      syslog(LOG_ERR, "initCAN(): canOpen() failed with error %ld", retvalue);
      bt_os_mutex_destroy(dev->mutex);
      free(dev);
      return NULL;
   }

   retvalue = canSetBaudrate(dev->handle, 0); /* 1 = 1Mbps, 2 = 500kbps, 3 = 250kbps*/
   if(retvalue != 0)
   {
      syslog(LOG_ERR, "initCAN(): canSetBaudrate() failed with error %ld", retvalue);
      bt_os_mutex_destroy(dev->mutex);
      free(dev);
      return NULL;
   }
   
   allowMessage(dev, 0x0000, 0x03E0); /* Messages sent directly to host*/
   allowMessage(dev, 0x0403, 0x03E0); /* Group 3 messages*/
   allowMessage(dev, 0x0406, 0x03E0); /* Group 6 messages*/
#endif
#if 0
   /* Intialize filter/mask */
   for(i = 0; i < MAX_FILTERS; i++){
      filter[i] = 0;
      mask[i] = 0;
   }
#endif

   /* Mask 3E0: 0000 0011 1110 0000*/
   accept[0] = 0x0000; mask[0] = 0x03E0;
   accept[1] = 0x0403; mask[1] = 0x03E0;
   accept[2] = 0x0406; mask[2] = 0x03E0;
   /*allowMessage(bus, 0x0000, 0x03E0); // Messages sent directly to host
   //allowMessage(bus, 0x0403, 0x03E0); // Group 3 messages
   //allowMessage(bus, 0x0406, 0x03E0); // Group 6 messages*/

   /* Set minimum required parameter values
    * WHY DO WE DO THIS? */
   /*
   VERS = 0;
   STAT = 5;
   PROP_END = 10;
   */
   max_property = 10;

   return dev;
}

void can_destroy(struct can_device * dev)
{
#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)
   CAN_Close(dev->handle);
#else
   canClose(dev->handle);
#endif
   bt_os_mutex_destroy(dev->mutex);
   free(dev);
}


int canReadMsg(struct can_device * dev, int *id, int *len, unsigned char *data, int blocking)
{
#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)
   TPCANRdMsg  msg;
   int       pendread;
   int       pendwrite;
#else
   CMSG    msg;
#endif

   long     retvalue=0;
   long      msgCt = 1;
   int       i;
   /*int      filterOK = 0;*/

#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)

   bt_os_rt_set_mode_hard();
   /*retvalue = rt_task_set_mode(0, T_PRIMARY, NULL);*/
   if(blocking)
   {/*attempt to read till there is a message available*/
      /*while(!filterOK){*/
         retvalue = LINUX_CAN_Read(dev->handle, &msg);
         /* Apply private acceptance filter 
         for(i = 0; i < MAX_FILTERS; i++){
            if((msg.Msg.ID & ~mask[i]) == accept[i]){
               filterOK = 1;
            }
         }*/
      /*}*/
   }
   else
   {/*check if a message is pending, if not wait for a period and try again and return*/

      retvalue = LINUX_CAN_Extended_Status(dev->handle, &pendread, &pendwrite);
      if(pendread)
      {
         retvalue = LINUX_CAN_Read(dev->handle, &msg);
      }
      else
      {
         bt_os_usleep(1000);
         retvalue = LINUX_CAN_Extended_Status(dev->handle, &pendread, &pendwrite);
         if(pendread)
            retvalue = LINUX_CAN_Read(dev->handle, &msg);
         else
            return(1);/*returned empty*/
      }
   }
   if(retvalue) /*if there is a error*/
   {
      syslog(LOG_ERR, "canReadMsg(): canRead error: %ld", retvalue);
      return(2);
   }
   if(msgCt == 1)
   {
      *id = msg.Msg.ID;
      *len = msg.Msg.LEN;
      for(i = 0; i < msg.Msg.LEN; i++)
         data[i] = msg.Msg.DATA[i];

      return(0);
   }
#else
   if(blocking)
   {
      /*while(!filterOK){*/
         retvalue = canRead(dev->handle, &msg, &msgCt, NULL);
         /* Apply private acceptance filter 
         for(i = 0; i < MAX_FILTERS; i++){
            if((msg.id & ~mask[i]) == accept[i]){
               filterOK = 1;
            }
         }*/
      /*}*/
   }
   else
   {
      retvalue = canTake(dev->handle, &msg, &msgCt);
   }
   if(retvalue != NTCAN_SUCCESS)
   {
      /*syslog(LOG_ERR, "canReadMsg(): canRead/canTake error: %ld", retvalue);*/
      if(retvalue == NTCAN_RX_TIMEOUT)
         return(1);
      else
         return(2);
   }
   if(msgCt == 1) {
      *id = msg.id;
      *len = msg.len;
      for(i = 0; i < msg.len; i++)
         data[i] = msg.data[i];

      return(0);
   }
#endif
   
   return(1); /* No message received, return err*/
}




int canSendMsg(struct can_device * dev, int id, char len, unsigned char *data, int blocking)
{

#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)
   TPCANMsg  msg;
   int       pendread;
   int       pendwrite=1;
#else
   CMSG    msg;
   long      msgCt = 1;
#endif

   DWORD     retvalue;
   int       i;

#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)

   msg.ID = id;
   msg.MSGTYPE = MSGTYPE_STANDARD;
   msg.LEN = len & 0x0F;
   for(i = 0; i < len; i++)
      msg.DATA[i] = data[i];

   /*make sure that write is in primary mode
   //retvalue = rt_task_set_mode(0, T_PRIMARY, NULL);*/
   bt_os_rt_set_mode_hard();
   if(blocking)
   {
      retvalue = CAN_Write(dev->handle, &msg);
   }
   else
   {/*non-blocking, check to see if bus is full or sending errors, if not send, else return*/
      retvalue = LINUX_CAN_Extended_Status(dev->handle, &pendread, &pendwrite);
      if (retvalue != CAN_ERR_OK)
      {
         syslog(LOG_ERR, "canSendMsg(): error while trying to get status");
      }
      else
      {
         retvalue = CAN_Write(dev->handle, &msg);
      }
   }
   if(retvalue)
   {
      syslog(LOG_ERR, "canSendMsg(): canSend error: %d", retvalue);
      return(1);
   }

#else

   msg.id = id;
   msg.len = len & 0x0F;
   for(i = 0; i < len; i++)
      msg.data[i] = data[i];
   
   if(blocking)
   {
      retvalue = canWrite(dev->handle, &msg, &msgCt, NULL);
   }
   else
   {
      retvalue = canSend(dev->handle, &msg, &msgCt);
   }

   if(retvalue != NTCAN_SUCCESS)
   {
      syslog(LOG_ERR, "canSendMsg(): canWrite/Send() failed with error %lu", retvalue);
      return 1;
   }
#endif
   
   return 0;
}


int can_clearmsg(struct can_device * dev)
{
#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)
   /*TPCANRdMsg  msg;*/
   DWORD       retvalue;
   int         pendread=1;
   int         pendwrite;
#endif
   unsigned char data[8];
   int id, len;
   
   
#if defined(CANTYPE_PEAKISA) || defined(CANTYPE_PEAKPCI)

   retvalue = LINUX_CAN_Extended_Status(dev->handle, &pendread, &pendwrite);

   while(pendread!=0)
   {
      retvalue =  canReadMsg(dev, &id, &len, data, 1);
      /*retvalue = LINUX_CAN_Read(canDev[bus], &msg);*/
      retvalue = LINUX_CAN_Extended_Status(dev->handle, &pendread, &pendwrite);
      
      /*syslog(LOG_ERR, "Cleared unexpected message from CANbus: ID[%4x] LEN[%d] DATA[%2x %2x %2x %2x %2x %2x %2x %2x]",
      //   id, len, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
      //usleep(1);*/
   }

#else
   /*find a better way of clearing the bus*/
   while(!canReadMsg(dev, &id, &len, data, FALSE))
   {
      syslog(LOG_ERR, "Cleared unexpected message from CANbus");
      bt_os_usleep(1);
   }
#endif
   
   return 0;
}





















/* int setTorques(struct can_device * dev, int group, int *values)
 * w/ data[0] = TORQ | 0x80; */
int can_set_torques(struct can_device * dev, int group, int *values, int torque_prop)
{
   unsigned char   data[8];
   int             err;
   int             i;

   /* Bound the torque commands */
   for (i = 0; i < 4; i++)
   {
      values[i] = Border(values[i], -8191, 8191);
   }

   /* Special value-packing compilation: Packs (4) 14-bit values into 8 bytes */
   /*     0        1        2        3        4        5        6        7    */
   /* ATPPPPPP AAAAAAaa aaaaaaBB BBBBbbbb bbbbCCCC CCcccccc ccDDDDDD dddddddd */

   data[0] = torque_prop | 0x80; /* Set the "Set" bit */
   data[1] = (unsigned char)(( values[0] >> 6) & 0x00FF);
   data[2] = (unsigned char)(((values[0] << 2) & 0x00FC) | ((values[1] >> 12) & 0x0003) );
   data[3] = (unsigned char)(( values[1] >> 4) & 0x00FF);
   data[4] = (unsigned char)(((values[1] << 4) & 0x00F0) | ((values[2] >> 10) & 0x000F) );
   data[5] = (unsigned char)(( values[2] >> 2) & 0x00FF);
   data[6] = (unsigned char)(((values[2] << 6) & 0x00C0) | ((values[3] >> 8) & 0x003F) );
   data[7] = (unsigned char)( values[3] & 0x00FF);

   /* Send the data*/
   bt_os_mutex_lock(dev->mutex);
   err = canSendMsg(dev, GROUPID(group), 8, data, TRUE);
   bt_os_mutex_unlock(dev->mutex);
   
   return 0;
}



/* int getPositions(struct can_device * dev, int group, int howMany, long *pos)
 * w/ data[0] = (unsigned char)AP; */
int can_get_packed(struct can_device * dev, int group, int howMany, long * data, int prop)
{
   int             err;
   unsigned char   packet[8];
   int             len;
   int             msgID;
   int             id;
   int             in_property;   
   long            reply;
   /*float alltime;*/

   /* Compile the packet*/
   packet[0] = (unsigned char)prop;

   bt_os_mutex_lock(dev->mutex);

   /* Send the packet*/

   err = canSendMsg(dev, GROUPID(group), 1, packet, TRUE);
   /*howMany = 2; // xxx Remove me*/

   /* Wait for each reply
   //time1 = btrt_get_time();*/
   while(howMany)
   {
      err = canReadMsg(dev, &msgID, &len, packet, TRUE);
      /* If no error*/
      if(!err)
      {
         int ispacked;
         /* Parse the reply*/
         err = parseMessage(msgID, len, packet, &id, &in_property, &ispacked, &reply);
         if(ispacked)
         {
            data[id] = reply;
            --howMany;
         }
         else
         {
            syslog(LOG_ERR, "getPositions(): Asked group %d for position, received property %d = %ld from id %d",
                   group, in_property, reply, id);
         }
      }
      else
      {
         /* Timeout or other error*/
         bt_os_mutex_unlock(dev->mutex);
         return(err);
      }

   }
   bt_os_mutex_unlock(dev->mutex);
   return(0);
}

int can_set_property(struct can_device * dev, int id, int property, int verify, long value)
{
   long            response;
   unsigned char   data[8];
   int             len;
   int             err;

   /* Check the property*/
   if(property > max_property)
   {
      syslog(LOG_ERR,"compile(): Invalid property = %d", property);
      return(1);
   }

   /*syslog(LOG_ERR, "About to compile setProperty, property = %d", property);
   // Compile 'set' packet*/
   err = compile(property, value, data, &len);

   /*syslog(LOG_ERR, "After compilation data[0] = %d", data[0]);*/
   data[0] |= 0x80; /* Set the 'Set' bit*/


   /* Send the packet*/
   bt_os_mutex_lock(dev->mutex);
   /*syslog(LOG_ERR,"a");*/
   err = canSendMsg(dev, (id & 0x0400) ? id : NODE2ADDR(id), len, data, TRUE);
   /*syslog(LOG_ERR,"b");*/
   bt_os_mutex_unlock(dev->mutex);

   /* BUG: This will not verify properties from groups of pucks*/
   if(verify)
   {
      /* Get the new value of the property*/
      can_get_property(dev, id, property, &response);

      /* Compare response to value*/
      if(response != value)
         return(1);
   }
   return(0);
}

int can_get_property(struct can_device * dev, int id, int property, long *reply)
{
   int err;
   unsigned char data[8];
   int len_in;
   int id_in;
   int property_in;


   /* Compile the packet*/
   data[0] = (unsigned char)property;

   bt_os_mutex_lock(dev->mutex);

   /* Send the packet*/
   err = canSendMsg(dev, NODE2ADDR(id), 1, data, TRUE);

   /* Wait for 1 reply*/
   err = canReadMsg(dev, &id_in, &len_in, data, TRUE);

   bt_os_mutex_unlock(dev->mutex);

   if(!err)
   {
      /* Parse the reply*/
      int ispackedpos;
      err = parseMessage(id_in, len_in, data, &id_in, &property_in, &ispackedpos, reply);


      /* Check that the id and property match, and that its not a packed position packet */
      if((id == id_in) && (!ispackedpos) && (property == property_in))

         return(0);
      else
      {
         syslog(LOG_ERR, "getProperty(): returned id or property do not match");
         return(1);
      }
   }
   else
   {
      syslog(LOG_ERR, "getProperty(): canReadMsg error = %d", err);
      return(1);
   }
}

/* Iterate through all live pucks */
void can_iterate_start(struct can_device * dev) { dev->iterator = 0; }
int can_iterate_next(struct can_device * dev,
                      int * nextid, int * nextstatus)
{
   int id;
   
   bt_os_rt_set_mode_hard();
   bt_os_mutex_lock(dev->mutex);
   
   for (id=dev->iterator; id<MAX_NODES; id++)
   {
      int ret;
      unsigned char data[8];
      int id_in;
      int len_in;
      int property_in;
      long status_in;
      
      /* Compile the packet*/
      data[0] = 5; /* STAT = 5 */
      
      /* Send the packet*/
      ret = canSendMsg(dev, NODE2ADDR(id), 1, data, TRUE);
      
      /* Wait 1ms*/
      bt_os_usleep(1000);
      
      /* Try to get 1 reply (non-blocking read)*/
      ret = canReadMsg(dev, &id_in, &len_in, data, FALSE);
      
      /* If no error*/
      if(!ret)
      {
         int ispackedpos;
         /* Parse the reply*/
         /* What if there's an error? */
         parseMessage(id_in, len_in, data, &id_in, &property_in, &ispackedpos, &status_in);
         if (status_in >= 0)
         {
            /* We found a live one! */
            bt_os_mutex_unlock(dev->mutex);
            dev->iterator = id + 1;
            *nextid = id;
            *nextstatus = status_in;
            return 1;
         }
#if 0
         /* If this is the first puck found, initialize the property definitions*/
         if(!firstFound)
         {
            firstFound = 1;
            wam_os_mutex_unlock(dev->mutex);;
            wakePuck(bus, id_in); /* Wake this puck*/
            err = getProperty(dev, id_in, 0, &fw_vers); /* Get the firmware version*/
            /*setProperty(bus, id_in, 5, FALSE, 0);*/ /* Reset this puck*/
            /*usleep(500000);*/
            wam_os_mutex_lock(dev->mutex);
            initPropertyDefs(fw_vers);
         }
#endif

      }
   }
   
   bt_os_mutex_unlock(dev->mutex);
   
   /* That's the end! */
   return 0;
}


/** Parse the data payload received from a Barrett Motor Controller.
    Allows selection of the CAN controller.
    
    \return 0 for no error
    \return 1 for <illegal message header> (syslog output is generated)
 
*/
static int parseMessage(
   /* Input */
   int id                      /** The message ID */,
   int len                     /** The data payload length */,
   unsigned char *messageData  /** Pointer to the message data payload */,

   /* Output */
   int *node       /** The controller node ID of the received message */,
   int *property   /** The property this message applies to */,
   int *ispacked,
   long *value     /** The value of the property being processed */)
{
   int i;
   int dataHeader;
   
   *ispacked=0;

   *node = ADDR2NODE(id);
   if (*node == -1)
      syslog(LOG_ERR,"msgID:%x ",id);
   dataHeader = ((messageData[0] >> 6) & 0x0002) | ((id & 0x041F) == 0x0403) | ((id & 0x041F) == 0x0407);
   /*messageData[0] &= 0x7F;*/
   /*syslog(LOG_ERR,"Entering parsemessage");*/
   switch (dataHeader)
   {
   case 3:  /* Data is a packed 22-bit position, acceleration, etc SET */
      *value = 0x00000000;
      *value |= ( (long)messageData[0] << 16) & 0x003F0000;
      *value |= ( (long)messageData[1] << 8 ) & 0x0000FF00;
      *value |= ( (long)messageData[2] ) & 0x000000FF;

      if (*value & 0x00200000) /* If negative */
         *value |= 0xFFC00000; /* Sign-extend */

      *ispacked=1;
      /**property = AP;*/
      /*syslog(LOG_ERR,"Received packed set property: %d from node: %d value:%d",*property,*node,*value);*/
      break;
   case 2:  /* Data is normal, SET */
      *property = messageData[0] & 0x7F;
      /*syslog(LOG_ERR, "Received property: %d", *property);*/
      /* Store the value, second byte of message is zero (for DSP word alignment) */
      *value = 0;
      for (i = 0; i < len - 2; i++)
         *value |= ((unsigned long)messageData[i + 2] << (i * 8))
                   & (0x000000FF << (i * 8));

      if (*value & (1 << ((i*8) - 1)))
         *value |= 0xFFFFFFFF << (i * 8); /* Sign extend the value */

      /*syslog(LOG_ERR, "Received normal set property: %d from node: %d value:%d", *property, *node, *value);*/
      /*syslog(LOG_ERR,"parsemessage after %d",value);*/
      break;
   case 0:  /* Assume firmware request (GET) */
         *property = -(messageData[0] & 0x7F); /* A negative (or zero) property means GET */
      *value = 0;
      /*syslog(LOG_ERR, "Received normal get property: %d from node: %d value:%d", *property, *node, *value);*/
      break;
   default:
         syslog(LOG_ERR, "<Illegal Message Header> %d\n", dataHeader);
      return(1);
   }
   /*if (*property != 8) syslog(LOG_ERR,"Value in parsemessage is: %d",*value);*/
   return (0);

}

/** Convert a property and value into a valid btcan packet.
    Used by getProperty() and setProperty() to build the data payload 
    section of a CAN message based on a given property and value.
    
    \return 0 for success
    \return non-zero, otherwise
   
*/
static int compile(
   int property        /** The property being compiled (use the enumerations in btcan.h) */,
   long longVal        /** The value to set the property to */,
   unsigned char *data /** A pointer to a character buffer in which to build the data payload */,
   int *dataLen        /** A pointer to the total length of the data payload for this packet */)
/*int isForSafety*/    /** A flag indicating whether this packet is destined for the safety circuit or not */
{
   int i;

   /* Insert the property */
   data[0] = property;
   data[1] = 0; /* To align the values for the tater's DSP */

   /* Append the value */
   for (i = 2; i < 6; i++)
   {
      data[i] = (char)(longVal & 0x000000FF);
      longVal >>= 8;
   }

   /* Record the proper data length */
   *dataLen = 6; /*(dataType[property] & 0x0007) + 2;*/

   /*if (i & 0x0003) *dataLen = 3; */ /* 8-bits */
   /*else if (i & 0x000C) *dataLen = 4; */ /* 16-bits */
   /*else if (i & 0x0030) *dataLen = 5; */ /* 24-bits */
   /*else if (i & 0x00C0) *dataLen = 6; */ /* 32-bits */

   return (0);
}

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
 
