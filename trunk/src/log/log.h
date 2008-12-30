/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... log.h
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2005 Feb 18
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2005-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *    2005 Nov 01 - TH
 *      Checked
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *
 * ======================================================================== */

/** \file log.h
    \brief Realtime data logging functionality

\section tmp Realtime data logging:

The btlogger object and member functions implement a double buffer data logger.
The double buffer mechanism allows data to be recorded at high speed into memory 
while writes to disk are done as efficient block operations in a low priority 
thread.


 - PrepDL(),AddDataDL(), and InitDL() are the setup functions.
 - DLon(), evalDL(), TriggerDL(), and DLoff() are the operation functions.
 - CloseDL() and DecodeDL() Are the shutdown and conversion functions.

 Typical usage is shown in the following pseudo code

\code
btlogger log;
double a,b,c[3];
void main()
{  
  //allocate fields
  PrepDL(&log,3);
  AddDataDL(&log,&a,sizeof(double),BTLOG_DOUBLE,"Arclength");
  AddDataDL(&log,&b,sizeof(double),BTLOG_DOUBLE,"Normal");
  AddDataDL(&log,c,sizeof(double)*3,BTLOG_DOUBLE,"Position");
  //initialize buffers
  InitDL(&log,1000,"logfile.dat");
  
  DLon(&log);
  while(!done){
    evalDL(&log);
  }
  DLoff(&log);
  CloseDL(&log);bt_log_addfield( t->log, gsl_vector_ptr(cur_position,i), BT_LOG_DOUBLE, "pos" );
}
void loop()
{
  while(1){
    a = a + b;
    c[2] = b/2;
    TriggerDL(&log);
  }
}
\endcode

*/
#ifndef BT_LOG_H
#define BT_LOG_H

#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

/* Note - one block is like a line, with many fields */

/* Field Types */
enum bt_log_fieldtype {
  BT_LOG_INT = 0,
  BT_LOG_LONG,
  BT_LOG_LONGLONG,
  BT_LOG_ULONGLONG,
  BT_LOG_DOUBLE
};


/** btlogger helper structure
  This structure holds info on each piece of data that the user wants to log.
*/
struct bt_log_data_info
{
  size_t size; /*!< size of the data being recorded. = sizeof(datatype)*arrayLength*/
  int type; /*!< 0 = int, 1 = long, 2 = double 3 = long long 4 = btreal*/
  void * data; /*!< pointer to the data start*/
  char name[50]; /*!< null terminated string describing the data. No ','s allowed*/
};

/* Is one of the buffers full? */
enum bt_log_full {
   BT_LOG_FULL_NONE,
   BT_LOG_FULL_A,
   BT_LOG_FULL_B
};

/** Data logging object

btlogger is used for buffering data in a high speed thread and writing the data
to disk in a low speed thread. btlogger uses two buffers. It records data to one 
while the other is being written. The buffers need to be big enough that a buffer 
is completely written before the other is filled up. 

Terminology:
 - Field: One piece of data. Per field info is stored in a btdata_info structure
 - Record: One set of Fields. This represents all the data that needs to be recorded.
 
Function definitions are in btlogger.h.
\internal
\bug Add checks to make sure buffer size is large enough.
*/
struct bt_log
{
  struct bt_log_data_info * data; /*!< list of data information*/
  size_t block_size; /*!< Total size of one block of data*/
  int num_fields_max; /*!< Total number of data_info structures allocated.*/
  int num_fields; /*!< Number of data_info pieces*/
  int initialized;
  
  int num_blocks;
  int buf_block_idx;
  char * buf; /*!< Pointer to current buffer*/
  char * buf_A; /*!< First data buffer*/
  char * buf_B;  /*!< Second data buffer*/
  enum bt_log_full full;
  
  int chunk_idx;
  FILE * file;
};

struct bt_log_read
{
  struct bt_log_data_info * data; /*!< list of data information*/
  size_t block_size; /*!< Total size of one block of data*/
  int num_fields_max; /*!< Total number of data_info structures allocated.*/
  int num_fields; /*!< Number of data_info pieces*/
  int initialized;
  
  FILE * file;
  char * line;
  size_t line_length;
  int blocks_read;
};



/* public functions */
struct bt_log * bt_log_create( unsigned int num_fields );
int bt_log_addfield( struct bt_log * log, void * data, int num, enum bt_log_fieldtype type, char * name);
int bt_log_init( struct bt_log * log, int num_blocks, char * filename);
int bt_log_destroy( struct bt_log * log );

int bt_log_trigger( struct bt_log * log );
int bt_log_flush( struct bt_log * log );

int bt_log_decode( char * infile, char * outfile, int header, int octave);


/* For reading a CSV file */

struct bt_log_read * bt_log_read_create( unsigned int num_fields );
int bt_log_read_addfield( struct bt_log_read * logread, void * data, int num, enum bt_log_fieldtype type);
int bt_log_read_init( struct bt_log_read * logread, char * filename, int header);
int bt_log_read_destroy( struct bt_log_read * logread );

int bt_log_read_get( struct bt_log_read * logread, int * block_num_ptr );



#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* BT_LOG_H */

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
 
