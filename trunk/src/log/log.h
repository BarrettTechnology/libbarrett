/** Definition of bt_log, a double-buffered data logger.
 *
 * \file log.h
 * \author Traveler Hauptman
 * \author Brian Zenowich
 * \author Christopher Dellin
 * \date 2005-2009
 */

/* Copyright 2005, 2006, 2007, 2008, 2009
 *           Barrett Technology <support@barrett.com> */

/*
 * This file is part of libbarrett.
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

/** \file log.h
 * \section sec_intro Introduction
 * 
 * The bt_log object and associated functions implement a double buffer data
 * logger.  The double buffer mechanism allows data to be recorded at high
 * speed into memory while writes to disk are done as efficient block
 * operations in a low priority thread.
 *
 * \section sec_usage Usage
 *
 * First, create a new bt_log object using bt_log_create(). The argument is
 * the number of fields in the log.
 * \code
 * struct bt_log * mylog;
 * mylog = bt_log_create(3); 
 * \endcode
 * 
 * Next, add the fields using bt_log_addfield() and initialize using
 * bt_log_init():
 * \code
 * int ticks;
 * double time;
 * unsigned long long numbers[7];
 * bt_log_addfield(mylog, &ticks, 1, BT_LOG_INT, "ticks");
 * bt_log_addfield(mylog, &time, 1, BT_LOG_DOUBLE, "time");
 * bt_log_addfield(mylog, &numbers, 7, BT_LOG_ULONGLONG, "numbers");
 * bt_log_init(mylog,1000,"datafile.bin");
 * \endcode
 *
 * Now, in some time-important thread (usually a real-time loop), we can
 * trigger the bt_log object, which will copy the current values of all of
 * the fields into the designated memory location:
 * \code
 * while (going)
 * {
 *    ticks++;
 *    time = get_time();
 *    get_numbers(numbers);
 *    bt_log_trigger(mylog);
 *    do_stuff();
 * }
 * \endcode
 *
 * Concurrently, in a lower-priority thread, the log must be flushed
 * periodically to disk (the binary file specified in bt_log_init()):
 * \code
 * while (going)
 * {
 *    bt_log_flush(mylog);
 *    sleep(FOR_A_WHILE);
 * }
 * \endcode
 *
 * Once we're done, we destroy the bt_log object.  We can then decode the
 * binary file to a CSV file (or an octave file):
 * \code
 * bt_log_destroy(mylog);
 * bt_log_decode("datafile.bin", "datafile.csv", 1, 0);
 * \endcode
 */

#ifndef BT_LOG_H
#define BT_LOG_H

#include <stdio.h> /* For FILE */

/* Note - one record is like a line, with many fields */

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
  enum bt_log_fieldtype type; /*!< 0 = int, 1 = long, 2 = double 3 = long long 4 = btreal*/
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
  size_t record_size; /*!< Total size of one record of data*/
  int num_fields_max; /*!< Total number of data_info structures allocated.*/
  int num_fields; /*!< Number of data_info pieces*/
  int initialized;
  
  int num_records;
  int buf_record_idx;
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
  size_t record_size; /*!< Total size of one record of data*/
  int num_fields_max; /*!< Total number of data_info structures allocated.*/
  int num_fields; /*!< Number of data_info pieces*/
  int initialized;
  
  FILE * file;
  char * line;
  size_t line_length;
  int records_read;
};



/* public functions */
struct bt_log * bt_log_create(unsigned int num_fields);
int bt_log_addfield(struct bt_log * log, void * data, int num, enum bt_log_fieldtype type, char * name);
int bt_log_init(struct bt_log * log, int num_records, char * filename);
int bt_log_destroy(struct bt_log * log);

int bt_log_trigger(struct bt_log * log);
int bt_log_flush(struct bt_log * log);

int bt_log_decode(char * infile, char * outfile, int header, int octave);


/* For reading a CSV file */

struct bt_log_read * bt_log_read_create(unsigned int num_fields );
int bt_log_read_addfield(struct bt_log_read * logread, void * data, int num, enum bt_log_fieldtype type);
int bt_log_read_init(struct bt_log_read * logread, char * filename, int header);
int bt_log_read_destroy(struct bt_log_read * logread);

int bt_log_read_get(struct bt_log_read * logread, int * record_num_ptr);

#endif/* BT_LOG_H */
