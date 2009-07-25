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

/** \file log.h
 *
 * \section sec_intro Introduction
 * 
 * The bt_log object and associated functions implement a double buffer data
 * logger.  The double buffer mechanism allows data to be recorded at high
 * speed into memory while writes to disk are done as efficient block
 * operations in a low priority thread.
 *
 * Terminology:
 *  - a \b field is a value or array of values, such as an int or a double[3]
 *  - a \b record is a set of fields to be synchronously logged on each call
 *    to bt_log_trigger()
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
 * bt_log_init().  The second argument to bt_log_init(), num_records, is the
 * size to make each data buffer in memory.  This should be made large enough
 * that the bt_log can be flushed before it fills up.
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
 * trigger the bt_log object, which will copy a single record, containing the
 * current values of all of the fields, into the designated memory location:
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
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h> /* For FILE */

/* Note - one record is like a line, with many fields */

/** bt_log and bt_log_read field types. */
enum bt_log_fieldtype
{
  BT_LOG_INT = 0,
  BT_LOG_LONG,
  BT_LOG_LONGLONG,
  BT_LOG_ULONGLONG,
  BT_LOG_DOUBLE
};

/** Description of each field to log. */
struct bt_log_field
{
   char name[50]; /**< null terminated string describing the data. No ','s allowed*/
   enum bt_log_fieldtype type;
   size_t size; /**< size of the data being recorded. = sizeof(datatype)*arrayLength*/
   void * data; /**< pointer to the data start*/
};

/** Is one of the buffers full? */
enum bt_log_full
{
   BT_LOG_FULL_NONE,
   BT_LOG_FULL_A,
   BT_LOG_FULL_B
};

/** Data logging object.
 *
 * A bt_log object is used to synchronously log a number of fields to memory,
 * and then periodically flush the log to disk.  This object should be
 * created by the bt_log_create() function and initialized by the
 * bt_log_addfield() and bt_log_init() functions before it is used.
 * See log.h for a detailed usage example.
 */
struct bt_log
{
  struct bt_log_field * fields; /*!< list of data information*/
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

/** Data log reading object.
 *
 * A bt_log_read object is used to read structured data from a CSV file
 * generated from a bt_log.
 */
struct bt_log_read
{
  struct bt_log_field * fields; /*!< list of data information*/
  size_t record_size; /*!< Total size of one record of data*/
  int num_fields_max; /*!< Total number of data_info structures allocated.*/
  int num_fields; /*!< Number of data_info pieces*/
  int initialized;
  
  FILE * file;
  char * line;
  size_t line_length;
  int records_read;
};


/** Create a bt_log object with the specified number of fields.
 *
 * Once created, the fields must be added, and the bt_log object initialized,
 * before it can be triggered / flushed.  See log.h for a detailed usage
 * example.
 *
 * \param[in] num_fields The number of fields to allocate
 * \return The bt_log object on success, or 0 on failure
 */
struct bt_log * bt_log_create(unsigned int num_fields);


/** Adds a field to log to the bt_log object.
 *
 * You must call bt_log_addfield the same number of times as specified in
 * the argument to bt_log_create().
 *
 * \param[in] log The bt_log object
 * \param[in] data A pointer to the first object in the field
 * \param[in] num The number of objects in the field (1 for a single object)
 * \param[in] type The type of the field
 * \param[in] name The name of the field (added to the decoded file header)
 * \retval 0 Success
 * \retval -1 bt_log already initialized
 * \retval -2 No more fields left; increase num_fields in bt_log_create()
 */
int bt_log_addfield(struct bt_log * log, void * data, int num,
                    enum bt_log_fieldtype type, char * name);


/** Initialize the buffers and prepare for logging.
 *
 * Call bt_log_init() after adding all fields with bt_log_addfield().  You
 * must specify the number of records to reserve memory space for, and the
 * filename to write the binary data to.  Remember that twice the memory will
 * be allocated, due to the double-buffering nature of bt_log.
 *
 * \param[in] log The bt_log object
 * \param[in] num_records The number of records to reserve memory space for
 * \param[in] filename The filename to flush the binary data to
 * \retval 0 Success
 * \retval -1 bt_log already initialized
 * \retval -2 Could not open binary file for writing
 * \retval -3 Not enough memory for the buffers
 */
int bt_log_init(struct bt_log * log, int num_records, char * filename);


/** Destroy the bt_log object.
 *
 * Completely de-allocate a bt_log object.
 *
 * \param[in] log The bt_log object
 * \retval 0 Success
 */
int bt_log_destroy(struct bt_log * log);


/** Trigger the bt_log to copy the current field values to memory.
 *
 * This function should be called every time you want to save a record to
 * the buffer.  Typically, this is called in a high priority thread with
 * a short loop period.
 *
 * \param[in] log The bt_log object
 * \retval 0 Success
 * \retval -1 bt_log not yet initialized
 */
int bt_log_trigger(struct bt_log * log);


/** Checks the buffers and writes them if one is full.
 *
 * This must be cyclically called (from an event loop perhaps) with a
 * period that is shorter than the time it takes to fill the buffer.
 *
 * \warning If you do not call this often enough, you will have buffer sized
 * gaps in your data file.
 *
 * \param[in] log The bt_log object
 * \retval 0 Success (buffer written, or no buffers full)
 * \retval -1 bt_log not yet initialized
 */
int bt_log_flush(struct bt_log * log);


/** Decode a bt_log binary file into a CSV or GNU Octave data file.
 *
 * \param[in] infile The binary data file generated by the bt_log object
 * \param[in] outfile The text file to save the data to
 * \param[in] header Whether to add a header to the file (1) or not (0)
 * \param[in] octave Whether to output in GNU Octave format (1) or CSV (0)
 * \retval 0 Success
 * \retval -1 Could not open input file
 * \retval -2 Could not open output file
 */
int bt_log_decode(char * infile, char * outfile, int header, int octave);


/** Create a bt_log_read object with the specified number of fields.
 *
 * Once created, the fields must be added, and the bt_log object initialized,
 * before it can be queried.  The bt_log_read object currently supports only
 * CSV files with single-width fields (num = 1).
 *
 * \param[in] num_fields The number of fields to allocate
 * \return The bt_log_read object (on success)
 * \return 0 (on failure)
 */
struct bt_log_read * bt_log_read_create(unsigned int num_fields);


/** Adds a field to retrieve to the bt_log_read object.
 *
 * You must call bt_log_read_addfield the same number of times as specified
 * in the argument to bt_log_read_create().
 *
 * \param[in] logread The bt_log_read object
 * \param[in] data A pointer to the first object in the field
 * \param[in] num The number of objects in the field (1 for a single object)
 * \param[in] type The type of the field
 * \retval 0 Success
 * \retval -1 bt_log_read already initialized
 * \retval -2 No more fields left; increase num_fields in
 *            bt_log_read_create()
 */
int bt_log_read_addfield(struct bt_log_read * logread, void * data, int num,
                         enum bt_log_fieldtype type);


/** Prepare for log reading.
 *
 * Call bt_log_read_init() after adding all fields with
 * bt_log_read_addfield().
 *
 * \param[in] logread The bt_log_read object
 * \param[in] filename The CSV file to retrieve the values from
 * \retval 0 Success
 * \retval -1 bt_log_read already initialized
 * \retval -2 Could not open CSV file for reading
 */
int bt_log_read_init(struct bt_log_read * logread, char * filename,
                     int header);


/** Destroy the bt_log_read object.
 *
 * Completely de-allocate a bt_log_read object.
 *
 * \param[in] logread The bt_log_read object
 * \retval 0 Success
 */
int bt_log_read_destroy(struct bt_log_read * logread);

/** Get the next record from a bt_log_read object.
 *
 * This function should be called iteratively on a bt_log_read object once
 * it has been correctly initialized.  Each subsequent call will retrieve the
 * next record into the fields designated by the bt_log_read_addfield()
 * function.  If record_num_ptr is passed, the current record number will
 * placed in *record_num_ptr, starting from 0 (first line).
 *
 * \param[in] logread The bt_log_read object
 * \param[out] record_num_ptr Pointer to a location to store the current 
 *                            record number
 */
int bt_log_read_get(struct bt_log_read * logread, int * record_num_ptr);

#ifdef __cplusplus
}
#endif
#endif/* BT_LOG_H */
