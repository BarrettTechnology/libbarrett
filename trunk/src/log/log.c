/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... btlogger.c
 *  Author ............. Traveler Hauptman  
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 2003 Apr 3
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2003-2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:                                                              
 *    Uses some code & concepts developed at Nortwestern University 
 *    by Traveler Hauptman          
 *
 *  REVISION HISTORY:                                                   
 *    2005 Nov 01 - TH
 *      Checked
 *    2008 Sept 15 - CD
 *      Ported from btsystem to libbt
 *                                                                      
 * ======================================================================== */

#define _GNU_SOURCE

#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <syslog.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "log.h"


/*==============================*
 * Functions                    *
 *==============================*/
/*****************************************************************************/
/** Allocate a specified number of fields.
 
You can add fewer fields than specified by PrepDL() but you cannot AddDataDL() 
after calling InitDL().
 
Typical usage:
\code
  PrepDL(&log,3);
  AddDataDL(&log,&a,sizeof(double),BTLOG_DOUBLE,"Arclength");
  //...more AddDataDL()
  //initialize buffers
  InitDL(&log,1000,"logfile.dat");
\endcode
 
\param db Pointer to the btlogger structure
\param fields The number of fields you want to record
 
\internal chk'd TH 051101
*/
/* Note: This is like prepDL */
struct bt_log * bt_log_create( unsigned int num_fields )
{
   int i;
   struct bt_log * log;
   
   log = (struct bt_log *) malloc(sizeof(struct bt_log));
   
   log->data = (struct bt_log_data_info *)
      malloc( num_fields * sizeof(struct bt_log_data_info) );
   log->num_fields_max = num_fields;
   log->num_fields = 0;
   
   for (i=0; i<log->num_fields_max; i++)
      log->data[i].size = 0;
   
   log->initialized = 0;
   
   return log;
}


/** Adds a field to the list of data to log.
 
You must call PrepDL() first. Fields are
added in the order that this function is called.
 
\param size Size of data = Array_length * sizeof(type)
\param data Pointer to data you want to log.
\param type See #btlog_enum
\param name Data name to be printed in the column heading
\retval 0 Success
\retval -1 No more fields available
\retval -2 db was invalid 
 
\internal chk'd TH 051101
\bug Check to see if we have already initialized with InitDL()
*/
/* Note: this is like AddDataDL */
int bt_log_addfield( struct bt_log * log, void * data, int num, enum bt_log_fieldtype type, char * name)
/* int AddDataDL(btlogger *db,void *data, int size, int type,char *name) */
{
   int i;
   
   if (log->initialized)
   {
      syslog(LOG_ERR,"bt_log_addfield: logger already initialized.");
      return -1;
   }
   
   i = log->num_fields;
   
   if (i >= log->num_fields_max)
   {
      syslog(LOG_ERR,"bt_log_addfield: No more field slots left. Use a higher value in btlog_create()");
      return -1;
   }
   
   strcpy( log->data[i].name, name );
   log->data[i].data = data;
   log->data[i].type = type;
   switch (type)
   {
      case BT_LOG_INT:
         log->data[i].size = num * sizeof(int);
         break;
      case BT_LOG_LONG:
         log->data[i].size = num * sizeof(long int);
         break;
      case BT_LOG_LONGLONG:
         log->data[i].size = num * sizeof(long long int);
         break;
      case BT_LOG_ULONGLONG:
         log->data[i].size = num * sizeof(unsigned long long int);
         break;
      case BT_LOG_DOUBLE:
         log->data[i].size = num * sizeof(double);
         break;
   }
   
   log->num_fields++;
   return 0;
}


/** Initialize the buffers and prepare for logging.
 
 Don't forget to call PrepDL() 
and AddDataDL() first to define what fields will be recorded.
 
\param size The number of records the buffer will hold
\param filename The path and filename where you want to write to
\retval 0 Success
\retval -1 Could not open file
\internal chk'd TH 051101
*/

int bt_log_init( struct bt_log * log, int num_blocks, char * filename)
{
   int i;
   
   if (log->initialized)
   {
      syslog(LOG_ERR,"bt_log_init: logger already initialized.");
      return -1;
   }
   
   if ((log->file = fopen(filename,"w"))==NULL) {
      syslog(LOG_ERR,"bt_log_init: Could not open datalogger file %s",filename);
      return -1;
   }

   /* Figure out the total size per record */
   log->num_blocks = num_blocks;
   log->block_size = 0;
   for(i=0; i < log->num_fields; i++)
      log->block_size += log->data[i].size;
   
   log->buf_A = malloc(log->num_blocks * log->block_size);
   log->buf_B = malloc(log->num_blocks * log->block_size);

   /* Was InitDataFileDL() */
   /*
    * --0--1--2--3---4--5--6--7--8--9-10-11-12-....-61
    * |  :  :  :  ||  :  :  :  |  :  :  :  |          |
    *   num_fields    f0:type     f0:size    f0:name
    *
    *               62-63-64-65-66-67-68-69-70-...-119
    *              |  :  :  :  |  :  :  :  |          |
    *                 f1:type     f1:size    f1:name
    *
    *              ... (num_fields of these)
    */
   fwrite(&(log->num_fields),sizeof(int),1,log->file);   /* number of fields */
   for(i = 0; i < log->num_fields; i++) {
      fwrite(&(log->data[i].type),sizeof(int),1,log->file);
      fwrite(&(log->data[i].size),sizeof(int),1,log->file); /* block size = arraysize + sizeof(variable) */
      fwrite(log->data[i].name,sizeof(char),50,log->file);
   }

   log->buf = log->buf_A;
   log->buf_block_idx = 0;
   log->full = BT_LOG_FULL_NONE;
   log->chunk_idx = 0;
   log->initialized = 1;
   return 0;
}


/** Close the logging file and free buffer memory
\internal chk'd TH 051101
*/
int bt_log_destroy( struct bt_log * log )
{
   /* Was flushDL() */
   long chunk_size;

   if (log->initialized) {
      /* If our data logging files are open and everything is peachy */
      fwrite(&log->chunk_idx,sizeof(int),1,log->file);     /*Write Record index as a binary integer*/
      
      chunk_size = log->buf_block_idx * log->block_size;         /*Calculate the chunk size*/
      fwrite(&chunk_size,sizeof(long),1,log->file);     /*Write the Record length in bytes*/

      fwrite(log->buf, log->block_size, log->buf_block_idx, log->file);  /*Write all the data in binary form*/
      log->chunk_idx++;                                        /*ncrement the record index*/
      
      free(log->buf_A);
      free(log->buf_B);
      fclose(log->file);
   }
   
   free(log->data);
   free(log);
   return 0;
}


/** Copy all the data pointed to by the btdata_info array into the present buffer.
 
 This function should be called every time you want to save a record to the buffer.
 Typically this is called in a high priority thread with a short loop period.
 
 \internal chk'd TH 051101
*/
int bt_log_trigger( struct bt_log * log )
{
   int i;
   char * start; /* it's indexed by bytes, anyways */
   
   /* point to the present location in the buffer */
   start = log->buf + log->buf_block_idx * log->block_size; /* ######## MADE THIS CHANGE 08-12 */

   /* copy user data to buffer */
   for(i = 0; i < log->num_fields; i++) {
      memcpy(start, log->data[i].data, log->data[i].size);
      start += log->data[i].size;
   }
   
   /* Was UpdateDL() */
   /* If our index exceeds our array size */
   log->buf_block_idx++;
   if (log->buf_block_idx >= log->num_blocks)
   {
      log->buf_block_idx = 0;                  /*reset our index to zero */

      /*If we are currently pointed to buffer A */
      if (log->buf == log->buf_A)
      {
         log->full = BT_LOG_FULL_A;
         log->buf = log->buf_B;           /*Point to buffer B */
      }
      else
      {
         log->full = BT_LOG_FULL_B;
         log->buf = log->buf_A;           /*Point to buffer A */
      }
   }
   
   return 0;
}


/** Checks the buffers and writes them if one is full.
 
This must be cyclically called (from an event loop perhaps)
with a period that is shorter than the time it takes to fill the buffer.
 
\warning If you do not call this often enough, you will have buffer sized gaps
in your data file.
 
\internal chk'd TH 051101
*/
/* was evalDL() */
int bt_log_flush( struct bt_log * log )
{
   void * buf_full;
   long chunk_size;
   
   if (!log->initialized)
   {
      syslog(LOG_ERR,"%s: logger not yet initialized.",__func__);
      return -1;
   }
   
   /* Make sure some buffer is full */
   if (log->full == BT_LOG_FULL_NONE) return 0;
   
   /* Point buf_full to the full buffer */
   if (log->full == BT_LOG_FULL_A)
      buf_full = log->buf_A;
   else if (log->full == BT_LOG_FULL_B)
      buf_full = log->buf_B;
   else
      return -1; /* Huh? */

   /* Write record index (full buffer counter) as binary integer
    * Start of chunk : 4 bytes for the chunk index */
   fwrite(&(log->chunk_idx),sizeof(int),1,log->file);
   
   /* Calculate / write the chunk size in bytes
    * (this is a full chunk) */
   chunk_size = log->num_blocks * log->block_size;
   fwrite(&chunk_size,sizeof(long),1,log->file);

   /* Write all the data in binary form */
   fwrite(buf_full, log->block_size, log->num_blocks, log->file);
   
   /* Increment the chunk index */
   log->chunk_idx++;
   
   /* Reset full buffer indicator to zero */
   log->full = BT_LOG_FULL_NONE;
   
   return 0;
}



/*************************** binary file to text file converter ***********/
/** Decode a binary file created by btlogger.

 
\param header If header !0 then the first line will be column names
 
\internal chk'd TH 051101
*/
int bt_log_decode( char * infile, char * outfile, int header, int octave)
{
   struct bt_log * log; /*use this just because it is convinient*/
   FILE *inf,*outf;
   int fieldcnt;
   int current_index;
   long length;
   char *data,*dataidx;
   int i;
   long cnt,ridx;
   int array_len;

   int *intdata;
   double *doubledata;
   long *longdata;
   long long *exlongdata;
   unsigned long long *exulongdata;
   
   int numcols;
   int numrows;
   long int fwrite_rowpos;
   
   if (octave) header = 1;

   syslog(LOG_ERR,"DecodeDL: Starting logfile decode from %s -> %s",infile,outfile);

   /*open input file*/
   if ((inf = fopen(infile,"rb"))==NULL) {
      syslog(LOG_ERR,"DecodeDL: Unable to open input file: %s",infile);
      return -1;
   }

   /*open output file*/
   if ((outf = fopen(outfile,"w"))==NULL) {
      syslog(LOG_ERR,"DecodeDL: Unable to open output file: %s\n",outfile);
      return -2;
   }
   
   /*figure out fields*/
   /*print fields*/
   fread(&fieldcnt,sizeof(int),1,inf);
   syslog(LOG_ERR,"DecodeDL:Fields %d",fieldcnt);
   log = bt_log_create(fieldcnt);

   /* Octave Header */
   if (octave) fprintf(outf,"# Created by btlog: ");
   numcols = 0;
   /* Read header info, write text header */
   for (i=0; i<fieldcnt; i++) {
      fread(&(log->data[i].type),sizeof(int),1,inf);
      fread(&(log->data[i].size),sizeof(int),1,inf);
      fread(log->data[i].name,sizeof(char),50,inf);

      switch (log->data[i].type)
      {
         case BT_LOG_INT:
            array_len = log->data[i].size / sizeof(int);
            break;
         case BT_LOG_LONG:
            array_len = log->data[i].size / sizeof(long);
            break;
         case BT_LOG_LONGLONG:
            array_len = log->data[i].size / sizeof(long long);
            break;
         case BT_LOG_ULONGLONG:
            array_len = log->data[i].size / sizeof(unsigned long long);
            break;
         case BT_LOG_DOUBLE:
            array_len = log->data[i].size / sizeof(double);
            break;
         default:
            /* Return an error? */
            return -8;
      }

      if (header) {
         if (array_len > 1) {
            for (ridx = 0; ridx < array_len; ridx++) {
               fprintf(outf,"%s[%ld]",log->data[i].name,ridx);
               if ((ridx < array_len - 1) || (i < fieldcnt - 1))
                  fprintf(outf,", ");
               numcols++;
            }
         } else {
            fprintf(outf,"%s",log->data[i].name);
            if (i < fieldcnt - 1)
               fprintf(outf,", ");
            numcols++;
         }
      }

      /*syslog(LOG_ERR,"DecodeDL:Field %d - type: %d size: %d, name: %s",cnt,db.data[cnt].type,db.data[cnt].size,db.data[cnt].name);*/
   }
   if (header) fprintf(outf,"\n");
   /* Octave Header */
   if (octave) fprintf(outf,"# name: btlog\n");
   if (octave) fprintf(outf,"# type: matrix\n");
   fwrite_rowpos = ftell(outf);
   if (octave) fprintf(outf,"# rows: %-20d\n",0); /* Fill this in at the end! */
   if (octave) fprintf(outf,"# columns: %d\n",numcols);

   numrows = 0;
   while(!feof(inf))
   {
      if(fread(&current_index,sizeof(int),1,inf)==0)
         break;
      fread(&length,sizeof(long),1,inf);
      /*syslog(LOG_ERR,"DecodeDL:record index: %d length: %ld",current_index,length);*/
      data = malloc(length);

      fread(data,sizeof(char),length,inf);

      dataidx = data;
      cnt = 0;
      while(cnt<length)
      {
         
         for (i=0; i<fieldcnt; i++)
         {
            switch (log->data[i].type) {
               case BT_LOG_INT:
                  array_len = log->data[i].size / sizeof(int);
                  if ((log->data[i].size % sizeof(int)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
                  for (ridx = 0; ridx < array_len; ridx++) {
                     intdata = (int *)dataidx;
                     if (octave)
                        fprintf(outf," %d",*intdata);
                     else
                     {
                        fprintf(outf,"%d",*intdata);
                        if (ridx < array_len - 1)
                           fprintf(outf,", ");
                     }
                     dataidx += sizeof(int);
                  }
                  cnt += log->data[i].size;
                  break;
               case BT_LOG_LONG:
                  array_len = log->data[i].size / sizeof(long);
                  if ((log->data[i].size % sizeof(long)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (long)");
                  for (ridx = 0; ridx < array_len; ridx++) {
                     longdata = (long *)dataidx;
                     if (octave)
                        fprintf(outf," %ld",*longdata);
                     else
                     {
                        fprintf(outf,"%ld",*longdata);
                        if (ridx < array_len - 1)
                           fprintf(outf,", ");
                     }
                     dataidx +=  sizeof(long);
                  }
                  cnt += log->data[i].size;
                  break;
               case BT_LOG_LONGLONG:
                  array_len = log->data[i].size / sizeof(long long);
                  if ((log->data[i].size % sizeof(long long)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
                  for (ridx = 0; ridx < array_len; ridx++) {
                     exlongdata = (long long *)dataidx;
                     if (octave)
                        fprintf(outf, "%lld",*exlongdata);
                     else
                     {
                        fprintf(outf,"%lld",*exlongdata);
                        if (ridx < array_len - 1)
                           fprintf(outf,", ");
                     }
                     dataidx += sizeof(long long);
                  }
                  cnt += log->data[i].size;
                  break;
               case BT_LOG_ULONGLONG:
                  array_len = log->data[i].size / sizeof(unsigned long long);
                  if ((log->data[i].size % sizeof(unsigned long long)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
                  for (ridx = 0; ridx < array_len; ridx++) {
                     exulongdata = (unsigned long long *)dataidx;
                     if (octave)
                        fprintf(outf, "%llu",*exulongdata);
                     else
                     {
                        fprintf(outf,"%llu",*exulongdata);
                        if (ridx < array_len - 1)
                           fprintf(outf,", ");
                     }
                     dataidx += sizeof(unsigned long long);
                  }
                  cnt += log->data[i].size;
                  break;
               case BT_LOG_DOUBLE:
                  array_len = log->data[i].size / sizeof(double);
                  if ((log->data[i].size % sizeof(double)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
                  for (ridx = 0; ridx < array_len; ridx++) {
                     doubledata = (double *)dataidx;
                     if (octave)
                        fprintf(outf," %f",*doubledata);
                     else
                     {
                        fprintf(outf,"%f",*doubledata);
                        if (ridx < array_len - 1)
                           fprintf(outf,", ");
                     }
                     dataidx += sizeof(double);
                  }
                  cnt += log->data[i].size;
                  break;
            }
            if (i < fieldcnt-1 && !octave)
               fprintf(outf,", ");

         }
         fprintf(outf,"\n");
         numrows++;
      }
      free(data);
   
   }
   
   free(log->data);
   free(log);
   
   fseek(outf, fwrite_rowpos, SEEK_SET);
   if (octave) fprintf(outf,"# rows: %-20d\n",numrows);
   
   fclose(inf);
   fclose(outf);
   return 0;
}


/* Log read functions ...
 * Note that this doesn't yet support arrays of fields ... */

struct bt_log_read * bt_log_read_create( unsigned int num_fields )
{
   int i;
   struct bt_log_read * logread;
   
   logread = (struct bt_log_read *) malloc(sizeof(struct bt_log_read));
   
   logread->data = (struct bt_log_data_info *)
      malloc( num_fields * sizeof(struct bt_log_data_info) );
   logread->num_fields_max = num_fields;
   logread->num_fields = 0;
   
   for (i=0; i<logread->num_fields_max; i++)
      logread->data[i].size = 0;
   
   logread->initialized = 0;
   
   return logread;
}

int bt_log_read_addfield( struct bt_log_read * logread, void * data, int num, enum bt_log_fieldtype type)
{
   int i;
   
   if (logread->initialized)
   {
      syslog(LOG_ERR,"bt_log_read_addfield: logger already initialized.");
      return -1;
   }
   
   i = logread->num_fields;
   
   if (i >= logread->num_fields_max)
   {
      syslog(LOG_ERR,"bt_log_read_addfield: No more field slots left. Use a higher value in btlog_create()");
      return -1;
   }
   
   logread->data[i].data = data;
   logread->data[i].type = type;
   switch (type)
   {
      case BT_LOG_INT:
         logread->data[i].size = num * sizeof(int);
         break;
      case BT_LOG_LONG:
         logread->data[i].size = num * sizeof(long int);
         break;
      case BT_LOG_LONGLONG:
         logread->data[i].size = num * sizeof(long long int);
         break;
      case BT_LOG_ULONGLONG:
         logread->data[i].size = num * sizeof(unsigned long long int);
         break;
      case BT_LOG_DOUBLE:
         logread->data[i].size = num * sizeof(double);
         break;
   }
   
   logread->num_fields++;
   return 0;
}

int bt_log_read_init( struct bt_log_read * logread, char * filename, int header)
{
   
   if (logread->initialized)
   {
      syslog(LOG_ERR,"bt_log_read_init: logger already initialized.");
      return -1;
   }
   
   if ((logread->file = fopen(filename,"r"))==NULL) {
      syslog(LOG_ERR,"bt_log_read_init: Could not open datalogger file %s",filename);
      return -1;
   }
   
   logread->line_length = 0;
   logread->line = 0;
   
   /* Skip the header, if its there ... */
   if (header)
      getline( &logread->line, &logread->line_length, logread->file );
   
   logread->blocks_read = 0;
   
   logread->initialized = 1;
   return 0;
}

int bt_log_read_destroy( struct bt_log_read * logread )
{
   if (logread->initialized) fclose(logread->file);
   if (logread->line) free(logread->line);
   free(logread);
   return 0;
}

int bt_log_read_get( struct bt_log_read * logread, int * block_num_ptr )
{
   int ret;
   char * field;
   int i;
   
   if (!logread->initialized)
   {
      syslog(LOG_ERR,"bt_log_read_get: logger not yet initialized.");
      return -1;
   }
   
   /* Grab the next line */
   ret = getline( &logread->line, &logread->line_length, logread->file );
   if (ret == -1)
   {
      /* End of file (or error) */
      return -2;
   }
   
   /* Tokenize the string by ',' */
   for(i = 0; i < logread->num_fields; i++)
   {
      if (i==0)
         field = strtok( logread->line, "," );
      else
         field = strtok( 0, "," );
      if (!field)
         return -3; /* Shouldn't happen (not enough commas?) */
      
      /* Copy the value */
      switch (logread->data[i].type)
      {
         int var_int;
         long int var_long_int;
         long long int var_long_long_int;
         unsigned long long int var_unsigned_long_long_int;
         double var_double;
         case BT_LOG_INT:
            var_int = strtol(field,0,0);
            memcpy( logread->data[i].data, &var_int, sizeof(int) );
            break;
         case BT_LOG_LONG:
            var_long_int = strtol(field,0,0);
            memcpy( logread->data[i].data, &var_long_int, sizeof(long int) );
            break;
         case BT_LOG_LONGLONG:
            var_long_long_int = strtoll(field,0,0);
            memcpy( logread->data[i].data, &var_long_long_int, sizeof(long long int) );
            break;
         case BT_LOG_ULONGLONG:
            var_unsigned_long_long_int = strtoull(field,0,0);
            memcpy( logread->data[i].data, &var_unsigned_long_long_int, sizeof(unsigned long long int) );
            break;
         case BT_LOG_DOUBLE:
            var_double = strtod(field,0);
            memcpy( logread->data[i].data, &var_double, sizeof(double) );
            break;
      }
   }
   
   if (block_num_ptr) (*block_num_ptr) = logread->blocks_read;
   logread->blocks_read++;
   
   return 0;
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
 
