/** Implementation of bt_log, a double-buffered data logger.
 *
 * \file log.c
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

#define _GNU_SOURCE /* This is for getline(), strtoll(), and strtoull() */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#undef _GNU_SOURCE

#include <syslog.h>

#include "log.h"

int bt_log_create(struct bt_log ** logptr, unsigned int num_fields)
{
   int i;
   struct bt_log * log;

   (*logptr) = 0;
   log = (struct bt_log *) malloc(sizeof(struct bt_log));
   if (!log)
      return -1;
   
   log->fields = (struct bt_log_field *)
      malloc( num_fields * sizeof(struct bt_log_field) );
   if (!log->fields)
   {
      free(log);
      return -1;
   }
   log->num_fields_max = num_fields;
   log->num_fields = 0;
   
   for (i=0; i<log->num_fields_max; i++)
      log->fields[i].size = 0;
   
   log->initialized = 0;
   log->finished = 0;

   (*logptr) = log;
   return 0;
}


int bt_log_addfield(struct bt_log * log, void * data, int num, enum bt_log_fieldtype type, char * name)
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
      return -2;
   }
   
   strcpy( log->fields[i].name, name );
   log->fields[i].data = data;
   log->fields[i].type = type;
   switch (type)
   {
      case BT_LOG_INT:
         log->fields[i].size = num * sizeof(int);
         break;
      case BT_LOG_LONG:
         log->fields[i].size = num * sizeof(long int);
         break;
      case BT_LOG_LONGLONG:
         log->fields[i].size = num * sizeof(long long int);
         break;
      case BT_LOG_ULONGLONG:
         log->fields[i].size = num * sizeof(unsigned long long int);
         break;
      case BT_LOG_DOUBLE:
         log->fields[i].size = num * sizeof(double);
         break;
   }
   
   log->num_fields++;
   return 0;
}


int bt_log_init(struct bt_log * log, int num_records, char * filename)
{
   int i;
   
   if (log->initialized)
   {
      syslog(LOG_ERR,"bt_log_init: logger already initialized.");
      return -1;
   }

   if (filename)
   {
      log->binfile = fopen(filename,"wb");
      if (!log->binfile)
      {
         syslog(LOG_ERR,"%s: Could not open datalogger file %s",__func__,filename);
         return -2;
      }
   }
   else
   {
      log->binfile = tmpfile();
      if (!log->binfile)
      {
         syslog(LOG_ERR,"%s: Could not open temporary datalogger file.",__func__);
         return -2;
      }
   }

   /* Figure out the total size per record */
   log->num_records = num_records;
   log->record_size = 0;
   for(i=0; i < log->num_fields; i++)
      log->record_size += log->fields[i].size;

   /* Allocate the buffers */
   log->buf_A = malloc(log->num_records * log->record_size);
   if (!log->buf_A)
   {
      fclose(log->binfile);
      return -3;
   }
   log->buf_B = malloc(log->num_records * log->record_size);
   if (!log->buf_B)
   {
      free(log->buf_A);
      fclose(log->binfile);
      return -3;
   }

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
   fwrite(&(log->num_fields),sizeof(int),1,log->binfile);   /* number of fields */
   for(i = 0; i < log->num_fields; i++) {
      fwrite(&(log->fields[i].type),sizeof(int),1,log->binfile);
      fwrite(&(log->fields[i].size),sizeof(int),1,log->binfile); /* record size = arraysize + sizeof(variable) */
      fwrite(log->fields[i].name,sizeof(char),50,log->binfile);
   }
   /* Save the current location (location of the data, after the header) */
   log->binfile_dataloc = ftell(log->binfile);

   log->buf = log->buf_A;
   log->buf_record_idx = 0;
   log->full = BT_LOG_FULL_NONE;
   log->chunk_idx = 0;
   log->initialized = 1;
   return 0;
}


int bt_log_destroy(struct bt_log * log)
{
   /* Finish the live logging if it's not already done */
   if (log->initialized && !log->finished)
      bt_log_finish(log);

   fclose(log->binfile);
   free(log->fields);
   free(log);
   return 0;
}


int bt_log_trigger(struct bt_log * log)
{
   int i;
   char * start; /* it's indexed by bytes, anyways */

   if (!log->initialized) return -1;
   if (log->finished) return -1;
   
   /* point to the present location in the buffer */
   start = log->buf + log->buf_record_idx * log->record_size; /* ######## MADE THIS CHANGE 08-12 */

   /* copy user data to buffer */
   for(i = 0; i < log->num_fields; i++) {
      memcpy(start, log->fields[i].data, log->fields[i].size);
      start += log->fields[i].size;
   }
   
   /* Was UpdateDL() */
   /* If our index exceeds our array size */
   log->buf_record_idx++;
   if (log->buf_record_idx >= log->num_records)
   {
      log->buf_record_idx = 0;                  /*reset our index to zero */

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


int bt_log_flush(struct bt_log * log)
{
   void * buf_full;
   long chunk_size;
   
   if (!log->initialized)
   {
      syslog(LOG_ERR,"%s: logger not yet initialized.",__func__);
      return -1;
   }
   if (log->finished) return -1;
   
   /* Make sure some buffer is full, and if so,
    * point buf_full to the full buffer.*/
   buf_full = 0;
   switch (log->full)
   {
      case BT_LOG_FULL_NONE:
         return 0;
      case BT_LOG_FULL_A:
         buf_full = log->buf_A;
         break;
      case BT_LOG_FULL_B:
         buf_full = log->buf_B;
         break;
   }
   
   /* Write record index (full buffer counter) as binary integer
    * Start of chunk : 4 bytes for the chunk index */
   fwrite(&(log->chunk_idx),sizeof(int),1,log->binfile);
   
   /* Calculate / write the chunk size in bytes
    * (this is a full chunk) */
   chunk_size = log->num_records * log->record_size;
   fwrite(&chunk_size,sizeof(long),1,log->binfile);

   /* Write all the data in binary form */
   fwrite(buf_full, log->record_size, log->num_records, log->binfile);
   
   /* Increment the chunk index */
   log->chunk_idx++;
   
   /* Reset full buffer indicator to zero */
   log->full = BT_LOG_FULL_NONE;
   
   return 0;
}


int bt_log_finish(struct bt_log * log)
{
   long chunk_size;
   
   if (!log->initialized) return -1;
   if (log->finished) return -1;

   /* Write what's remaining in the current buffer
    * if there's something there */
   if (log->buf_record_idx)
   {
      fwrite(&log->chunk_idx,sizeof(int),1,log->binfile);     /*Write Record index as a binary integer*/
      
      chunk_size = log->buf_record_idx * log->record_size;         /*Calculate the chunk size*/
      fwrite(&chunk_size,sizeof(long),1,log->binfile);     /*Write the Record length in bytes*/

      fwrite(log->buf, log->record_size, log->buf_record_idx, log->binfile);  /*Write all the data in binary form*/
      log->chunk_idx++;                                        /*ncrement the record index*/
   }
   
   free(log->buf_A);
   free(log->buf_B);

   log->finished = 1;

   return 0;
}


int bt_log_decode_start(struct bt_log * log)
{
   /* Make sure we're finished writing live data to the binfile */
   if (!log->finished) return -1;

   /* Go to the beginning of the binfile, just after the binary header */
   fseek(log->binfile, log->binfile_dataloc, SEEK_SET);

   /* We haven't read any records,
    * and there are no more records to read in the current chunk */
   log->records_read = 0;
   log->chunk_records_to_read = 0;
   
   return 0;
}


int bt_log_decode_next(struct bt_log * log, int * record_num_ptr)
{
   int i;
   
   /* Make sure we're finished writing live data to the binfile */
   if (!log->finished) return -1;

   /* If we're about to decode the next chunk */
   if (!log->chunk_records_to_read)
   {
      int chunk_idx;
      long chunk_size;

      /* Attempt to read (and ignore) the chunk index */
      fread(&chunk_idx,sizeof(int),1,log->binfile);

      /* Quit if we're done decoding */
      if(feof(log->binfile)) return 1;

      /* Read the chunk size (in bytes) */
      fread(&chunk_size,sizeof(long),1,log->binfile);
      log->chunk_records_to_read = chunk_size / log->record_size;
   }

   /* chunk_records_to_read should be non-zero at this point */

   /* read a record */
   for(i = 0; i < log->num_fields; i++)
      fread(log->fields[i].data, log->fields[i].size, 1, log->binfile);

   if (record_num_ptr)
      (*record_num_ptr) = log->records_read;
   
   log->records_read++;
   log->chunk_records_to_read--;

   return 0;
}


int bt_log_decode_file(char * infile, char * outfile, int header, int octave)
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
   bt_log_create(&log,fieldcnt);

   /* Octave Header */
   if (octave) fprintf(outf,"# Created by btlog: ");
   numcols = 0;
   /* Read header info, write text header */
   for (i=0; i<fieldcnt; i++) {
      fread(&(log->fields[i].type),sizeof(int),1,inf);
      fread(&(log->fields[i].size),sizeof(int),1,inf);
      fread(log->fields[i].name,sizeof(char),50,inf);

      switch (log->fields[i].type)
      {
         case BT_LOG_INT:
            array_len = log->fields[i].size / sizeof(int);
            break;
         case BT_LOG_LONG:
            array_len = log->fields[i].size / sizeof(long);
            break;
         case BT_LOG_LONGLONG:
            array_len = log->fields[i].size / sizeof(long long);
            break;
         case BT_LOG_ULONGLONG:
            array_len = log->fields[i].size / sizeof(unsigned long long);
            break;
         case BT_LOG_DOUBLE:
            array_len = log->fields[i].size / sizeof(double);
            break;
         default:
            /* Return an error? */
            return -8;
      }

      if (header) {
         if (array_len > 1) {
            for (ridx = 0; ridx < array_len; ridx++) {
               fprintf(outf,"%s[%ld]",log->fields[i].name,ridx);
               if ((ridx < array_len - 1) || (i < fieldcnt - 1))
                  fprintf(outf,", ");
               numcols++;
            }
         } else {
            fprintf(outf,"%s",log->fields[i].name);
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
            switch (log->fields[i].type) {
               case BT_LOG_INT:
                  array_len = log->fields[i].size / sizeof(int);
                  if ((log->fields[i].size % sizeof(int)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This record is not an even multiple of our datatype (int)");
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
                  cnt += log->fields[i].size;
                  break;
               case BT_LOG_LONG:
                  array_len = log->fields[i].size / sizeof(long);
                  if ((log->fields[i].size % sizeof(long)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This record is not an even multiple of our datatype (long)");
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
                  cnt += log->fields[i].size;
                  break;
               case BT_LOG_LONGLONG:
                  array_len = log->fields[i].size / sizeof(long long);
                  if ((log->fields[i].size % sizeof(long long)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This record is not an even multiple of our datatype (int)");
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
                  cnt += log->fields[i].size;
                  break;
               case BT_LOG_ULONGLONG:
                  array_len = log->fields[i].size / sizeof(unsigned long long);
                  if ((log->fields[i].size % sizeof(unsigned long long)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This record is not an even multiple of our datatype (int)");
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
                  cnt += log->fields[i].size;
                  break;
               case BT_LOG_DOUBLE:
                  array_len = log->fields[i].size / sizeof(double);
                  if ((log->fields[i].size % sizeof(double)) != 0)
                     syslog(LOG_ERR,"DecodeDL: This record is not an even multiple of our datatype (int)");
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
                  cnt += log->fields[i].size;
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
   
   free(log->fields);
   free(log);
   
   fseek(outf, fwrite_rowpos, SEEK_SET);
   if (octave) fprintf(outf,"# rows: %-20d\n",numrows);
   
   fclose(inf);
   fclose(outf);
   return 0;
}