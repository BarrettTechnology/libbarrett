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

#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
   log->fields = 0;
   log->maxfields = num_fields;
   
   for (i=0; i<num_fields; i++)
      log->data[i].size = 0;
   
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
   
   i = log->fields;
   
   if (i >= log->maxfields)
   {
      syslog(LOG_ERR,"btlog_addfield: No more field slots left. Use a higher value in btlog_create()");
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
      case BT_LOG_DOUBLE:
         log->data[i].size = num * sizeof(double);
         break;
   }
   
   log->fields++;
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

int bt_log_init( struct bt_log * log, int size, char * filename)
{
   int i;
   
   log->Log_Data = 0;
   log->DLwritten = 0;
   log->Log_File_Open = 0;
   log->DLctr = 0;
   log->DLidx = 0;
   if ((log->DLfile = fopen(filename,"w"))==NULL) {
      syslog(LOG_ERR,"btlog_init:Could not open datalogger file %s",filename);
      return -1;
   }
   log->Log_File_Open = 1;
   log->buffersize = size;
   
   /* Figure out the total size per record */
   log->data_size = 0;
   for(i=0; i < log->maxfields; i++)
      log->data_size += log->data[i].size;
   
   log->DLbuffer1 = malloc(size * log->data_size);
   log->DLbuffer2 = malloc(size * log->data_size);

   /* Was InitDataFileDL() */
   fwrite(&(log->fields),sizeof(int),1,log->DLfile);   /* number of fields */
   for(i = 0; i < log->fields; i++) {
      fwrite(&(log->data[i].type),sizeof(int),1,log->DLfile);
      fwrite(&(log->data[i].size),sizeof(int),1,log->DLfile); /* block size = arraysize + sizeof(variable) */
      fwrite(log->data[i].name,sizeof(char),50,log->DLfile);
   }

   log->DL = log->DLbuffer1;
   return 0;
}


/** Close the logging file and free buffer memory
\internal chk'd TH 051101
*/
int bt_log_destroy( struct bt_log * log )
{
   /* Was flushDL() */
   void * DLout;
   int Ridx;
   long Rlength;

   if (log->Log_File_Open) {
      /* If our data logging files are open and everything is peachy */
      DLout = log->DL;
      Ridx = log->DLctr;                           /*Record index = full buffer counter*/
      fwrite(&Ridx,sizeof(int),1,log->DLfile);     /*Write Record index as a binary integer*/
      Rlength = log->DLidx * log->data_size;         /*Calculate the Record length*/
      fwrite(&Rlength,sizeof(long),1,log->DLfile);     /*Write the Record length in bytes*/

      fwrite(DLout, log->data_size, log->DLidx, log->DLfile);  /*Write all the data in binary form*/
      log->DLctr++;                                        /*ncrement the record index*/
   }
   
   free(log->data);
   free(log->DLbuffer1);
   free(log->DLbuffer2);
   fclose(log->DLfile);
   return 0;
}



/** Turn the data logger on.
The data logger will not record data until after DLon() is called
\internal chk'd TH 051101
*/
int bt_log_on( struct bt_log * log )
{
   log->Log_Data = 1;
   return 0;
}

/** Turn the data logger off.
When the datalogger is turned off with DLoff(), data logging is paused until
logging is turned back on by DLon()
\internal chk'd TH 051101
*/
int bt_log_off( struct bt_log * log )
{
   log->Log_Data = 0;
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

   /* If data logging is turned on */
   if (!log->Log_Data) return 0;
   
   /* point to the present location in the buffer */
   start = log->DL + log->DLidx * log->data_size; /* ######## MADE THIS CHANGE 08-12 */

   /* copy user data to buffer */
   for(i = 0; i < log->fields; i++) {
      memcpy(start, log->data[i].data, log->data[i].size);
      start += log->data[i].size;
   }
   
   /* Was UpdateDL() */
   /* If our index exceeds our array size */
   log->DLidx++;
   if (log->DLidx >= log->buffersize)
   {
      log->DLidx = 0;                  /*reset our index to zero */

      /*If we are currently pointed to buffer 1*/
      if (log->DL == log->DLbuffer1)
      {
         log->DL = log->DLbuffer2;           /*Point to buffer 2*/
         log->DLwritten = 1;            /*Indicate that buffer 1 is full*/
      }
      else
      {
         log->DL = log->DLbuffer1;           /*Point to buffer 1*/
         log->DLwritten = 2;            /*Indicate that buffer 2 is full*/
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
   void * DLout;
   long Rlength;
   
   /* Make sure data logging is turned on */
   if (!log->Log_Data) return 0;
   /* Make sure some buffer is full */
   if (!log->DLwritten) return 0;

   /*If our data logging files are open and everything is peachy */
   if (log->Log_File_Open)
   {
      /* Point DLout to the full buffer */
      if (log->DLwritten == 1)
         DLout = log->DLbuffer1;
      else if (log->DLwritten == 2)
         DLout = log->DLbuffer2;
      else
         return -1; /* Huh? */
      

      /* Write record index (full buffer counter) as binary integer */
      fwrite(&(log->DLctr),sizeof(int),1,log->DLfile);
      
      /* Calculate / write the record length in bytes */
      Rlength = log->buffersize * log->data_size;
      fwrite(&Rlength,sizeof(long),1,log->DLfile);

      /* Write all the data in binary form */
      fwrite(DLout,log->data_size, log->buffersize, log->DLfile);
      
      /* Increment the record index */
      log->DLctr++;
   }
   
   /* Reset full buffer indicator to zero */
   log->DLwritten=0;
   
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
   
   int numcols;
   int numrows;
   long int fwrite_rowpos;
   
   if (octave) header = 1;

   syslog(LOG_ERR,"DecodeDL: Starting logfile decode from %s -> %s",infile,outfile);

   /*open input file*/
   if ((inf = fopen(infile,"rb"))==NULL) {
      syslog(LOG_ERR,"DecodeDL:Unable to open input file: %s",infile);
      return -1;
   }

   /*open output file*/
   if ((outf = fopen(outfile,"w"))==NULL) {
      syslog(LOG_ERR,"DecodeDL:Unable to open output file: %s\n",outfile);
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
 
