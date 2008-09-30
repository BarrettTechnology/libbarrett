/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... gsl.c
 *  Author ............. Christopher Dellin
 *  Creation Date ...... 2008 Sept 15
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 * Copyright (C) 2008   Barrett Technology <support@barrett.com>
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *
 * ======================================================================== */

#include "gsl.h"
#include "libconfig.h"

#include <string.h>

/* Note - based on Traveler Hauptman's sprint_vn() from btmath */
char * bt_gsl_vector_sprintf(char * buf, gsl_vector * vec)
{
   int i;
   buf[0] = '<';
   buf[1] = 0;
   for (i=0; i<vec->size; i++)
      sprintf( buf+strlen(buf), "%8.4f,", gsl_vector_get(vec,i) );
   buf[strlen(buf)-1] = 0;
   strcat(buf,">");
   return buf;
}

/* Grab a double from a setting */
int bt_gsl_config_get_double(config_setting_t * setting, double * result)
{
   if (!setting)
      return -1;
   switch (config_setting_type(setting))
   {
      case CONFIG_TYPE_INT:
         *result = config_setting_get_int(setting);
         return 0;
      case CONFIG_TYPE_FLOAT:
         *result = config_setting_get_float(setting);
         return 0;
      default:
         return -1;
   }
}

