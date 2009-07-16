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

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <string.h>

/* Vector cross product.
 * Make sure to clear res before this call; it will just add the result in.
 * Cannot be performed in-place! */
int bt_gsl_cross( gsl_vector * a, gsl_vector * b, gsl_vector * res )
{
   *gsl_vector_ptr(res,0) += gsl_vector_get(a,1)*gsl_vector_get(b,2)
                           - gsl_vector_get(a,2)*gsl_vector_get(b,1);
   *gsl_vector_ptr(res,1) += gsl_vector_get(a,2)*gsl_vector_get(b,0)
                           - gsl_vector_get(a,0)*gsl_vector_get(b,2);
   *gsl_vector_ptr(res,2) += gsl_vector_get(a,0)*gsl_vector_get(b,1)
                           - gsl_vector_get(a,1)*gsl_vector_get(b,0);
   return 0;
}

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

/* A convenience function for libconfig, getting a double from a group */
int bt_gsl_config_double_from_group( config_setting_t * grp, char * name, double * puthere )
{
   config_setting_t * setting;
   setting = config_setting_get_member( grp, name );
   if (setting == NULL)
      return -1;
   switch (config_setting_type(setting))
   {
      case CONFIG_TYPE_INT:
         (*puthere) = (double) config_setting_get_int(setting);
         break;
      case CONFIG_TYPE_FLOAT:
         (*puthere) = config_setting_get_float(setting);
         break;
      default:
         return -2;
   }
   return 0;
}

/* A couple of convenience / glue functions from gsl to libconfig */
int bt_gsl_fill_vector(gsl_vector * vec, config_setting_t * parent, const char * name)
{
   int i;
   config_setting_t * child;
   child = config_setting_get_member( parent, name );
   if (child == NULL) return -1;
   if (   config_setting_type(child) != CONFIG_TYPE_ARRAY
       && config_setting_type(child) != CONFIG_TYPE_LIST ) return -1;
   if (config_setting_length(child) != vec->size) return -1;
   for (i=0; i<vec->size; i++)
   {
      config_setting_t * element;
      element = config_setting_get_elem(child,i);
      switch (config_setting_type(element))
      {
         case CONFIG_TYPE_INT:
            gsl_vector_set(vec,i,config_setting_get_int(element));
            break;
         case CONFIG_TYPE_FLOAT:
            gsl_vector_set(vec,i,config_setting_get_float(element));
            break;
         default:
            return -1;
      }
   }
   return 0;
}
int bt_gsl_fill_matrix(gsl_matrix * mat, config_setting_t * parent, const char * name)
{
   int i, j;
   config_setting_t * rows;
   rows = config_setting_get_member( parent, name );
   if (rows == NULL) return -1;
   if (config_setting_type(rows) != CONFIG_TYPE_LIST) return -1;
   if (config_setting_length(rows) != mat->size1) return -1;
   for (i=0; i<mat->size1; i++)
   {
      config_setting_t * row;
      row = config_setting_get_elem(rows,i);
      if (   config_setting_type(row) != CONFIG_TYPE_ARRAY
          && config_setting_type(row) != CONFIG_TYPE_LIST ) return -1;
      if (config_setting_length(row) != mat->size2) return -1;
      for (j=0; j<mat->size2; j++)
      {
         config_setting_t * element;
         element = config_setting_get_elem(row,j);
         switch (config_setting_type(element))
         {
            case CONFIG_TYPE_INT:
               gsl_matrix_set(mat,i,j,config_setting_get_int(element));
               break;
            case CONFIG_TYPE_FLOAT:
               gsl_matrix_set(mat,i,j,config_setting_get_float(element));
               break;
            default:
               return -1;
         }
      }
   }
   return 0;
}

