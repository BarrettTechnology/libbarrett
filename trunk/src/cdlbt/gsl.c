/** Implementation of a set of GSL helper functions, including the
 *  cross-product vector formatting, and libconfig parsing functions.
 *
 * \file gsl.c
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009
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

#include <string.h>
#include <stdarg.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <libconfig.h>

#include <barrett/cdlbt/gsl.h>


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


char * bt_gsl_vector_sprintf(char * buf, gsl_vector * vec)
{
   int i;
   buf[0] = '<';
   buf[1] = 0;
   for (i=0; i<vec->size; i++)
      sprintf(buf+strlen(buf), "%8.4f,", gsl_vector_get(vec,i));
   buf[strlen(buf)-1] = 0;
   strcat(buf,">");
   return buf;
}


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
         return -2;
   }
}


int bt_gsl_config_double_from_group(config_setting_t * grp, char * name,
                                    double * result)
{
   config_setting_t * setting;
   setting = config_setting_get_member(grp, name);
   if (setting == 0)
      return -1;
   switch (config_setting_type(setting))
   {
      case CONFIG_TYPE_INT:
         (*result) = (double) config_setting_get_int(setting);
         break;
      case CONFIG_TYPE_FLOAT:
         (*result) = config_setting_get_float(setting);
         break;
      default:
         return -2;
   }
   return 0;
}


int bt_gsl_fill_vector_cfggroup(gsl_vector * vec, config_setting_t * parent,
                       const char * name)
{
   config_setting_t * array;
   array = config_setting_get_member( parent, name );
   if (!array) return -1;
   return bt_gsl_fill_vector_cfgarray(vec, array);
}


int bt_gsl_fill_vector_cfgarray(gsl_vector * vec, config_setting_t * array)
{
   int i;
   if (   config_setting_type(array) != CONFIG_TYPE_ARRAY
       && config_setting_type(array) != CONFIG_TYPE_LIST ) return -1;
   if (config_setting_length(array) != vec->size) return -1;
   for (i=0; i<vec->size; i++)
   {
      config_setting_t * element;
      element = config_setting_get_elem(array,i);
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


int bt_gsl_fill_vector(gsl_vector * vec, ...)
{
   int i;
   va_list ap;

   va_start(ap,vec);
   for (i=0; i<vec->size; i++)
      gsl_vector_set(vec,i,va_arg(ap,double));
   va_end(ap);
   
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
