/* ======================================================================== *
 *  Module ............. libbt
 *  File ............... gsl.h
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

#ifndef BT_GSL_H
#define BT_GSL_H

#include <libconfig.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/* Vector cross product.
 * Make sure to clear res before this call; it will just add the result in.
 * Cannot be performed in-place! */
int bt_gsl_cross( gsl_vector * a, gsl_vector * b, gsl_vector * res );

/* Note - this does not check for the size of the buffer! */
char * bt_gsl_vector_sprintf(char * buffer, gsl_vector * vector);

/* This should really go somewhere else ... */
int bt_gsl_config_get_double(config_setting_t * setting, double * result);

/* A convenience function for libconfig, getting a double from a group */
int bt_gsl_config_double_from_group( config_setting_t * grp, char * name, double * puthere );

/* Note: vector must already be allocated */
int bt_gsl_fill_vector(gsl_vector * vec, config_setting_t * parent, const char * name);

/* Note: matrix must already be allocated */
int bt_gsl_fill_matrix(gsl_matrix * mat, config_setting_t * parent, const char * name);

#endif /* BT_GSL_H */
