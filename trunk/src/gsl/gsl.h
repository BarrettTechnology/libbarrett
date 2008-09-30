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

/* Note - this does not check for the size of the buffer! */
char * bt_gsl_vector_sprintf(char * buffer, gsl_vector * vector);

/* This should really go somewhere else ... */
int bt_gsl_config_get_double(config_setting_t * setting, double * result);

#endif /* BT_GSL_H */
