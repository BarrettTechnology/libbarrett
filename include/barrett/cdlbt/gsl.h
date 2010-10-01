/** Definition of a set of GSL helper functions, including the
 *  cross-product vector formatting, and libconfig parsing functions.
 *
 * \file gsl.h
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

#ifndef BARRETT_CDLBT_GSL_H_
#define BARRETT_CDLBT_GSL_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <libconfig.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/** Perform a vector cross-product, res += a x b.
 *
 * \note This function will add the result to the vector specified by res.
 *       Therefore, make sure to set the output vector to zero if you want
 *       the result explicitly!
 *
 * \param[in] a First vector
 * \param[in] b Second vector (to be crossed into a)
 * \param[out] res Result vector to add to
 * \retval 0 Success
 */
int bt_gsl_cross(gsl_vector * a, gsl_vector * b, gsl_vector * res);


/** Format the vector into the buffer.
 *
 * The format used is "<aaa.aaaa,bbb.bbbb,ccc.cccc>", with vector values
 * specified by the format string %8.4f.
 *
 * \note This function assumes the buffer is sufficiently large to hold the
 *       string.
 *
 * \param[out] buffer The character buffer to write the result to
 * \param[in] vector The vector to format
 * \returns The value of buffer
 */
char * bt_gsl_vector_sprintf(char * buffer, gsl_vector * vector);


/** Retrieve a double from a libconfig setting.
 *
 * \param[in] setting The setting to use
 * \param[out] result The location into which to save the value
 * \retval 0 Success
 * \retval -1 The setting does not exist
 * \retval -2 The setting's value is not convertable to double
 */
int bt_gsl_config_get_double(config_setting_t * setting, double * result);


/** Retrieve a double from a named setting in a libconfig group.
 *
 * \param[in] grp The group to use
 * \param[in] name The name of the setting to find
 * \param[out] result The location into which to save the value
 * \retval 0 Success
 * \retval -1 The named setting does not exist
 * \retval -2 The setting's value is not convertable to double
 */
int bt_gsl_config_double_from_group(config_setting_t * grp, char * name,
                                    double * result);


/** Fill a vector from a libconfig array or list.
 *
 * \note The vector must already be allocated.
 *
 * \param[out] vec The vector to fill
 * \param[in] parent The group to use
 * \param[in] name The name of the array or list in the parent
 * \retval 0 Success
 * \retval -1 The named setting does not exist, or is not an array or list
 *            with the same length as the vector
 * \retval -2 At least one of the array or list's values is not convertable
 *            to double
 */
int bt_gsl_fill_vector_cfggroup(gsl_vector * vec, config_setting_t * parent,
                       const char * name);


int bt_gsl_fill_vector_cfgarray(gsl_vector * vec, config_setting_t * array);


int bt_gsl_fill_vector(gsl_vector * vec, ...);


/** Fill a matrix from a libconfig list of arrays or lists.
 *
 * \note The matrix must already be allocated.
 *
 * \param[out] mat The matrix to fill
 * \param[in] parent The group to use
 * \param[in] name The name of the array or list in the parent
 * \retval 0 Success
 * \retval -1 The named setting does not exist, or is not a list of arrays or
 *            lists with the same dimensions as the matrix
 * \retval -2 At least one of the read values is not convertable to double
 */
int bt_gsl_fill_matrix(gsl_matrix * mat, config_setting_t * parent,
                       const char * name);


#ifdef __cplusplus
}
#endif
#endif /* BARRETT_CDLBT_GSL_H_ */
