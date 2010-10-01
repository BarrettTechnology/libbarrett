/** Definition of bt_interp, GSL interpolator type with support for
 *  2-point trajectories, as well as natural or slope-drive endpoints.
 *
 * \file interp.h
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

/** \file interp.h
 *
 * \section sec_intro Introduction
 *
 * A bt_interp type is a GSL interpolator type based on GSL's interpolation
 * framework.  For more information about this framework, see the
 * "Interpolation" subsection of the GSL manual, located here:
 * <http://www.gnu.org/software/gsl/manual/html_node/Interpolation.html>
 *
 * A bt_interp type is similar to the standard GSL gsl_interp_cspline
 * cubic spline type, with a number of enhancements.
 *
 * First, while a gsl_interp_cspline has a minimum number of 3 points, a
 * bt_interp also supports 2-point (linear) trajectories.
 *
 * Second, while a gsl_interp_cspline only supports natural boundary
 * conditions (i.e. zero-concavity), a bt_interp also supports explicitly
 * setting the slope at either endpoint.
 */

#ifndef BARRETT_CDLBT_INTERP_H_
#define BARRETT_CDLBT_INTERP_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <gsl/gsl_interp.h>

   
/** The type of a bt_interp endpoint, either natural or slope.
 *
 * An endpoint of type BT_INTERP_NATURAL has a zero concavity (or
 * second-derivative), while an endpoint of type BT_INTERP_SLOPE has an
 * explicitly defined slope at the endpoint.
 */
enum bt_interp_type {
   BT_INTERP_NATURAL,
   BT_INTERP_SLOPE
};


/** This is the GSL interpolator type object to be used with
 * gsl_interp_alloc() when a bt_interp type is to be used. */
const gsl_interp_type * bt_interp;


/** This function is used to set the types of the two endpoints before the
 *  interpolator is initialized. */
int bt_interp_set_type(gsl_interp * interp, enum bt_interp_type ltype,
                       enum bt_interp_type rtype);


/** This function is used to set the explicit slopes of the two endpoints
 *  before the interpolator is initialized.
 *
 * \note The values specified here will only be used if the endpoint is also
 *       of type BT_INTERP_SLOPE.
 */
int bt_interp_set_slopes(gsl_interp * interp, double lslope, double rslope);


#ifdef __cplusplus
}
#endif
#endif /* BARRETT_CDLBT_INTERP_H_ */
