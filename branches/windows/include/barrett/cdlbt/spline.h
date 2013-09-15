/** Definition of bt_spline, an n-dimensional vector interpolator.
 *
 * \file cdlbt/spline.h
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

/** \file cdlbt/spline.h
 *
 * \section sec_intro Introduction
 *
 * A bt_spline is an abstraction over a set of GSL interpolators, one for each
 * dimension of the spline's space.  Points can be added to the spline as
 * vectors, and the spline can then be interpolated over in a control loop.
 * The spline can be parameterized by arclength or by an external parameter.
 */

#ifndef BARRETT_CDLBT_SPLINE_H_
#define BARRETT_CDLBT_SPLINE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <gsl/gsl_vector.h>
#include <gsl/gsl_interp.h>


/** A bt_spline has two different parameterization modes, arclen and
 *  external.
 *
 * In BT_SPLINE_MODE_ARCLEN, the spline calculates the parameter s itself
 * once it is initialized, based on the linear distance between the points.
 * The resulting length is then available in the bt_spline::length variable.
 * In this mode the s parameter in the bt_spline_add() function is ignored.
 *
 * In BT_SPLINE_MODE_EXTERNAL, an external parameter is used.  This value is
 * saved along with each point through the s parameter of bt_spline_add().
 * The values start at zero when the starting point is added, and the
 * subsequent values used must be monotonically increasing as the points are
 * added. This mode is most often used with elapsed time serving as the
 * external parameter.
 */
enum bt_spline_mode
{
   BT_SPLINE_MODE_ARCLEN, /* The spline uses computed arc-length */
   BT_SPLINE_MODE_EXTERNAL /* The spline uses an external parameter */
};


/** Spline data, including the dimensionality, array of saved points and
 *  parameters, and GSL interpolators / accelerators used when retrieving
 *  points.*/
struct bt_spline
{
   enum bt_spline_mode mode;
   int dimension;
   int npoints;
   double * ss;
   double length;
   double ** points;
   gsl_interp_accel * acc;
   gsl_interp ** interps;
};


/** Create a bt_spline object from a given start location.
 *
 * This function creates a new bt_spline object, with the given vector as its
 * starting point.
 *
 * \param[out] splineptr The bt_spline object on success, or 0 on failure
 * \param[in] start Starting point
 * \param[in] mode Spline mode; either arclen or external.
 * \retval 0 Success
 */
int bt_spline_create(struct bt_spline ** splineptr, const gsl_vector * start,
                     enum bt_spline_mode mode);


/** Add a point to a bt_spline.
 *
 * This function is called iteratively once a bt_spline has been created to
 * add points to the end of the spline.  If the mode is
 * BT_SPLINE_MODE_EXTERNAL, care must be take to ensure that the parameter
 * s is monotonically increasing.  If the mode is BT_SPLINE_MODE_ARCLEN, the
 * s parameter is ignored.
 *
 * \param[in] spline The bt_spline object to which to add the point
 * \param[in] vec The point to add
 * \param[in] s The values of the external parameter at this point
 * \retval 0 Success
 */
int bt_spline_add(struct bt_spline * spline, const gsl_vector * vec, double s);


/** Initialize the bt_spline after adding all points.
 *
 * Once points are added to the spline using bt_spline_add(), the spline is
 * initialized.  This will create the interpolating objects for each
 * dimension.  If the start argument is passed, the original starting point
 * given in bt_spline_create() will be replaced before any calculations
 * take place.  If the direction argument is passed, the initial slopes to
 * each interpolator will be proportional to the normalized direction
 * vector's values.
 *
 * \note The direction vector, if passed and non-zero, will be normalized
 *       in place.
 *
 * \param[in] spline The bt_spline object to initialize
 * \param[in] start A vector to replace the original start vector with,
 *                  or 0 to skip this step
 * \param[in] direction A vector to use as the direction of the new spline
 * \retval 0 Success
 */
int bt_spline_init(struct bt_spline * spline, gsl_vector * start,
                   gsl_vector * direction);


/** Destroy a bt_spline object.
 *
 * This function destroys a bt_spline object created by
 * bt_spline_create().
 *
 * \param[in] spline The bt_spline object to destroy
 * \retval 0 Success
 */
int bt_spline_destroy(struct bt_spline * spline);


/** Retrieve the interpolated vector at a given parameter value.
 *
 * This function is designed to be called in a control loop to retrieve the
 * interpolated vector at the paramater value s.
 *
 * \param[in] spline The bt_spline object to use
 * \param[in] result The vector at the given paramter value
 * \param[in] s The parameter value to use
 * \retval 0 Success
 */
int bt_spline_get(struct bt_spline * spline, gsl_vector * result, double s);


#ifdef __cplusplus
}
#endif
#endif /* BARRETT_CDLBT_SPLINE_H_ */
