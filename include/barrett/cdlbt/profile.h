/** Definition of bt_profile, a trapezoidal profile mapping from time
 *  to arc-length.
 *
 * \file profile.h
 * \author Christopher Dellin
 * \date 2008-2009
 *
 * \copydoc bt_profile
 */

/* Copyright 2008, 2009 Barrett Technology <support@barrett.com> */

/*
 * This file is part of libbarrett.
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

#ifndef BARRETT_CDLBT_PROFILE_H_
#define BARRETT_CDLBT_PROFILE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <gsl/gsl_vector.h>
#include <gsl/gsl_interp.h>

/** A profile is a mapping from elapsed time to location along an arc-length.
 *
 * For now, there is only one kind of profile, which is a trapezoidal profile
 * (in velocity), which accelerates with constant acceleration up to a given
 * maximum velocity, and later deccelerates with the same acceleration. The
 * profile supports starting at a non-zero velocity, but it always ends at
 * zero velocity.
 *
 * To create a profile, you pass the velocity and acceleration arguments,
 * as well as the initial velocity and arc length.
 */
struct bt_profile {
   double vel; /* Note - we may want this to be different for different dimensions */
   double acc;
   double v_init;
   double time_endup;
   double time_startdown;
   double time_end;
   double s_endup;
   double s_startdown;
   double s_end;
};

/** bt_profile create function */
int bt_profile_create(struct bt_profile ** profileptr, double vel,
                      double acc, double v_init, double length);

/** bt_profile destroy function */
int bt_profile_destroy( struct bt_profile * profile );

/** Function to get the distance s at a given time t from a bt_profile */
int bt_profile_get( struct bt_profile * profile, double * s, double t );

#ifdef __cplusplus
}
#endif
#endif /* BARRETT_CDLBT_PROFILE_H_ */
