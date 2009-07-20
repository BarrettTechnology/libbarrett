/** Definition of bt_wambot_phys, a physical WAM robot.
 *
 * \file wambot_phys.h
 * \author Christopher Dellin
 * \date 2008-2009
 */

/* Copyright 2008, 2009 Barrett Technology <support@barrett.com> */

/*
 * This file is part of libbarrett.
 *
 * libbarrett is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libbarrett is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libbarrett.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */

#ifndef BT_WAMBOT_PHYS_H
#define BT_WAMBOT_PHYS_H

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <libconfig.h>

#include "wambot.h"
#include "bus.h"

/** A physical WAM robot, which consists of a series of motor controllers on
 *  a bus.
 *
 * A "wambot_phys" is a low-level abstraction of a physical Barrett WAM
 * robot.  This robot uses electric motors and linear steel cable
 * transmissions to convey power to its joints.  Currently, the dynamics
 * of the motors and the transmissions are assumed to be perfectly lienar.
 */
struct bt_wambot_phys
{
   /** We "inherit" from a bt_wambot. */
   struct bt_wambot base;

   /* Constant stuff to be read from config file */
   struct bt_bus * bus; /**< The bt_bus this WAM is on */
   gsl_vector * zeroangle; /**< zero angle enc value */
   gsl_matrix * j2mp; /**< ratios */
   
   /* Constant cache stuff computed from config file stuff above */
   gsl_matrix * m2jp; /**< inverse of j2mp */
   gsl_matrix * j2mt; /**< inverse of j2mp^T */
   
   /* temporary storage locations */
   gsl_vector * mposition; /**< Motor position, in radians */
   gsl_vector * mvelocity; /**< Motor velocity, in radians */
   gsl_vector * mtorque; /**< Motor torques, in Newton-meters */
   
};

/** bt_wambot_phys creation function, given a configuration group.
 *
 * This function creates a new wambot_phys object, given a libconfig
 * configuration group.
 */
struct bt_wambot_phys * bt_wambot_phys_create( config_setting_t * config );

/** bt_wambot_phys destroy function.
 *
 * This function destroys a wambot_phys.
 */
int bt_wambot_phys_destroy( struct bt_wambot_phys * wambot );

#endif /* BT_WAMBOT_PHYS_H */
