/** Low-level abstraction of a WAM robot.
 *
 * \file wambot.h
 * \author Christopher Dellin
 * \date 2008-2009
 *
 * \copydoc bt_wambot
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

#ifndef BT_WAMBOT_H
#define BT_WAMBOT_H

#include <gsl/gsl_vector.h>

/** Update a wambot (get position, velocity, etc) */
#define bt_wambot_update(wb) (wb)->update((wb))

/** Set the joint torque to wambot */
#define bt_wambot_setjtor(wb) (wb)->setjtor((wb))

/** A simple WAM robot, which supports getting joint positions
 *  (and velocities, etc) and setting joint torques.
 *
 * A "wambot" is a low-level abstraction of a Barrett WAM robot.  There may
 * be several implementations of the wambot, including a physical wambot and
 * a simulated wambot.
 *
 * This is a generic wambot structure. Implementations should "subclass" from
 * this structure.
 */
struct bt_wambot
{
   /** The number of degrees-of-freedom (to be read from config file). */
   int dof;

   /** Approximate home location (to be read from config file). */
   gsl_vector * home; /* rad */
   
   /* Communication with actuators, interface with bot */

   /** Joint position vector as read from the wambot. */
   gsl_vector * jposition; /* rad */

   /** Joint velocity vector as read from the wambot. */
   gsl_vector * jvelocity; /* rad/s */

   /** Joint torque vector to send to the wambot. */
   gsl_vector * jtorque; /* Nm */

   /** Function to grab the latest position and/or velocity from the wambot.
    */
   int (*update)( struct bt_wambot * wambot );

   /** Function to send the current joint torque to the wambot. */
   int (*setjtor)( struct bt_wambot * wambot );
};

#endif /* BT_WAMBOT_H */
