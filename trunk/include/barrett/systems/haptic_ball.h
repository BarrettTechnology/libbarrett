/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * haptic_ball.h
 *
 *  Created on: Feb 19, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_HAPTIC_BALL_H_
#define BARRETT_SYSTEMS_HAPTIC_BALL_H_


#include <cmath>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class HapticBall : public HapticObject {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	HapticBall(const cp_type& center, double radius, const std::string& sysName = "HapticBall") :
		HapticObject(sysName),
		c(center), r(radius), keepOutside(true) /* doesn't matter how this is initialized, it'll fix itself */,
		depth(0.0), error(0.0) {}
	virtual ~HapticBall() { mandatoryCleanUp(); }

	void setCenter(const cp_type& newCenter){
		BARRETT_SCOPED_LOCK(getEmMutex());
		c = newCenter;
	}
	void setRadius(double newRadius){
		BARRETT_SCOPED_LOCK(getEmMutex());
		r = newRadius;
	}

	const cp_type& getCenter() const { return c; }
	double getRadius() const { return r; }

protected:
	virtual void operate() {
		error = input.getValue() - c;
		double mag = error.norm();

		bool outside = mag > r;

		// if we are inside the ball and we shouldn't be, or we are outside the ball and we shouldn't be
		if ((keepOutside && !outside)  ||  (!keepOutside && outside)) {
			depth = r - mag;
			if (math::abs(depth) > 0.02) {
				keepOutside = !keepOutside;

				depth = 0.0;
				error.setZero();
			} else {
				// unit vector pointing towards the surface of the ball
				error /= mag;
			}
		} else {
			depth = 0.0;
			error.setZero();
		}

		depthOutputValue->setData(&depth);
		directionOutputValue->setData(&error);
	}

	cp_type c;
	double r;
	bool keepOutside;

	double depth;
	cf_type error;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticBall);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_HAPTIC_BALL_H_ */
