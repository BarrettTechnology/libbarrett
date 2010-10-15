/*
 * haptic_ball.h
 *
 *  Created on: Feb 19, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_HAPTIC_BALL_H_
#define BARRETT_SYSTEMS_HAPTIC_BALL_H_


#include <cmath>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class HapticBall : public HapticObject {
public:
	HapticBall(units::CartesianPosition::type center, double radius) :
		c(center), r(radius), keepOutside(true) /* doesn't matter how this is initialized, it'll fix itself */ {}
	virtual ~HapticBall() {}

protected:
	virtual void operate() {
		units::CartesianForce::type error = input.getValue() - c;
		double mag = error.norm();

		bool outside = mag > r;

		// if we are inside the ball and we shouldn't be, or we are outside the ball and we shouldn't be
		if ((keepOutside && !outside)  || (!keepOutside && outside)) {
			if (math::abs(r-mag) > 0.02) {
				keepOutside = !keepOutside;
			} else {
				depthOutputValue->setValue(r-mag);

				// unit vector pointing towards the surface of the ball
				directionOutputValue->setValue(error/mag);
				return;
			}
		}

		depthOutputValue->setValue(0.0);
		directionOutputValue->setValue(units::CartesianForce::type(0.0));
	}

	units::CartesianPosition::type c;
	double r;
	bool keepOutside;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticBall);
};


}
}


#endif /* BARRETT_SYSTEMS_HAPTIC_BALL_H_ */
