/*
 * haptic_ball.h
 *
 *  Created on: Feb 19, 2010
 *      Author: dc
 */

#ifndef HAPTIC_BALL_H_
#define HAPTIC_BALL_H_


#include <cmath>

#include "../detail/ca_macro.h"
#include "../units.h"
#include "./abstract/haptic_object.h"


namespace barrett {
namespace systems {


class HapticBall : public HapticObject {
public:
	HapticBall(units::CartesianPosition::type center, double radius) :
		c(center), r(radius) {}
	virtual ~HapticBall() {}

protected:
	virtual void operate() {
		units::CartesianForce::type error = input.getValue() - c;
		double mag = error.norm();
		//double mag = std::sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);

		if (mag < r) {  // if we are inside the ball
			depthOutputValue->setValue(r-mag);

			// unit vector pointing towards the surface of the ball
			directionOutputValue->setValue(error/mag);
		} else {
			depthOutputValue->setValue(0.0);
//			directionOutputValue->setValue(units::CartesianForce::type::Zero());
			directionOutputValue->setValue(units::CartesianForce::type(0.0));
		}
	}

	units::CartesianPosition::type c;
	double r;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticBall);
};


}
}


#endif /* HAPTIC_BALL_H_ */
