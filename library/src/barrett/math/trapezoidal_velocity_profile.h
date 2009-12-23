/*
 * trapezoidal_velocity_profile.h
 *
 *  Created on: Dec 23, 2009
 *      Author: dc
 */

#ifndef TRAPEZOIDAL_VELOCITY_PROFILE_H_
#define TRAPEZOIDAL_VELOCITY_PROFILE_H_


#include "../detail/ca_macro.h"


// forward declaration from <barrett/profile/profile.h>
struct bt_profile;


namespace barrett {
namespace math {


class TrapezoidalVelocityProfile {
public:
	TrapezoidalVelocityProfile(double velocity, double acceleration,
			double initialVelocity, double pathLength);
	~TrapezoidalVelocityProfile();

	double finalT() const;

	double eval(double t) const;

protected:
	double v, a, v_0, l;
	struct bt_profile* impl;

private:
	DISALLOW_COPY_AND_ASSIGN(TrapezoidalVelocityProfile);
};


}
}


#endif /* TRAPEZOIDAL_VELOCITY_PROFILE_H_ */
