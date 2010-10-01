/*
 * trapezoidal_velocity_profile.h
 *
 *  Created on: Dec 23, 2009
 *      Author: dc
 */

#ifndef BARRETT_MATH_TRAPEZOIDAL_VELOCITY_PROFILE_H_
#define BARRETT_MATH_TRAPEZOIDAL_VELOCITY_PROFILE_H_


#include <barrett/detail/ca_macro.h>


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

	typedef double result_type;  ///< For use with boost::bind().
	result_type operator() (double t) const {
		return eval(t);
	}

protected:
	double v, a, v_0, l;
	struct bt_profile* impl;

private:
	DISALLOW_COPY_AND_ASSIGN(TrapezoidalVelocityProfile);
};


}
}


#endif /* BARRETT_MATH_TRAPEZOIDAL_VELOCITY_PROFILE_H_ */
