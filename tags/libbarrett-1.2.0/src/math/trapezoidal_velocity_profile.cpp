/*
 * trapezoidal_velocity_profile.cpp
 *
 *  Created on: Dec 23, 2009
 *      Author: dc
 */


#include <barrett/cdlbt/profile.h>
#include <barrett/math/trapezoidal_velocity_profile.h>


namespace barrett {
namespace math {


TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(double velocity,
		double acceleration, double initialVelocity, double pathLength) :
	v(velocity), a(acceleration), v_0(initialVelocity), l(pathLength),
	impl(NULL)
{
	bt_profile_create(&impl, v, a, v_0, l);
}

TrapezoidalVelocityProfile::~TrapezoidalVelocityProfile()
{
	bt_profile_destroy(impl);
	impl = NULL;
}

double TrapezoidalVelocityProfile::finalT() const
{
	return impl->time_end;
}

double TrapezoidalVelocityProfile::eval(double t) const
{
	double x;
	bt_profile_get(impl, &x, t);
	return x;
}


}
}
