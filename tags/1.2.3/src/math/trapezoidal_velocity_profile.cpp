/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */
/*
 * @trapezoidal_velocity_profile.cpp
 * @date 12/13/2009
 * @author Dan Cody
 *
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
