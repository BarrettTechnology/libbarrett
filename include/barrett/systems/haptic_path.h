/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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
 * haptic_path.h
 *
 *  Created on: Jun 21, 2012
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_HAPTIC_PATH_H_
#define BARRETT_SYSTEMS_HAPTIC_PATH_H_


#include <cmath>
#include <cassert>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class HapticPath : public HapticObject {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	HapticPath(const math::Spline<cp_type>* pathPtr, const std::string& sysName = "HapticPath") :
		HapticObject(sysName), path(*pathPtr)
	{
		sLow = path.initialS();
		sHigh = path.finalS();
		sNearest = (sLow + sHigh) / 2.0;

		low = path.eval(sLow);
		nearest = path.eval(sNearest);
		high = path.eval(sHigh);
	}
	virtual ~HapticPath() { mandatoryCleanUp(); }

protected:
	// The WAM is typically configured to velocity fault at 1.5 m/s. With a
	// loop-rate of 500 Hz, this corresponds to a maximum change in position of
	// 0.003 m per execution cycle. It's very unlikely for the WAM to move more
	// than 0.005 m in one EC.
	static const double SEARCH_WIDTH = 0.005;
	static const double TOLERANCE = 0.001;
	virtual void operate() {
		const cp_type& cp = input.getValue();

		minDist = (nearest - cp).norm();
		for (double s = sLow; s <= sHigh; s += SEARCH_WIDTH) {
			nearest = path.eval(s);
			double dist = (nearest - cp).norm();
			if (dist < minDist) {
				minDist = dist;
				sNearest = s;
			}
		}
		nearest = path.eval(sNearest);

		dir = (nearest - cp).normalized();

		depthOutputValue->setData(&minDist);
		directionOutputValue->setData(&dir);
	}

	double minDist;
	cf_type dir;

	const math::Spline<cp_type>& path;
	double sLow, sNearest, sHigh;
	cp_type low, nearest, high;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticPath);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_HAPTIC_PATH_H_ */
