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
#include <vector>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class HapticPath : public HapticObject {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

	static const double COARSE_STEP = 0.01;
	static const double FINE_STEP = 0.0001;

public:		System::Output<cp_type> tangentDirectionOutput;
protected:	System::Output<cp_type>::Value* tangentDirectionOutputValue;

public:
	HapticPath(const std::vector<cp_type>& path,
			const std::string& sysName = "HapticPath") :
		HapticObject(sysName),
		tangentDirectionOutput(this, &tangentDirectionOutputValue),
		nearestIndex(0), spline(NULL)
	{
		// Sample the path
		cp_type prev = path[0];
		for (size_t i = 0; i < path.size(); ++i) {
			if ((path[i] - prev).norm() > COARSE_STEP) {
				coarsePath.push_back(path[i]);
				prev = path[i];
			}
		}
		spline = new math::Spline<cp_type>(coarsePath);
	}

	virtual ~HapticPath() {
		mandatoryCleanUp();
		delete spline;
	}

protected:
	virtual void operate() {
		const cp_type& cp = input.getValue();

		// Coarse search
		minDist = (coarsePath[nearestIndex] - cp).norm();
		for (size_t i = 0; i < coarsePath.size(); ++i) {
			double dist = (coarsePath[i] - cp).norm();
			if (dist < minDist) {
				minDist = dist;
				nearestIndex = i;
			}
		}

		// Fine search
		// TODO(dc): Can we do this without relying on Spline's implementation?
		double sNearest = spline->getImplementation()->ss[nearestIndex];
		double sLow = sNearest - COARSE_STEP;
		double sHigh = sNearest + COARSE_STEP;
		for (double s = sLow; s <= sHigh; s += FINE_STEP) {
			double dist = (spline->eval(s) - cp).norm();
			if (dist < minDist) {
				minDist = dist;
				sNearest = s;
			}
		}

		dir = (spline->eval(sNearest) - cp).normalized();
		tangentDir = spline->evalDerivative(sNearest).normalized();

		depthOutputValue->setData(&minDist);
		directionOutputValue->setData(&dir);
		tangentDirectionOutputValue->setData(&tangentDir);
	}

	double minDist;
	size_t nearestIndex;
	cf_type dir;
	cp_type tangentDir;

	std::vector<cp_type> coarsePath;
	math::Spline<cp_type>* spline;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticPath);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_HAPTIC_PATH_H_ */
