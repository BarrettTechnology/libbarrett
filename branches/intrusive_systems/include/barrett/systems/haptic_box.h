/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

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
 * haptic_box.h
 *
 *  Created on: Apr 16, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_HAPTIC_BOX_H_
#define BARRETT_SYSTEMS_HAPTIC_BOX_H_


#include <cmath>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class HapticBox : public HapticObject {
public:
	HapticBox(units::CartesianPosition::type center, double xSize, double ySize, double zSize) :
		c(center), size(xSize/2, ySize/2, zSize/2), inBox(false), index(-1), keepOutside(true) {}
	virtual ~HapticBox() {}

protected:
	virtual void operate() {
		pos = input.getValue() - c;

		bool outside = (pos.cwise().abs().cwise() > size).any();

		// if we are inside the box and we shouldn't be
		if (keepOutside  &&  !outside) {
			if ( !inBox ) {  // if we weren't in the box last time
				// find out what side we entered on
				(size - pos.cwise().abs()).minCoeff(&index);
			}

			double depth = size[index] - math::abs(pos[index]);
			if (depth > 0.02) {
				keepOutside = !keepOutside;
			} else {
				depthOutputValue->setValue(depth);
				directionOutputValue->setValue(math::sign(pos[index]) * units::CartesianForce::type::Unit(index));

				inBox = true;
				return;
			}
		}
		inBox = false;

		// if we are outside the box and we shouldn't be
		if (!keepOutside  &&  outside) {
			units::CartesianForce::type dir = size - pos.cwise().abs();
			dir = dir.cwise() * (dir.cwise() < 0.0).cast<double>();
			dir = dir.cwise() * math::sign(pos);

			double depth = dir.norm();
			if (depth > 0.02) {
				keepOutside = !keepOutside;
			} else {
				depthOutputValue->setValue(depth);
				directionOutputValue->setValue(dir/depth);
				return;
			}
		}

		depthOutputValue->setValue(0.0);
		directionOutputValue->setValue(units::CartesianForce::type(0.0));
	}

	units::CartesianPosition::type c;
	math::Vector<3>::type size;

	// state & temporaries
	units::CartesianForce::type pos;
	bool inBox;
	int index;
	bool keepOutside;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticBox);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_HAPTIC_BOX_H_ */
