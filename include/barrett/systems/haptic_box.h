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
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	HapticBox(const cp_type& center, const math::Vector<3>::type& size,
			const std::string& sysName = "HapticBox") :
		HapticObject(sysName),
		c(center), halfSize(0.0), inBox(false), index(-1), keepOutside(true),
		depth(0.0), dir(0.0)
	{
		setSize(size);
	}
	virtual ~HapticBox() { mandatoryCleanUp(); }

	void setCenter(const cp_type& newCenter) {
		BARRETT_SCOPED_LOCK(getEmMutex());
		c = newCenter;
	}
	void setSize(const math::Vector<3>::type& newSize) {
		BARRETT_SCOPED_LOCK(getEmMutex());
		halfSize = newSize / 2.0;
	}

	const cp_type& getCenter() const { return c; }
	math::Vector<3>::type getSize() const { return halfSize * 2.0; }

protected:
	virtual void operate() {
		pos = input.getValue() - c;

		bool outside = (pos.cwise().abs().cwise() > halfSize).any();

		// if we are inside the box and we shouldn't be
		if (keepOutside  &&  !outside) {
			if ( !inBox ) {  // if we weren't in the box last time
				// find out what side we entered on
				(halfSize - pos.cwise().abs()).minCoeff(&index);
			}

			depth = halfSize[index] - math::abs(pos[index]);
			if (depth > 0.02) {
				keepOutside = !keepOutside;

				depth = 0.0;
				dir.setZero();
			} else {
				dir = math::sign(pos[index]) * cf_type::Unit(index);

				inBox = true;
			}
		} else {
			inBox = false;

			// if we are outside the box and we shouldn't be
			if (!keepOutside  &&  outside) {
				dir = halfSize - pos.cwise().abs();
				dir = dir.cwise() * (dir.cwise() < 0.0).cast<double>();
				dir = dir.cwise() * math::sign(pos);

				depth = dir.norm();
				if (depth > 0.02) {
					keepOutside = !keepOutside;

					depth = 0.0;
					dir.setZero();
				} else {
					dir /= depth;
				}
			} else {
				depth = 0.0;
				dir.setZero();
			}
		}

		depthOutputValue->setData(&depth);
		directionOutputValue->setData(&dir);
	}

	cp_type c;
	math::Vector<3>::type halfSize;

	// state & temporaries
	cf_type pos;
	bool inBox;
	int index;
	bool keepOutside;

	double depth;
	cf_type dir;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticBox);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}

#endif /* BARRETT_SYSTEMS_HAPTIC_BOX_H_ */
