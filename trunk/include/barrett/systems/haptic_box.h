/*
 * haptic_box.h
 *
 *  Created on: Apr 16, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_HAPTIC_BOX_H_
#define BARRETT_SYSTEMS_HAPTIC_BOX_H_


#include <cmath>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class HapticBox : public HapticObject {
public:
	HapticBox(units::CartesianPosition::type center, double xSize, double ySize, double zSize) :
//		c(center), x(xSize/2), y(ySize/2), z(zSize/2), pos() {}
		c(center), size(xSize/2, ySize/2, zSize/2), inBox(false), index(-1) {}
	virtual ~HapticBox() {}

protected:
	virtual void operate() {
		pos = input.getValue() - c;

		if ((pos.cwise().abs().cwise() < size).all()) {  // if we are inside the box
			if ( !inBox ) {  // if we weren't in the box last time
				// find out what side we entered on
				(size - pos.cwise().abs()).minCoeff(&index);
			}

			depthOutputValue->setValue(size[index] - math::abs(pos[index]));
			directionOutputValue->setValue(math::sign(pos[index]) * units::CartesianForce::type::Unit(index));

			inBox = true;
		} else {
			inBox = false;
			depthOutputValue->setValue(0.0);
			directionOutputValue->setValue(units::CartesianForce::type(0.0));
		}
	}

	units::CartesianPosition::type c;
//	double x,y,z;
	math::Vector<3>::type size;

	// temporaries
	units::CartesianForce::type pos;
	bool inBox;
	int index;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticBox);
};


}
}


#endif /* BARRETT_SYSTEMS_HAPTIC_BOX_H_ */
