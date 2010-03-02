/*
 * haptic_object.h
 *
 *  Created on: Feb 19, 2010
 *      Author: dc
 */

#ifndef HAPTIC_OBJECT_H_
#define HAPTIC_OBJECT_H_


#include "../../detail/ca_macro.h"

#include "../../units.h"
#include "./system.h"
#include "./single_io.h"


namespace barrett {
namespace systems {


class HapticObject : public System, public SingleInput<units::CartesianPosition> {
// IO
public:		Output<double> depthOutput;
protected:	Output<double>::Value* depthOutputValue;
public:		Output<units::CartesianForce> directionOutput;
protected:	Output<units::CartesianForce>::Value* directionOutputValue;

public:
	HapticObject() :
		SingleInput<units::CartesianPosition>(this),
		depthOutput(this, &depthOutputValue),
		directionOutput(this, &directionOutputValue) {}
	virtual ~HapticObject() {}

private:
	DISALLOW_COPY_AND_ASSIGN(HapticObject);
};


}
}


#endif /* HAPTIC_OBJECT_H_ */
