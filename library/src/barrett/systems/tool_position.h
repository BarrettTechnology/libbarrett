/*
 * tool_position.h
 *
 *  Created on: Jan 15, 2010
 *      Author: dc
 */

#ifndef TOOL_POSITION_H_
#define TOOL_POSITION_H_


#include "../detail/ca_macro.h"
#include "../units.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"
#include "./kinematics_base.h"


namespace barrett {
namespace systems {


template<size_t DOF>
class ToolPosition : public System, public KinematicsInput<DOF>,
					 public SingleOutput<units::CartesianPosition::type> {
public:
	ToolPosition() :
		KinematicsInput<DOF>(this),
		SingleOutput<units::CartesianPosition::type>(this) {}
	virtual ~ToolPosition() {}

protected:
	virtual void operate() {
		this->outputValue->setValue(
				units::CartesianPosition::type(
						this->kinInput.getValue()->impl->tool->origin_pos));
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ToolPosition);
};


}
}


#endif /* TOOL_POSITION_H_ */
