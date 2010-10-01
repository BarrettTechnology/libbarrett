/*
 * tool_position.h
 *
 *  Created on: Jan 15, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_TOOL_POSITION_H_
#define BARRETT_SYSTEMS_TOOL_POSITION_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>


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


#endif /* BARRETT_SYSTEMS_TOOL_POSITION_H_ */
