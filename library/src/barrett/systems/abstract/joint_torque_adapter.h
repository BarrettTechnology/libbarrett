/*
 * joint_torque_adapter.h
 *
 *  Created on: Oct 9, 2009
 *      Author: dc
 */

#ifndef JOINT_TORQUE_ADAPTER_H_
#define JOINT_TORQUE_ADAPTER_H_


#include <vector>

#include "../../detail/ca_macro.h"
#include "../../units.h"
#include "./system.h"


namespace barrett {
namespace systems {


// An abstract system that represents the conversion of some InputType into
// JointTorques. Used by SupervisoryController.
class JointTorqueAdapter : public System {
// IO
public:		Output<units::JointTorques> jointTorqueOutput;
protected:	Output<units::JointTorques>::Value* jointTorqueOutputValue;


public:
	JointTorqueAdapter() :
		jointTorqueOutput(&jointTorqueOutputValue) {}
	explicit JointTorqueAdapter(units::JointTorques initialOutputValue) :
		jointTorqueOutput(initialOutputValue, &jointTorqueOutputValue) {}
	virtual ~JointTorqueAdapter() {}

	virtual AbstractInput* getControlInput() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(JointTorqueAdapter);
};


}
}


#endif /* JOINT_TORQUE_ADAPTER_H_ */
