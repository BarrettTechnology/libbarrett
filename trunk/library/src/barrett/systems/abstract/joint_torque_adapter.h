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
template<size_t N>
class JointTorqueAdapter : public System {
// IO
public:		Output<units::JointTorques<N> > jointTorqueOutput;
protected:
	typename Output<units::JointTorques<N> >::Value* jointTorqueOutputValue;


public:
	JointTorqueAdapter() :
		jointTorqueOutput(&jointTorqueOutputValue) {}
	explicit JointTorqueAdapter(units::JointTorques<N> initialOutputValue) :
		jointTorqueOutput(initialOutputValue, &jointTorqueOutputValue) {}
	virtual ~JointTorqueAdapter() {}

	virtual AbstractInput* getControlInput() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(JointTorqueAdapter);
};


}
}


#endif /* JOINT_TORQUE_ADAPTER_H_ */
