/*
 * joint_torque_jta.h
 *
 *  Created on: Oct 21, 2009
 *      Author: dc
 */

#ifndef JOINT_TORQUE_JTA_H_
#define JOINT_TORQUE_JTA_H_


#include "../detail/ca_macro.h"
#include "../units.h"
#include "./abstract/system.h"
#include "./abstract/joint_torque_adapter.h"


namespace barrett {
namespace systems {


class JointTorqueJTA : public JointTorqueAdapter {
// IO
public:		Input<units::JointTorques> controlInput;


public:
	JointTorqueJTA() :
		JointTorqueAdapter(), controlInput(this) {}
	explicit JointTorqueJTA(units::JointTorques initialOutputValue) :
		JointTorqueAdapter(initialOutputValue), controlInput(this) {}
	virtual ~JointTorqueJTA() {}

	virtual Input<units::JointTorques>* getControlInput();

protected:
	virtual void operate();

private:
	DISALLOW_COPY_AND_ASSIGN(JointTorqueJTA);
};


}
}


#endif /* JOINT_TORQUE_JTA_H_ */
