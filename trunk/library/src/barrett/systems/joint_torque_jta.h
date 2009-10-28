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


template<size_t N>
class JointTorqueJTA : public JointTorqueAdapter<N> {
// IO
public:	System::Input<units::JointTorques<N> > controlInput;


public:
	JointTorqueJTA() :
		JointTorqueAdapter<N>(), controlInput(this) {}
	explicit JointTorqueJTA(units::JointTorques<N> initialOutputValue) :
		JointTorqueAdapter<N>(initialOutputValue), controlInput(this) {}
	virtual ~JointTorqueJTA() {}

	virtual System::Input<units::JointTorques<N> >* getControlInput();

protected:
	virtual void operate();

private:
	DISALLOW_COPY_AND_ASSIGN(JointTorqueJTA);
};


}
}


#endif /* JOINT_TORQUE_JTA_H_ */
