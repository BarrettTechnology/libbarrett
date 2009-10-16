/*
 * abstract_controller.h
 *
 *  Created on: Oct 6, 2009
 *      Author: dc
 */

#ifndef ABSTRACT_CONTROLLER_H_
#define ABSTRACT_CONTROLLER_H_


#include <list>
#include "./system.h"
#include "./joint_torque_adapter.h"


namespace Systems {


class AbstractController : public System {
public:
	AbstractController() {}
	virtual ~AbstractController() {}

	virtual AbstractInput* getReferenceInput() = 0;
	virtual AbstractInput* getFeedbackInput() = 0;
	virtual AbstractOutput* getControlOutput() = 0;

	virtual void selectAdapter(
			const std::list<JointTorqueAdapter*>& adapters) const
	throw(std::invalid_argument) = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(AbstractController);
};


}


#endif /* ABSTRACT_CONTROLLER_H_ */
