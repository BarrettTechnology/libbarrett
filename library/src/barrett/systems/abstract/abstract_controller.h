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

// removed because it causes unfortunate include dependencies.
// forward declared instead
//#include "../supervisory_controller.h"


namespace barrett {
namespace systems {


class SupervisoryController;


class AbstractController : public System {
public:
	AbstractController() {}
	virtual ~AbstractController() {}

	virtual AbstractInput* getReferenceInput() = 0;
	virtual AbstractInput* getFeedbackInput() = 0;
	virtual AbstractOutput* getControlOutput() = 0;

	// double-dispatch to find and connect an appropriate JointTorqueAdapter
	virtual void selectAndConnectAdapter(const SupervisoryController& sc)
	throw(std::invalid_argument) = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(AbstractController);
};


}
}


#endif /* ABSTRACT_CONTROLLER_H_ */
