/*
 * abstract_controller.h
 *
 *  Created on: Oct 6, 2009
 *      Author: dc
 */

#ifndef ABSTRACT_CONTROLLER_H_
#define ABSTRACT_CONTROLLER_H_


#include "./system.h"


namespace Systems {


class AbstractController : public System {
public:
	AbstractController() {}
	virtual ~AbstractController() {}

	virtual System::AbstractInput* getReferenceInput() = 0;
	virtual System::AbstractInput* getFeedbackInput() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(AbstractController);
};


}


#endif /* ABSTRACT_CONTROLLER_H_ */
