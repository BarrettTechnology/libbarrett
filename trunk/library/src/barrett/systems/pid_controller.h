/*
 * pid_controller.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_


#include "./abstract/controller.h"


namespace barrett {
namespace systems {


template<typename InputType,
		 typename OutputType = typename InputType::actuator_type>
class PIDController : public Controller<InputType, OutputType> {
protected:
	virtual void operate();
};


}
}


// include template definitions
#include "./detail/pid_controller-inl.h"


#endif /* PID_CONTROLLER_H_ */
