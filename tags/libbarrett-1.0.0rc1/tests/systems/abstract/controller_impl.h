/*
 * controller_impl.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */

#ifndef CONTROLLER_IMPL_H_
#define CONTROLLER_IMPL_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/controller.h>


template<typename InputType, typename OutputType = InputType>
class ControllerImpl :
		public barrett::systems::Controller<InputType, OutputType> {
public:
	ControllerImpl(const std::string& sysName = "ControllerImpl")
		: barrett::systems::Controller<InputType, OutputType>(sysName) {}
	virtual ~ControllerImpl() { this->mandatoryCleanUp(); }

protected:
	virtual void operate() {}

private:
	DISALLOW_COPY_AND_ASSIGN(ControllerImpl);
};


#endif /* CONTROLLER_IMPL_H_ */
