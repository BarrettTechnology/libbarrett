/*
 * supervisory_controller.cpp
 *
 *  Created on: Oct 5, 2009
 *      Author: dc
 */


#include <list>

#include "../../units.h"
#include "../abstract/abstract_controller.h"
#include "../pid_controller.h"
#include "../joint_torque_jta.h"
#include "../supervisory_controller.h"


namespace barrett {
namespace systems {


SupervisoryController::SupervisoryController(
//		const std::list<AbstractController*>& additionalControllers,
		bool includeStandardControllers,
		bool includeStandardAdapters) :
//			controllers(additionalControllers)
			input(this), output(&outputValue), controllers(), adapters()
{
	if (includeStandardControllers) {
		controllers.push_back(new PIDController<units::JointAngles>);
	}

	if (includeStandardAdapters) {
		adapters.push_back(new JointTorqueJTA);
	}
}


}
}
