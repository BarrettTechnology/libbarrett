/*
 * supervisory_controller.cpp
 *
 *  Created on: Oct 5, 2009
 *      Author: dc
 */


#include <list>

#include "../abstract/abstract_controller.h"
#include "../supervisory_controller.h"


namespace barrett {
namespace systems {


SupervisoryController::SupervisoryController(
//		const std::list<AbstractController*>& additionalControllers,
		bool includeStandardControllers) :
//			controllers(additionalControllers)
			controllers()
{
	if (includeStandardControllers) {
		// controllers.push_back
	}
}


}
}
