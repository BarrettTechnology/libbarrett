/*
 * supervisory_controller.h
 *
 *  Created on: Oct 4, 2009
 *      Author: dc
 */

#ifndef SUPERVISORY_CONTROLLER_H_
#define SUPERVISORY_CONTROLLER_H_


#include <list>
#include "abstract/system.h"
#include "../detail/ca_macro.h"
#include "./abstract/abstract_controller.h"


namespace Systems {


class SupervisoryController /*: System*/ {
public:
	SupervisoryController(
//			const std::list<AbstractController*>& additionalControllers =
//					std::list<AbstractController*>(),
			bool includeStandardControllers = true);
	~SupervisoryController();

	template<typename T>
	void trackReferenceSignal(System::Output<T>& referenceSignal)  //NOLINT: non-const reference for syntax
	throw(std::invalid_argument);

protected:
	// the SupervisoryController owns the objects pointed to by this list
	std::list<AbstractController*> controllers;

private:
	DISALLOW_COPY_AND_ASSIGN(SupervisoryController);
};


}


// include inline definitions
#include "detail/supervisory_controller-inl.h"


#endif /* SUPERVISORY_CONTROLLER_H_ */
