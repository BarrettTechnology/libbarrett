/*
 * supervisory_controllable.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef SUPERVISORY_CONTROLLABLE_H_
#define SUPERVISORY_CONTROLLABLE_H_


#include "../../detail/ca_macro.h"
#include "./system.h"


namespace barrett {
namespace systems {


// objects implementing this interface can be manipulated by a
// SupervisoryController
template<typename OutputType>
class SupervisoryControllable {
public:
	SupervisoryControllable() {}
	virtual ~SupervisoryControllable() {}

	virtual System::AbstractInput* getSupervisoryControllableInput() = 0;
	virtual System::Output<OutputType>& getSupervisoryControllableOutput() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(SupervisoryControllable);
};


}
}


#endif /* SUPERVISORY_CONTROLLABLE_H_ */
