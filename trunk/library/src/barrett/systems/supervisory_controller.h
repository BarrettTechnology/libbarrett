/*
 * supervisory_controller.h
 *
 *  Created on: Oct 29, 2009
 *      Author: dc
 */

#ifndef SUPERVISORY_CONTROLLER_H_
#define SUPERVISORY_CONTROLLER_H_


#include <list>
#include <utility>

#include "../detail/ca_macro.h"
#include "./abstract/system.h"
#include "./abstract/supervisory_controllable.h"


namespace barrett {
namespace systems {


template<typename OutputType>
class SupervisoryController : public System {
//IO
// input protected because it is for internal connections only
protected:	System::Input<OutputType> shaddowInput;
public:		System::Output<OutputType> output;
protected:	typename System::Output<OutputType>::Value* outputValue;

public:
	SupervisoryController() :
		shaddowInput(this), output(&outputValue), controllables() {}
	explicit SupervisoryController(const OutputType& initialOutputValue) :
		shaddowInput(this), output(initialOutputValue, &outputValue),
		controllables() {}
	virtual ~SupervisoryController();

	void registerControllable(
			SupervisoryControllable<OutputType>* controllable);

	template<typename T>
	void trackReferenceSignal(System::Output<T>& referenceOutput)  //NOLINT: non-const reference for syntax
	throw(std::invalid_argument);

	template<typename T>
	bool trackReferenceSignal(System::Output<T>& referenceOutput,  //NOLINT: non-const reference for syntax
			SupervisoryControllable<OutputType>* controllable);
//	throw(std::invalid_argument);

protected:
	virtual void operate();

	std::list<SupervisoryControllable<OutputType>*> controllables;

private:
	DISALLOW_COPY_AND_ASSIGN(SupervisoryController);
};


}
}


// include template definitions
#include "./detail/supervisory_controller-inl.h"


#endif /* SUPERVISORY_CONTROLLER_H_ */
