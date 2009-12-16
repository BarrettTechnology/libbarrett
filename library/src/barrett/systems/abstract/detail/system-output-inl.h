/*
 * system-output-inl.h
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */


#include <algorithm>
#include <vector>
#include <list>


namespace barrett {
namespace systems {


// the compiler requires a definition for a dtor, even if it's "pure" virtual
inline System::AbstractOutput::~AbstractOutput() { }


template<typename T>
System::Output<T>::Output(System* parentSystem, Value** valueHandle) :
	value(parentSystem, *this), inputs(), delegators()
{
	(*valueHandle) = &value;
}

template<typename T>
System::Output<T>::~Output()
{
	disconnect(*this);
	value.undelegate();

	// value.undelegate() removes elements from the delegators list
	while (delegators.size()) {
		delegators.front()->value.undelegate();
	}
}


template<typename T>
typename System::Output<T>::Value*
System::Output<T>::getValueObject()
{
	Value* v = &value;
	while (v->delegate != NULL) {
		v = v->delegate;
	}

	return v;
}


}
}
