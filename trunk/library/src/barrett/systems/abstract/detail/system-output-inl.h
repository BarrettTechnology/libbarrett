/*
 * system::output.tcc
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

#include <list>


namespace barrett {
namespace systems {


// the compiler requires a definition for a dtor, even if it's pure virtual
inline System::AbstractOutput::~AbstractOutput() {}


template<typename T>
System::Output<T>::Output(Value** valueHandle) :
	value(*this), inputs(), delegators()
{
	(*valueHandle) = &value;
}

template<typename T>
System::Output<T>::Output(const T& initialValue, Value** valueHandle) :
	value(*this, initialValue), inputs(), delegators()
{
	(*valueHandle) = &value;
}

template<typename T>
System::Output<T>::~Output()
{
	// value.undelegate() removes elements from the delegators list
	while (delegators.size()) {
		(*delegators.begin())->value.undelegate();
	}

	value.undelegate();
}

// FIXME(dc): are these const casts actually ok? i don't remember writing
// those comments :(  can we use lists of const pointers instead?
template<typename T>
inline void System::Output<T>::addInput(const Input<T>& input)
{
	// const_cast is ok because these methods don't modify the input
	System::Input<T>* inputPtr = const_cast<Input<T>* >(&input);

	// FIXME(dc): i don't remember why i thought the duplicates thing was
	// necessary
	inputs.remove(inputPtr);  // prevent duplicates
	inputs.push_back(inputPtr);
}

template<typename T>
inline void System::Output<T>::removeInput(const Input<T>& input)
{
	// const_cast is ok because these methods don't modify the input
	inputs.remove(const_cast<Input<T>* >(&input));
}

template<typename T>
inline void System::Output<T>::addDelegator(const Output<T>& output) const
{
	// const_cast is ok because these methods don't modify the output
	delegators.push_back(const_cast<Output<T>* >(&output));
}

template<typename T>
inline void System::Output<T>::removeDelegator(const Output<T>& output) const
{
	// const_cast is ok because these methods don't modify the output
	delegators.remove(const_cast<Output<T>* >(&output));
}

template<typename T>
void System::Output<T>::notifyListeners() const
{
	// notify inputs
	typename std::list<Input<T>* >::const_iterator inputItr = inputs.begin();
	while (inputItr != inputs.end()) {
		(*inputItr)->onValueChanged();
		++inputItr;
	}

	// notify delegators
	typename std::list<Output<T>* >::const_iterator outputItr = delegators.begin();
	while (outputItr != delegators.end()) {
		(*outputItr)->notifyListeners();
		++outputItr;
	}
}

template<typename T>
const typename System::Output<T>::Value*
System::Output<T>::getValueObject() const
{
	const Value* v = &value;
	while (v->delegate != NULL) {
		v = v->delegate;
	}

	return v;
}


}
}
