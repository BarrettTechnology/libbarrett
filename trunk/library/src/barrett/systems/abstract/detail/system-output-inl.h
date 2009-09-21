/*
 * system::output.tcc
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

#include <list>


namespace Systems {


template<typename T>
inline void System::Output<T>::addInput(const System::Input<T>& input)
{
	// const_cast is ok because these methods don't modify the input
	System::Input<T>* inputPtr = const_cast<System::Input<T>* >(&input);

	inputs.remove(inputPtr);  // prevent duplicates
	inputs.push_back(inputPtr);
}

template<typename T>
inline void System::Output<T>::removeInput(const Input<T>& input)
{
	// const_cast is ok because these methods don't modify the input
	inputs.remove(const_cast<System::Input<T>* >(&input));
}

template<typename T>
void System::Output<T>::notifyInputs() const
{
	if (inputs.empty()) {
		return;
	}

	typename std::list<Input<T>* >::const_iterator inputItr = inputs.begin();
	while (inputItr != inputs.end()) {
		(*inputItr)->onValueChanged();
		++inputItr;
	}
}

template<typename T>
System::Output<T>::Output(Value** valueHandle) :
	value(*this), inputs()
{
	(*valueHandle) = &value;
}

template<typename T>
System::Output<T>::Output(const T& initialValue, Value** valueHandle) :
	value(*this, initialValue), inputs()
{
	(*valueHandle) = &value;
}


}
