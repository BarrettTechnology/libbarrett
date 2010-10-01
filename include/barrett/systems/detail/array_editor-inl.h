/*
 * array_editor-inl.h
 *
 *  Created on: Nov 12, 2009
 *      Author: dc
 */


#include <barrett/detail/stl_utils.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


template <typename T>
ArrayEditor<T>::ArrayEditor() :
	SingleIO<T, T>()
{
	initInputs();
}

template <typename T>
ArrayEditor<T>::ArrayEditor(const T& initialOutputValue) :
	SingleIO<T, T>(initialOutputValue)
{
	initInputs();
}

template <typename T>
inline ArrayEditor<T>::~ArrayEditor()
{
	purge(elementInputs);
}


template <typename T>
inline System::Input<double>& ArrayEditor<T>::getElementInput(const size_t i)
{
	return *( elementInputs.at(i) );
}

template <typename T>
void ArrayEditor<T>::operate()
{
	T tmp = this->input.getValue();
	for (size_t i = 0; i < T::SIZE; ++i) {
		if (elementInputs[i]->valueDefined()) {
			tmp[i] = elementInputs[i]->getValue();
		}
	}

	this->outputValue->setValue(tmp);
}

template <typename T>
bool ArrayEditor<T>::inputsValid()
{
	return this->input.valueDefined();
}


template<typename T>
void ArrayEditor<T>::initInputs()
{
	for (size_t i = 0; i < T::SIZE; ++i) {
		elementInputs[i] = new System::Input<double>(this);
	}
}


}
}
