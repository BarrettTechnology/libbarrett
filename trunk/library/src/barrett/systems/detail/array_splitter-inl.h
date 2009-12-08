/*
 * array_splitter-inl.h
 *
 *  Created on: Nov 17, 2009
 *      Author: dc
 */


#include "../../detail/purge.h"
#include "../abstract/system.h"


namespace barrett {
namespace systems {


template <typename T>
ArraySplitter<T>::ArraySplitter() :
	input(this)
{
	initOutputs();
}

template <typename T>
inline ArraySplitter<T>::~ArraySplitter()
{
	purge(outputs);
}


template <typename T>
inline System::Output<double>& ArraySplitter<T>::getOutput(const size_t i)
{
	return *( outputs.at(i) );
}

template <typename T>
void ArraySplitter<T>::operate()
{
	for (size_t i = 0; i < T::SIZE; ++i) {
		outputValues[i]->setValue(input.getValue()[i]);
	}
}


template<typename T>
void ArraySplitter<T>::initOutputs()
{
	for (size_t i = 0; i < T::SIZE; ++i) {
		outputs[i] = new System::Output<double>(this, &outputValues[i]);
	}
}


}
}
