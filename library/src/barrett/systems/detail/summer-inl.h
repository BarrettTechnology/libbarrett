/*
 * summer.tcc
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

#include <boost/array.hpp>
//#include <vector>
#include <string>
#include <bitset>

#include "../../detail/stl_utils.h"
#include "../abstract/system.h"


namespace barrett {
namespace systems {


template<typename T, size_t numInputs>
Summer<T, numInputs>::Summer(const Polarity& inputPolarity) :
	SingleOutput<T>(this),
	polarity(inputPolarity)
{
	initInputs();
}

template<typename T, size_t numInputs>
Summer<T, numInputs>::Summer(const std::string& inputPolarity) :
	SingleOutput<T>(this),
	polarity(inputPolarity)
{
	initInputs();
}

template<typename T, size_t numInputs>
Summer<T, numInputs>::Summer(const std::bitset<numInputs>& inputPolarity) :
	SingleOutput<T>(this),
	polarity(inputPolarity)
{
	initInputs();
}

template<typename T, size_t numInputs>
inline Summer<T, numInputs>::~Summer()
{
	purge(inputs);
}

template<typename T, size_t numInputs>
inline System::Input<T>& Summer<T, numInputs>::getInput(const size_t i) {
	return *( inputs.at(i) );
}

template<typename T, size_t numInputs>
void Summer<T, numInputs>::operate()
{
	T sum;
	for (size_t i = 0; i < numInputs; ++i) {
		sum = sum + polarity[i] * inputs[i]->getValue();
	}

	this->outputValue->setValue(sum);
}

template<typename T, size_t numInputs>
void Summer<T, numInputs>::initInputs()
{
	for (size_t i = 0; i < numInputs; ++i) {
		inputs[i] = new Input<T>(this);
	}
}


}
}
