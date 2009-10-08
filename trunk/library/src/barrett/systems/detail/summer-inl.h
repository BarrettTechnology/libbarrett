/*
 * summer.tcc
 *
 *  Created on: Sep 11, 2009
 *      Author: dc
 */

#include <boost/array.hpp>
// #include <vector>
#include <string>
#include <bitset>
#include "../../detail/purge.h"
#include "../abstract/system.h"


namespace Systems {

template<typename T, size_t numInputs>
void Summer<T, numInputs>::initInputs()
{
	for (size_t i = 0; i < numInputs; ++i) {
		inputs[i] = new Input<T>(this);
	}
}

template<typename T, size_t numInputs>
Summer<T, numInputs>::Summer(const Polarity& inputPolarity) :
	output(&outputValue),
	polarity(inputPolarity)
{
	initInputs();
}

template<typename T, size_t numInputs>
Summer<T, numInputs>::Summer(const std::string& inputPolarity) :
	output(&outputValue),
	polarity(inputPolarity)
{
	initInputs();
}

template<typename T, size_t numInputs>
Summer<T, numInputs>::Summer(const std::bitset<numInputs>& inputPolarity) :
	output(&outputValue),
	polarity(inputPolarity)
{
	initInputs();
}

template<typename T, size_t numInputs>
inline Summer<T, numInputs>::~Summer()
{
	purge(inputs);
/*	for (size_t i = 0; i < numInputs; ++i) {
		delete inputs[i];
		inputs[i] = NULL;
	}
*/
}

template<typename T, size_t numInputs>
void Summer<T, numInputs>::operate()
{
	T sum;
	for (size_t i = 0; i < numInputs; ++i) {
		sum += polarity[i] * inputs[i]->getValue();
	}

	outputValue->setValue(sum);
}

template<typename T, size_t numInputs>
inline System::Input<T>& Summer<T, numInputs>::getInput(const size_t i) {
	return *( inputs.at(i) );
}


}
