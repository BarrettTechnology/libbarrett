/*
 * summer::polarity.tcc
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 */

#include <string>
#include <stdexcept>


namespace Systems {


template<typename T, size_t numInputs>
Summer<T, numInputs>::Polarity::Polarity() :  // default: all positive
	polarity()
{
	polarity.set();
}

template<typename T, size_t numInputs>
Summer<T, numInputs>::Polarity::Polarity(std::string polarityStr)
throw(std::invalid_argument) :
	polarity()
{
	if (polarityStr.size() != numInputs) {
		throw std::invalid_argument("(Systems::Summer::Polarity::Polarity): "
		                            "polarityStr must be of the same length "
		                            "as the number of inputs to the Summer.");
	}

	for (size_t i = 0; i < numInputs; ++i) {
		switch (polarityStr[i]) {
		case '+':
			polarity.set(i);
			break;
		case '-':
			polarity.reset(i);
			break;
		default:
			throw std::invalid_argument(
					"(Systems::Summer::Polarity::Polarity): polarityStr must "
					"contain only '+' and '-' characters.");
			break;
		}
	}
}

template<typename T, size_t numInputs>
const int Summer<T, numInputs>::Polarity::operator[] (const size_t i) const
{
	return polarity[i] ? 1 : -1;
}


}
