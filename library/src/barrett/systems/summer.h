/*
 * summer.h
 *
 *  Created on: Sep 10, 2009
 *      Author: dc
 */

#ifndef SUMMER_H_
#define SUMMER_H_

#include <boost/array.hpp>
// #include <vector>
#include <bitset>
#include <string>
#include <stdexcept>
#include "./abstract/system.h"
#include "../ca_macro.h"


namespace Systems {


template<typename T, size_t numInputs = 2>
class Summer : public System {
// IO
// protected because of variable number of inputs
// FIXME: make this a boost::array? potential initialization problems...
//   store Input<T>* instead?
protected:	boost::array<Input<T>*, numInputs> inputs;
public:		Output<T> output;
protected:	typename Output<T>::Value* outputValue;


public:
	class Polarity {  // FIXME: does this deserve a nested class?
	protected:
		std::bitset<numInputs> polarity;

	public:
		Polarity();  // default: all positive
		explicit Polarity(std::string polarityStr) throw(std::invalid_argument);
		explicit Polarity(const std::bitset<numInputs>& inputPolarity) :
			polarity(inputPolarity) {}
		virtual ~Polarity() {}

		// TODO(dc): operator[]=
		virtual const int operator[] (const size_t i) const;
	};

protected:
	virtual void operate();
	void initInputs();

public:
	Polarity polarity;

	explicit Summer(const Polarity& inputPolarity = Polarity());
	explicit Summer(const std::string& inputPolarity);
	explicit Summer(const std::bitset<numInputs>& inputPolarity);
	virtual ~Summer();

	Input<T>& getInput(const size_t i);

private:
	DISALLOW_COPY_AND_ASSIGN(Summer);
};


}


// include template definitions
#include "detail/summer-inl.h"
#include "detail/summer-polarity-inl.h"


#endif /* SUMMER_H_ */
