/*
 * gain.h
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 */

#ifndef GAIN_H_
#define GAIN_H_


#include "abstract/single_io.h"
#include "../ca_macro.h"


namespace Systems {


template<typename InputType,
		 typename GainType = InputType,
		 typename OutputType = InputType>
class Gain : public SingleIO<InputType, OutputType> {
protected:
	GainType gain;

	virtual void operate() {
		this->outputValue->setValue(this->input.getValue() * gain);
	}

public:
	explicit Gain(GainType gain) :
		gain(gain) {}
	virtual ~Gain() {}

private:
	DISALLOW_COPY_AND_ASSIGN(Gain);
};


}


#endif /* GAIN_H_ */
