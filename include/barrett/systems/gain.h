/*
 * gain.h
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_GAIN_H_
#define BARRETT_SYSTEMS_GAIN_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {

// TODO(dc): add a configuration file interface

template<typename InputType,
		 typename GainType = InputType,
		 typename OutputType = InputType>
class Gain : public SingleIO<InputType, OutputType> {
public:
	explicit Gain(GainType gain) :
		gain(gain) {}
	virtual ~Gain() {}

protected:
	GainType gain;

	virtual void operate() {
		this->outputValue->setValue(this->input.getValue() * gain);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Gain);
};


}
}


#endif /* BARRETT_SYSTEMS_GAIN_H_ */
