/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * gain.h
 *
 *  Created on: Sep 12, 2009
 *      Author: dc
 *              CJ Valle
 */

#ifndef BARRETT_SYSTEMS_GAIN_H_
#define BARRETT_SYSTEMS_GAIN_H_


#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {

// TODO(dc): add a configuration file interface

template<typename InputType,
		 typename GainType = InputType,
		 typename OutputType = InputType,
		 bool RequiresAlignment = (math::Traits<GainType>::RequiresAlignment || math::Traits<OutputType>::RequiresAlignment)>
class Gain : public SingleIO<InputType, OutputType> {
public:
	explicit Gain(GainType gain, const std::string& sysName = "Gain") :
		SingleIO<InputType, OutputType>(sysName), gain(gain) {}
	virtual ~Gain() { this->mandatoryCleanUp(); }

	void setGain(const GainType& g) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		gain = g;
	}

protected:
	GainType gain;
	OutputType data;

	virtual void operate() {
		data = gain * this->input.getValue();
		this->outputValue->setData(&data);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Gain);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(RequiresAlignment)
};


}
}


#endif /* BARRETT_SYSTEMS_GAIN_H_ */
