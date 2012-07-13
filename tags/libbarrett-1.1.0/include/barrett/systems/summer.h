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
 * summer.h
 *
 *  Created on: Sep 10, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_SUMMER_H_
#define BARRETT_SYSTEMS_SUMMER_H_

#include <bitset>
#include <string>
#include <stdexcept>

#include <boost/array.hpp>
#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


// FIXME: it might be nice to have a Summer with the number of inputs
//        determined at runtime
template<typename T, size_t numInputs = 2, bool RequiresAlignment = math::Traits<T>::RequiresAlignment>
class Summer : public System, public SingleOutput<T> {
// IO
// protected because of variable number of inputs
protected:	boost::array<Input<T>*, numInputs> inputs;


public:
	class Polarity {
	public:
		Polarity();  // default: all positive
		explicit Polarity(std::string polarityStr) throw(std::invalid_argument);
		explicit Polarity(const std::bitset<numInputs>& inputPolarity) :
			polarity(inputPolarity) {}
		virtual ~Polarity() {}

		// TODO(dc): operator[]=
		int operator[] (const size_t i) const;

	protected:
		std::bitset<numInputs> polarity;
	};

	Polarity polarity;

	explicit Summer(const Polarity& inputPolarity = Polarity(), bool undefinedIsZero = false, const std::string& sysName = "Summer");
	explicit Summer(const std::string& inputPolarity, bool undefinedIsZero = false, const std::string& sysName = "Summer");
	explicit Summer(const char* inputPolarity, bool undefinedIsZero = false, const std::string& sysName = "Summer");  // Without this, a string literal argument calls the Summer(bool) overload.
	explicit Summer(const std::bitset<numInputs>& inputPolarity, bool undefinedIsZero = false, const std::string& sysName = "Summer");
	explicit Summer(bool undefinedIsZero, const std::string& sysName = "Summer");
	virtual ~Summer();

	Input<T>& getInput(const size_t i);

protected:
	virtual bool inputsValid() {  return true;  };
	virtual void operate();
	virtual void invalidateOutputs() {}

	void initInputs();

	bool strict;
	T sum;

private:
	DISALLOW_COPY_AND_ASSIGN(Summer);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(RequiresAlignment)
};


}
}


// include template definitions
#include <barrett/systems/detail/summer-inl.h>
#include <barrett/systems/detail/summer-polarity-inl.h>


#endif /* BARRETT_SYSTEMS_SUMMER_H_ */
