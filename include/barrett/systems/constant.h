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
 * constant.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_CONSTANT_H_
#define BARRETT_SYSTEMS_CONSTANT_H_


#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {

// TODO(dc): add a configuration file interface

template<typename T, bool RequiresAlignment = math::Traits<T>::RequiresAlignment>
class Constant : public System, public SingleOutput<T> {
public:
	explicit Constant(const T& value, const std::string& sysName = "Constant") :
		System(sysName), SingleOutput<T>(this), data(value)
	{
		this->outputValue->setData(&data);
	}
	virtual ~Constant() { mandatoryCleanUp(); }

protected:
	virtual void operate() {  /* do nothing */  }
	virtual void invalidateOutputs() { /* do nothing */ }

	T data;

private:
	DISALLOW_COPY_AND_ASSIGN(Constant);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(RequiresAlignment)
};


}
}


#endif /* BARRETT_SYSTEMS_CONSTANT_H_ */
