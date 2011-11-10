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
 * inverse_dynamics.h
 *
 *  Created on: Nov 9, 2011
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_INVERSE_DYNAMICS_H_
#define BARRETT_SYSTEMS_INVERSE_DYNAMICS_H_


#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>

#include <barrett/math/dynamics.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class InverseDynamics :
	public SingleIO<typename units::JointAccelerations<DOF>::type, typename units::JointTorques<DOF>::type>,
	public KinematicsInput<DOF>,
	public math::Dynamics<DOF>
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO
public:		System::Input<jv_type> jvInput;

public:
	explicit InverseDynamics(const libconfig::Setting& setting, const std::string& sysName = "InverseDynamics")
		: SingleIO<ja_type, jt_type>(sysName), KinematicsInput<DOF>(this), math::Dynamics<DOF>(setting), jvInput(this) {}
	virtual ~InverseDynamics() { this->mandatoryCleanUp(); }

protected:
	virtual void operate() {
		evalInverse(this->kinInput.getValue(), jvInput.getValue(), this->input.getValue());
		this->outputValue->setData(&this->jt);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(InverseDynamics);
};


}
}


#endif /* BARRETT_SYSTEMS_INVERSE_DYNAMICS_H_ */
