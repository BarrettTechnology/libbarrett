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
 * kinematics_base.h
 *
 *  Created on: Jan 15, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_KINEMATICS_BASE_H_
#define BARRETT_SYSTEMS_KINEMATICS_BASE_H_


#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math/kinematics.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class KinematicsInput {  // not a System in order to avoid diamond inheritance
// IO
public:	System::Input<math::Kinematics<DOF> > kinInput;


public:
	explicit KinematicsInput(System* parentSys) :
		kinInput(parentSys) {}

private:
	DISALLOW_COPY_AND_ASSIGN(KinematicsInput);
};


template<size_t DOF>
class KinematicsBase : public System {
// IO
public:		Input<typename units::JointPositions<DOF>::type> jpInput;
public:		Input<typename units::JointVelocities<DOF>::type> jvInput;
public:		Output<math::Kinematics<DOF> > kinOutput;
protected:	typename Output<math::Kinematics<DOF> >::Value* kinOutputValue;


public:
	explicit KinematicsBase(const libconfig::Setting& setting, const std::string& sysName = "KinematicsBase") :
		System(sysName),
		jpInput(this), jvInput(this),
		kinOutput(this, &kinOutputValue), kin(setting) {}
	virtual ~KinematicsBase() { mandatoryCleanUp(); }

protected:
	virtual void operate() {
		kin.eval(jpInput.getValue(), jvInput.getValue());
		kinOutputValue->setData(&kin);
	}

	math::Kinematics<DOF> kin;

private:
	DISALLOW_COPY_AND_ASSIGN(KinematicsBase);
};


}
}


#endif /* BARRETT_SYSTEMS_KINEMATICS_BASE_H_ */
