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
 * gravity_compensator.h
 *
 *  Created on: Feb 9, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_GRAVITY_COMPENSATOR_H_
#define BARRETT_SYSTEMS_GRAVITY_COMPENSATOR_H_


#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/cdlbt/calgrav.h>

#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class GravityCompensator : public System,
						   public KinematicsInput<DOF>,
						   public SingleOutput<typename units::JointTorques<DOF>::type> {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	explicit GravityCompensator(const libconfig::Setting& setting,
			const std::string& sysName = "GravityCompensator") :
		System(sysName), KinematicsInput<DOF>(this), SingleOutput<jt_type>(this), impl(NULL), data()
	{
		bt_calgrav_create(&impl, setting.getCSetting(), DOF);
	}

	virtual ~GravityCompensator() {
		mandatoryCleanUp();

		bt_calgrav_destroy(impl);
		impl = NULL;
	}

protected:
	virtual void operate() {
		bt_calgrav_eval(impl, this->kinInput.getValue().impl, data.asGslType());
		this->outputValue->setData(&data);
	}

	struct bt_calgrav* impl;
	jt_type data;

private:
	DISALLOW_COPY_AND_ASSIGN(GravityCompensator);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_GRAVITY_COMPENSATOR_H_ */
