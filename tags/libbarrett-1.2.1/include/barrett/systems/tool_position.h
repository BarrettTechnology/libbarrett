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
 * tool_position.h
 *
 *  Created on: Jan 15, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_TOOL_POSITION_H_
#define BARRETT_SYSTEMS_TOOL_POSITION_H_


#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class ToolPosition : public System, public KinematicsInput<DOF>,
					 public SingleOutput<units::CartesianPosition::type> {
public:
	ToolPosition(const std::string& sysName = "ToolPosition") :
		System(sysName), KinematicsInput<DOF>(this),
		SingleOutput<units::CartesianPosition::type>(this), data() {}
	virtual ~ToolPosition() { mandatoryCleanUp(); }

protected:
	virtual void operate() {
		data.copyFrom(this->kinInput.getValue().impl->tool->origin_pos);
		this->outputValue->setData(&data);
	}

	units::CartesianPosition::type data;

private:
	DISALLOW_COPY_AND_ASSIGN(ToolPosition);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_TOOL_POSITION_H_ */
