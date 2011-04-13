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
 * low_level_wam_wrapper.h
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_LOW_LEVEL_WAM_WRAPPER_H_
#define BARRETT_SYSTEMS_LOW_LEVEL_WAM_WRAPPER_H_


#include <vector>

#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/low_level_wam.h>
#include <barrett/products/safety_module.h>

#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class LowLevelWamWrapper {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:		System::Input<jt_type>& input;
public:		System::Output<jp_type>& jpOutput;
public:		System::Output<jv_type>& jvOutput;


public:
	// genericPucks must be ordered by joint and must break into torque groups as arranged
	LowLevelWamWrapper(ExecutionManager* em, const std::vector<Puck*>& genericPucks,
			SafetyModule* safetyModule, const libconfig::Setting& setting,
			std::vector<int> torqueGroupIds = std::vector<int>(),
			const std::string& sysName = "LowLevelWamWrapper");
	~LowLevelWamWrapper() {}

	LowLevelWam<DOF>& getLowLevelWam() { return llw; }
	const LowLevelWam<DOF>& getLowLevelWam() const { return llw; }

	thread::Mutex& getEmMutex() const { return sink.getEmMutex(); }

protected:
	class Sink : public System, public SingleInput<jt_type> {
	public:
		Sink(LowLevelWamWrapper* parent, ExecutionManager* em,
				const std::string& sysName = "LowLevelWamWrapper::Sink") :
			System(sysName),
			SingleInput<jt_type>(this),
			parent(parent)
		{
			// Update every execution cycle because this is a sink.
			if (em != NULL) {
				em->startManaging(*this);
			}
		}
		virtual ~Sink() { mandatoryCleanUp(); }

	protected:
		virtual void operate();

		LowLevelWamWrapper* parent;

	private:
		DISALLOW_COPY_AND_ASSIGN(Sink);
	};


	class Source : public System {
	// IO
	public:		Output<jp_type> jpOutput;
	protected:	typename Output<jp_type>::Value* jpOutputValue;
	public:		Output<jv_type> jvOutput;
	protected:	typename Output<jv_type>::Value* jvOutputValue;


	public:
		Source(LowLevelWamWrapper* parent, ExecutionManager* em,
				const std::string& sysName = "LowLevelWamWrapper::Sink") :
			System(sysName),
			jpOutput(this, &jpOutputValue), jvOutput(this, &jvOutputValue),
			parent(parent)
		{
			// Update every execution cycle to prevent heartbeat
			// faults. Depending on connections, this System
			// might not be called for a period of time. In this
			// situation, the pucks would stop reporting
			// positions and the safety system would assume they
			// had died.
			if (em != NULL) {
				em->startManaging(*this);
			}
		}
		virtual ~Source() { mandatoryCleanUp(); }

	protected:
		virtual void operate();

		LowLevelWamWrapper* parent;

	private:
		DISALLOW_COPY_AND_ASSIGN(Source);
	};


	LowLevelWam<DOF> llw;

	Sink sink;
	Source source;

private:
	DISALLOW_COPY_AND_ASSIGN(LowLevelWamWrapper);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


// include template definitions
#include <barrett/systems/detail/low_level_wam_wrapper-inl.h>


#endif /* BARRETT_SYSTEMS_LOW_LEVEL_WAM_WRAPPER_H_ */
