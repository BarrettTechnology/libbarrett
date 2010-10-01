/*
 * low_level_wam.h
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_LOW_LEVEL_WAM_H_
#define BARRETT_SYSTEMS_LOW_LEVEL_WAM_H_


#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/cdlbt/wambot_phys.h>

#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class LowLevelWam {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:		System::Input<jt_type>& input;
public:		System::Output<jp_type>& jpOutput;
public:		System::Output<jv_type>& jvOutput;


public:
	explicit LowLevelWam(const libconfig::Setting& setting);
	~LowLevelWam();

protected:
	class Sink : public System, public SingleInput<jt_type> {
	public:
		Sink(LowLevelWam* parent) :
			System(true),  // Update every execution cycle because this is a sink.
			SingleInput<jt_type>(this),
			parent(parent) {}
		virtual ~Sink() {}

	protected:
		virtual void operate();

		LowLevelWam* parent;

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
		Source(LowLevelWam* parent) :
			System(true),  // Update every execution cycle to prevent heartbeat
						   // faults. Depending on connections, this System
						   // might not be called for a period of time. In this
						   // situation, the pucks would stop reporting
						   // positions and the safety system would assume they
						   // had died.
			jpOutput(this, &jpOutputValue), jvOutput(this, &jvOutputValue),
			parent(parent) {}
		virtual ~Source() {}

	protected:
		virtual void operate();

		LowLevelWam* parent;

	private:
		DISALLOW_COPY_AND_ASSIGN(Source);
	};


	Sink sink;
	Source source;

public:
	struct bt_wambot_phys* wambot;  // TODO(dc): hack to quickly let other entities talk on the CAN bus (e.g. BH8-280).

private:
	DISALLOW_COPY_AND_ASSIGN(LowLevelWam);
};


}
}


// include template definitions
#include <barrett/systems/detail/low_level_wam-inl.h>


#endif /* BARRETT_SYSTEMS_LOW_LEVEL_WAM_H_ */
