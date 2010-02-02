/*
 * low_level_wam.h
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */

#ifndef LOW_LEVEL_WAM_H_
#define LOW_LEVEL_WAM_H_


#include <libconfig.h++>

//#include <barrett/wam/wam.h>
//#include <barrett/wam/wam_local.h>
#include <barrett/wambot/wambot_phys.h>

#include "../detail/ca_macro.h"
#include "../units.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


template<size_t DOF>
class LowLevelWam {
public:
	typedef units::JointTorques<DOF> jt_type;
	typedef units::JointPositions<DOF> jp_type;
	typedef units::JointVelocities<DOF> jv_type;


public:		System::Input<jt_type>& input;
public:		System::Output<jp_type>& jpOutput;
public:		System::Output<jv_type>& jvOutput;


public:
	// TODO(dc): Change parameter to const libconfig::Setting& once we get past the libconfig::Setting::getCSetting() hack.
	explicit LowLevelWam(libconfig::Setting& setting);
	~LowLevelWam();

protected:
	class Sink : public System, public SingleInput<jt_type> {
	public:
		Sink(LowLevelWam* parent) :
			System(true), SingleInput<jt_type>(this),
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
			System(),
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

private:
//	struct bt_wam* wam;
//	struct bt_wam_local* wamLocal;
	struct bt_wambot_phys* wambot;

	DISALLOW_COPY_AND_ASSIGN(LowLevelWam);
};


}
}


// include template definitions
#include "./detail/low_level_wam-inl.h"


#endif /* LOW_LEVEL_WAM_H_ */
