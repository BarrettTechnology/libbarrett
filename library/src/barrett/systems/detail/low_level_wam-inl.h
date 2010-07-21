/*
 * low_level_wam-inl.h
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */


#include <stdexcept>
#include <sstream>

#include <syslog.h>

#include <libconfig.h++>
#include <gsl/gsl_vector.h>

#include "../../cdlbt/wambot/wambot_phys.h"


namespace barrett {
namespace systems {


template<size_t DOF>
LowLevelWam<DOF>::LowLevelWam(const libconfig::Setting& setting) :
	input(sink.input),
	jpOutput(source.jpOutput), jvOutput(source.jvOutput),
	sink(this), source(this), wambot(NULL)
{
	// open the WAM
	bt_wambot_phys_create(&wambot, setting.getCSetting(), 0 /* do zeroangle compensation */);
	if (wambot == NULL) {
		// TODO(dc): make better error messages
		throw std::runtime_error(
				"(systems::LowLevelWam::LowLevelWam()): Couldn't make WAM. "
				"Check /var/log/syslog for more info.");
	}
	if (wambot->base.dof != DOF) {
		bt_wambot_phys_destroy(wambot);

		std::stringstream ss;
		ss << "(systems::LowLevelWam::LowLevelWam()): Configuration doesn't "
				"match. The code expects " << DOF << " DOF, configuration "
				"specifies " << wambot->base.dof << " DOF.";
		throw std::runtime_error(ss.str());
	}
}

template<size_t DOF>
LowLevelWam<DOF>::~LowLevelWam()
{
	bt_wambot_phys_destroy(wambot);
	wambot = NULL;
}

template<size_t DOF>
void LowLevelWam<DOF>::Sink::operate()
{
	this->input.getValue().copyTo(parent->wambot->base.jtorque);
	bt_wambot_setjtor(&parent->wambot->base);
}

template<size_t DOF>
void LowLevelWam<DOF>::Source::operate()
{
	bt_wambot_update(&parent->wambot->base);
	this->jpOutputValue->setValue(jp_type(parent->wambot->base.jposition));
	this->jvOutputValue->setValue(jv_type(parent->wambot->base.jvelocity));
}


}
}
