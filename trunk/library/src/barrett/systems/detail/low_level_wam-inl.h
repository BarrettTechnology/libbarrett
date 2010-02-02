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

//#include <barrett/wam/wam.h>
//#include <barrett/wam/wam_local.h>
#include <barrett/wambot/wambot_phys.h>


namespace barrett {
namespace systems {


template<size_t DOF>
LowLevelWam<DOF>::LowLevelWam(const libconfig::Setting& setting) :
	input(sink.input),
	jpOutput(source.jpOutput), jvOutput(source.jvOutput),
	sink(this), source(this), wambot(NULL) //, wam(NULL), wamLocal(NULL)
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
		std::stringstream ss;
		ss << "(systems::LowLevelWam::LowLevelWam()): Configuration doesn't "
				"match. The code expects " << DOF << " DOF, configuration "
				"specifies " << wambot->base.dof << " DOF.";
		throw std::runtime_error(ss.str());
	}

	/*
	// open the WAM
	bt_wam_create(&wam, "wamg");
	if (wam == NULL) {
		// TODO(dc): make better error messages
		throw std::runtime_error(
				"(systems::LowLevelWam::LowLevelWam()): Couldn't make WAM. "
				"Check /var/log/syslog for more info.");
	}
	if (bt_wam_dof(wam) != DOF) {
		std::stringstream ss;
		ss << "(systems::LowLevelWam::LowLevelWam()): Configuration doesn't "
				"match. The code expects " << DOF << " DOF, configuration "
				"specifies " << bt_wam_dof(wam) << " DOF.";
		throw std::runtime_error(ss.str());
	}

	wamLocal = bt_wam_get_local(wam);
	*/
}

template<size_t DOF>
LowLevelWam<DOF>::~LowLevelWam()
{
	bt_wambot_phys_destroy(wambot);
	wambot = NULL;

//	bt_wam_destroy(wam);
//	wam = NULL;
//	wamLocal = NULL;
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
//	gsl_vector_set_zero(parent->wambot->base.jtorque);
	this->jpOutputValue->setValue(jp_type(parent->wambot->base.jposition));
	this->jvOutputValue->setValue(jv_type(parent->wambot->base.jvelocity));
}


}
}
