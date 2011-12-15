/*
 * ramp.cpp
 *
 *  Created on: May 18, 2011
 *      Author: dc
 */

#include <barrett/systems/ramp.h>


namespace barrett {
namespace systems {


Ramp::Ramp(ExecutionManager* em, double slope, const std::string& sysName) :
	System(sysName), SingleOutput<double>(this),
	T_s(0.0), gain(slope), finalGain(0.0), curGain(0.0), curvature(0.0), y(0.0)
{
	// Update every execution cycle so the ramp stays current even if the
	// data isn't used for a time.
	if (em != NULL) {
		em->startManaging(*this);
	}

	getSamplePeriodFromEM();
}
Ramp::~Ramp() {
	mandatoryCleanUp();
}


void Ramp::onExecutionManagerChanged() {
	System::onExecutionManagerChanged();  // First, call super
	getSamplePeriodFromEM();
}

void Ramp::operate() {
	if (isRunning()) {
		if (curvature != 0.0) {
			curGain += T_s * curvature;
			if ((curvature > 0.0  &&  curGain >= finalGain)  ||
				(curvature < 0.0  &&  curGain <= finalGain)) {
				curvature = 0.0;
				curGain = finalGain;
			}
		}

		y += T_s * curGain;
	}

	outputValue->setData(&y);
}

void Ramp::getSamplePeriodFromEM()
{
	if (this->hasExecutionManager()) {
		assert(this->getExecutionManager()->getPeriod() > 0.0);
		T_s = this->getExecutionManager()->getPeriod();
	} else {
		T_s = 0.0;
	}
}


}
}
