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
 * ramp.h
 *
 *  Created on: Jan 5, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_RAMP_H_
#define BARRETT_SYSTEMS_RAMP_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


// TODO(dc): test this!
// TODO(dc): add a configuration file interface


class Ramp : public System, public SingleOutput<double> {
public:
	explicit Ramp(ExecutionManager* em, double slope = 1.0,
			const std::string& sysName = "Ramp") :
		System(sysName), SingleOutput<double>(this),
		T_s(0.0), gain(slope), y(0.0), running(false)
	{
		// Update every execution cycle so the ramp stays current even if the
		// data isn't used for a time.
		if (em != NULL) {
			em->startManaging(*this);
		}

		getSamplePeriodFromEM();
	}
	virtual ~Ramp() { mandatoryCleanUp(); }

	void setSlope(double slope) {  gain = slope;  }

	void start() {  running = true;  }
	void stop() {  running = false;  }
	void reset() {  setOutput(0.0);  }
	void setOutput(double newOutput) {
		// y is written and read in operate(), so it needs to be locked.
		BARRETT_SCOPED_LOCK(getEmMutex());
		y = newOutput;
	}

protected:
	virtual void onExecutionManagerChanged() {
		System::onExecutionManagerChanged();  // First, call super
		getSamplePeriodFromEM();
	}

	virtual void operate() {
		if (running) {
			y += T_s * gain;
		}

		outputValue->setData(&y);
	}


	void getSamplePeriodFromEM()
	{
		if (this->hasExecutionManager()) {
			assert(this->getExecutionManager()->getPeriod() > 0.0);
			T_s = this->getExecutionManager()->getPeriod();
		} else {
			T_s = 0.0;
		}
	}


	double T_s, gain, y;
	bool running;

private:
	DISALLOW_COPY_AND_ASSIGN(Ramp);
};


}
}


#endif /* BARRETT_SYSTEMS_RAMP_H_ */
