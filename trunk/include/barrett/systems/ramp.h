/*
 * ramp.h
 *
 *  Created on: Jan 5, 2010
 *      Author: dc
 */

#ifndef RAMP_H_
#define RAMP_H_


#include "../detail/ca_macro.h"
#include "../thread/abstract/mutex.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"


namespace barrett {
namespace systems {


// TODO(dc): test this!
// TODO(dc): add a configuration file interface


class Ramp : public System, public SingleOutput<double> {
public:
	explicit Ramp(double slope = 1.0, bool updateEveryExecutionCycle = true) :
		System(updateEveryExecutionCycle), SingleOutput<double>(this),
		T_s(0.0), gain(slope), y(0.0), running(false)
	{
		outputValue->setValue(y);
	}

	void setSamplePeriod(double timeStep) {  T_s = timeStep;  }
	void setSlope(double slope) {  gain = slope;  }

	void start() {  running = true;  }
	void stop() {  running = false;  }
	void reset() {  setOutput(0.0);  }
	void setOutput(double newOutput) {
		// y is written and read in operate(), so it needs to be locked.
		SCOPED_LOCK(getEmMutex());
		y = newOutput;
	}

protected:
	virtual void operate() {
		if (running) {
			y += T_s * gain;
			outputValue->setValue(y);
		}
	}

	double T_s, gain, y;
	bool running;

private:
	DISALLOW_COPY_AND_ASSIGN(Ramp);
};


}
}


#endif /* RAMP_H_ */
