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


namespace barrett {
namespace systems {


// TODO(dc): test this!


class Ramp : public System {
// IO
public:		Output<double> output;
protected:	typename Output<double>::Value* outputValue;


public:
	explicit Ramp(double slope = 1.0, bool updateEveryExecutionCycle = true) :
		System(updateEveryExecutionCycle), output(this, &outputValue),
		T_s(0.0), gain(slope), y(0.0), running(false)
	{
		outputValue->setValue(y);
	}

	void setSamplePeriod(double timeStep) {  T_s = timeStep;  }

	void start() {  running = true;  }
	void stop() {  running = false;  }
	void reset() {
		SCOPED_LOCK(getEmMutex());
		y = 0.0;
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
