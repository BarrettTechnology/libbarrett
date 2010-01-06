/*
 * periodic_data_logger.h
 *
 *  Created on: Dec 24, 2009
 *      Author: dc
 */

#ifndef PERIODIC_DATA_LOGGER_H_
#define PERIODIC_DATA_LOGGER_H_


#include "../detail/ca_macro.h"
#include "../thread/abstract/mutex.h"
#include "./abstract/system.h"
#include "../log/real_time_writer.h"


namespace barrett {
namespace systems {


template<typename T, typename LogWriterType = log::RealTimeWriter<T> >
class PeriodicDataLogger : public System {
// IO
public:	Input<T> input;


public:
	PeriodicDataLogger(LogWriterType* logWriter, size_t periodMultiplier = 10) :
		System(true), input(this),
		lw(logWriter), logging(true),
		ecCount(0), ecCountRollover(periodMultiplier) {}

	virtual ~PeriodicDataLogger() {
		if (isLogging()) {
			closeLog();
		}
	}

	bool isLogging() {
		return logging;
	}

	void closeLog() {
		if (isLogging()) {
			thread::Mutex& emMutex = getEmMutex();
			emMutex.lock();
			logging = false;
			emMutex.unlock();

			// lw::close() is probably not real-time safe, so keep it out of the critical section.
			lw->close();
			delete lw;
			lw = NULL;
		}
	}

protected:
	virtual bool inputsValid() {
		ecCount = (ecCount + 1) % ecCountRollover;
		return logging  &&  ecCount == 0  &&  input.valueDefined();
	}

	virtual void operate() {
		lw->putRecord(input.getValue());
	}

	// Optimization: this System has no Outputs to invalidate.
	virtual void invalidateOutputs() {}

	LogWriterType* lw;
	bool logging;
	size_t ecCount, ecCountRollover;

private:
	DISALLOW_COPY_AND_ASSIGN(PeriodicDataLogger);
};


}
}


#endif /* PERIODIC_DATA_LOGGER_H_ */
