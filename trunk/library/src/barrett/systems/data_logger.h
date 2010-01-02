/*
 * data_logger.h
 *
 *  Created on: Dec 24, 2009
 *      Author: dc
 */

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_


#include "../detail/ca_macro.h"
#include "./abstract/system.h"
#include "../log/real_time_writer.h"


namespace barrett {
namespace systems {


template<typename T, typename LogWriterType = log::RealTimeWriter<T> >
class DataLogger : public System {
// IO
public:	Input<T> dataInput;
public:	Input<bool> triggerInput;


public:
	DataLogger(LogWriterType* logWriter, size_t periodMultiplier = 10) :
		System(true), dataInput(this), triggerInput(this),
		lw(logWriter), logging(true), ecCount(0), ecCountRollover(periodMultiplier) {}
	virtual ~DataLogger() {}

	bool isLogging() {
		return logging;
	}

	void closeLog() {
		if (isLogging()) {
			lockExecutionManager();
			logging = false;
			unlockExecutionManager();

			// lw::close() is probably not real-time safe, so keep it out of the critical section.
			lw->close();
			lw = NULL;
		}
	}

protected:
	virtual bool inputsValid() {
		ecCount = (ecCount + 1) % ecCountRollover;

		/* The data we're logging must be present, and we must either be triggered or our trigger must be disabled. If our trigger is disabled, limit rate to
		 * once every ecCountRollover execution cycles. We always need to update triggerInput's value, but we only sometimes need to update dataInput's value,
		 * so we check dataInput second in case we can short-circuit.
		 */
		return	logging  &&  (
					(
						!triggerInput.valueDefined()  &&
						ecCount == 0
					)  ||  triggerInput.getValue() == true
				)  &&  dataInput.valueDefined();
	}

	virtual void operate() {
		lw->putRecord(dataInput.getValue());
	}

	// Optimization: this System has no Outputs to invalidate.
	virtual void invalidateOutputs() {}

	LogWriterType* lw;
	bool logging;
	size_t ecCount, ecCountRollover;

private:
	DISALLOW_COPY_AND_ASSIGN(DataLogger);
};


}
}


#endif /* DATA_LOGGER_H_ */
