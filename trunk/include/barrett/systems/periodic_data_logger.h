/*
 * periodic_data_logger.h
 *
 *  Created on: Dec 24, 2009
 *      Author: dc
 */

#ifndef PERIODIC_DATA_LOGGER_H_
#define PERIODIC_DATA_LOGGER_H_


#include "../detail/ca_macro.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"
#include "../log/real_time_writer.h"


namespace barrett {
namespace systems {


// TODO(dc): add a configuration file interface

template<typename T, typename LogWriterType = log::RealTimeWriter<T> >
class PeriodicDataLogger : public System, public SingleInput<T> {
public:
	// The PeriodicDataLogger owns the logWriter pointer and will delete it when it is no longer needed.
	PeriodicDataLogger(LogWriterType* logWriter, size_t periodMultiplier = 10);
	virtual ~PeriodicDataLogger();

	bool isLogging();
	void closeLog();

protected:
	virtual bool inputsValid();
	virtual void operate();

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


// include template definitions
#include "./detail/periodic_data_logger-inl.h"


#endif /* PERIODIC_DATA_LOGGER_H_ */
