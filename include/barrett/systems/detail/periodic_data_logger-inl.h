/*
 * periodic_data_logger-inl.h
 *
 *  Created on: Jan 6, 2010
 *      Author: dc
 */


#include "../../thread/abstract/mutex.h"


namespace barrett {
namespace systems {


template<typename T, typename LogWriterType>
PeriodicDataLogger<T, LogWriterType>::PeriodicDataLogger(LogWriterType* logWriter, size_t periodMultiplier) :
	System(true), SingleInput<T>(this),
	lw(logWriter), logging(true),
	ecCount(0), ecCountRollover(periodMultiplier) {}

template<typename T, typename LogWriterType>
PeriodicDataLogger<T, LogWriterType>::~PeriodicDataLogger() {
	if (isLogging()) {
		closeLog();
	}
}

template<typename T, typename LogWriterType>
inline bool PeriodicDataLogger<T, LogWriterType>::isLogging() {
	return logging;
}

template<typename T, typename LogWriterType>
void PeriodicDataLogger<T, LogWriterType>::closeLog() {
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

template<typename T, typename LogWriterType>
inline bool PeriodicDataLogger<T, LogWriterType>::inputsValid() {
	ecCount = (ecCount + 1) % ecCountRollover;
	return logging  &&  ecCount == 0  &&  this->input.valueDefined();
}

template<typename T, typename LogWriterType>
inline void PeriodicDataLogger<T, LogWriterType>::operate() {
	lw->putRecord(this->input.getValue());
}


}
}
