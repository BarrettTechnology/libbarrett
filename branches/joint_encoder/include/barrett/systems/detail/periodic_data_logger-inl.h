/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * periodic_data_logger-inl.h
 *
 *  Created on: Jan 6, 2010
 *      Author: dc
 */


#include <boost/thread.hpp>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace systems {


template<typename T, typename LogWriterType>
PeriodicDataLogger<T, LogWriterType>::PeriodicDataLogger(ExecutionManager* em,
		LogWriterType* logWriter, size_t periodMultiplier, const std::string& sysName) :
	System(sysName), SingleInput<T>(this),
	lw(logWriter), logging(true),
	ecCount(0), ecCountRollover(periodMultiplier)
{
	if (em != NULL) {
		em->startManaging(*this);
	}
}

template<typename T, typename LogWriterType>
PeriodicDataLogger<T, LogWriterType>::~PeriodicDataLogger() {
	mandatoryCleanUp();

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
		// One of the functions below is an interruption point. We must disable
		// interruption (for the duration of this scope) so that a
		// boost::thread_interrupted exception doesn't get thrown.
		boost::this_thread::disable_interruption di;

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
