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
 * triggered_data_logger.h
 *
 *  Created on: Dec 24, 2009
 *      Author: dc
 */

#ifndef TRIGGERED_DATA_LOGGER_H_
#define TRIGGERED_DATA_LOGGER_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/log/real_time_writer.h>
#include <barrett/systems/periodic_data_logger.h>


namespace barrett {
namespace systems {


// TODO(dc): add a configuration file interface

template<typename T, typename LogWriterType = log::RealTimeWriter<T> >
class TriggeredDataLogger : public PeriodicDataLogger<T, LogWriterType> {
// IO
public:	System::Input<bool> triggerInput;


public:
	TriggeredDataLogger(ExecutionManager* em, LogWriterType* logWriter, const std::string& sysName = "TriggeredDataLogger") :
			PeriodicDataLogger<T, LogWriterType>(em, logWriter, 1, sysName),
			triggerInput(this) {}
	virtual ~TriggeredDataLogger() { this->mandatoryCleanUp(); }

protected:
	virtual bool inputsValid() {
		return	this->logging  &&  triggerInput.valueDefined()  &&
				triggerInput.getValue() == true  &&  this->input.valueDefined();
	}

private:
	DISALLOW_COPY_AND_ASSIGN(TriggeredDataLogger);
};


}
}


#endif /* TRIGGERED_DATA_LOGGER_H_ */
