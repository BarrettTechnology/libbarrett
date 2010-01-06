/*
 * triggered_data_logger.h
 *
 *  Created on: Dec 24, 2009
 *      Author: dc
 */

#ifndef TRIGGERED_DATA_LOGGER_H_
#define TRIGGERED_DATA_LOGGER_H_


#include "../detail/ca_macro.h"
#include "../log/real_time_writer.h"
#include "./periodic_data_logger.h"


namespace barrett {
namespace systems {


template<typename T, typename LogWriterType = log::RealTimeWriter<T> >
class TriggeredDataLogger : public PeriodicDataLogger<T, LogWriterType> {
// IO
public:	System::Input<bool> triggerInput;


public:
	TriggeredDataLogger(LogWriterType* logWriter) :
			PeriodicDataLogger<T, LogWriterType>(logWriter),
			triggerInput(this) {}

	virtual ~TriggeredDataLogger() {}

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
