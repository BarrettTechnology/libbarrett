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
 * exposed_output.h
 *
 *  Created on: Nov 13, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_EXPOSED_OUTPUT_H_
#define BARRETT_SYSTEMS_EXPOSED_OUTPUT_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template <typename T>
class ExposedOutput : public System, public SingleOutput<T> {
public:
	ExposedOutput() :
		SingleOutput<T>(this) {}
	explicit ExposedOutput(const T& initialValue) :
		SingleOutput<T>(this)
	{
		this->outputValue->setValue(initialValue);
	}

	void setValue(const T& value) {
		BARRETT_SCOPED_LOCK(getEmMutex());
		this->outputValue->setValue(value);
	}
	void setValueUndefined() {
		BARRETT_SCOPED_LOCK(getEmMutex());
		this->outputValue->setValueUndefined();
	}
	void delegateTo(const System::Output<T>& delegate) {
		BARRETT_SCOPED_LOCK(getEmMutex());
		this->outputValue->delegateTo(delegate);
	}

protected:
	virtual void operate() {  /* do nothing */  }

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedOutput);
};


}
}


#endif /* BARRETT_SYSTEMS_EXPOSED_OUTPUT_H_ */
