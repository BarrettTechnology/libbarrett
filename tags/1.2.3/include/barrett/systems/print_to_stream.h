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
 * print_to_screen.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_PRINT_TO_STREAM_H_
#define BARRETT_SYSTEMS_PRINT_TO_STREAM_H_


#include <iostream>
#include <string>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<typename T>
class PrintToStream : public System, public SingleInput<T> {
public:
	explicit PrintToStream(ExecutionManager* em, const std::string& prependedLabel = "",
			std::ostream& ostream = std::cout, const std::string& sysName = "PrintToStream") :
		System(sysName), SingleInput<T>(this), label(prependedLabel), os(ostream)
	{
		if (em != NULL) {
			em->startManaging(*this);
		}
	}
	virtual ~PrintToStream() {
		mandatoryCleanUp();
	}

protected:
	std::string label;
	std::ostream& os;

	virtual void operate() {
		os << label << this->input.getValue() << std::endl;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(PrintToStream);
};


}
}


#endif /* BARRETT_SYSTEMS_PRINT_TO_STREAM_H_ */
