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
	explicit PrintToStream(const std::string& prependedLabel = std::string(),
	                       std::ostream& ostream = std::cout) :
		System(true), SingleInput<T>(this), label(prependedLabel), os(ostream) {}
	virtual ~PrintToStream() {}

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
