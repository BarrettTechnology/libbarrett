/*
 * print_to_screen.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef PRINT_TO_STREAM_H_
#define PRINT_TO_STREAM_H_


#include <iostream>
#include <string>

#include "./abstract/system.h"
#include "../detail/ca_macro.h"


namespace barrett {
namespace systems {


template<typename T>
class PrintToStream : public System {
// IO
public:	Input<T> input;


public:
	explicit PrintToStream(const std::string& prependedLabel = std::string(),
	                       std::ostream& ostream = std::cout) :
		System(true), input(this), label(prependedLabel), os(ostream) {}
	virtual ~PrintToStream() {}

protected:
	std::string label;
	std::ostream& os;

	virtual void operate() {
		os << label << input.getValue() << std::endl;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(PrintToStream);
};


}
}


#endif /* PRINT_TO_STREAM_H_ */
