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
#include "../ca_macro.h"


namespace Systems {


template<typename T>
class PrintToStream : public System {
// IO
public:	Input<T> input;


protected:
	std::string label;
	std::ostream& os;

	virtual void operate() {
		os << label << input.getValue() << std::endl;
	}

public:
	explicit PrintToStream(const std::string& prependedLabel,
	                       std::ostream& ostream = std::cout) :
		input(this), label(prependedLabel), os(ostream) {}
	virtual ~PrintToStream() {}

private:
	DISALLOW_COPY_AND_ASSIGN(PrintToStream);
};


}


#endif /* PRINT_TO_STREAM_H_ */
