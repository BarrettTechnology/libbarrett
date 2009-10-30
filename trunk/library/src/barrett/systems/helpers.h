/*
 * connect.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef HELPERS_H_
#define HELPERS_H_
// FIXME: rename to manipulators.h?

#include <stdexcept>
#include "./abstract/system.h"


namespace barrett {
namespace systems {


template<typename T>
void connect(System::Output<T>& output, System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
throw(std::invalid_argument)
{
	if (input.output != NULL) {
		throw std::invalid_argument("(systems::connect): "
		                            "Input is already connected to something. "
		                            "Use 'systems::reconnect' instead.");
	}

	input.output = &output;
	output.addInput(input);
	// input.onValueChanged();  // FIXME: should this be here?
}

template<typename T>
void reconnect(System::Output<T>& newOutput, System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
throw(std::invalid_argument)
{
	if (input.output == NULL) {
		throw std::invalid_argument("(systems::reconnect): "
		                            "Input is not connected to anything. "
		                            "Use 'systems::connect' instead.");
	}

	// disconnect old output
	input.output->removeInput(input);

	// connect new output
	input.output = &newOutput;
	newOutput.addInput(input);
	// input.onValueChanged();		// FIXME: should this be here?
}

// this function could also be named connectOrReconnect
template<typename T>
void forceConnect(System::Output<T>& output, System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
{
	if (input.output != NULL) {
		// disconnect old output
		input.output->removeInput(input);
	}

	input.output = &output;
	output.addInput(input);
	// input.onValueChanged();  // FIXME: should this be here?
}

template<typename T>
void disconnect(System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
throw(std::invalid_argument)
{
	if (input.output == NULL) {
		throw std::invalid_argument("(systems::disconnect): "
		                            "Input is not connected to anything. "
		                            "Cannot disconnect.");
	}

	input.output->removeInput(input);
	input.output = NULL;
}


}
}

// TODO(dc): void disconnect(System::Output<T>& output)?


#endif /* HELPERS_H_ */
