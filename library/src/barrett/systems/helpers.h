/** Functions to modify connections between \ref barrett::systems::System::Input "Input"s and \ref barrett::systems::System::Output "Output"s.
 *
 * @file helpers.h
 * @date Sep 4, 2009
 * @author Dan Cody
 */

/* Copyright 2009 Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */


#ifndef BARRETT_SYSTEMS_HELPERS_H_
#define BARRETT_SYSTEMS_HELPERS_H_
// FIXME: rename to manipulators.h?


#include <stdexcept>
#include "./abstract/system.h"


namespace barrett {
namespace systems {


/** Connects a System::Output to a System::Input.
 *
 * @param[in] output The System::Output that will source the data.
 * @param[in] input The System::Input that will receive the data.
 * @throws std::invalid_argument in the case that \c input was already
 *         connected to a System::Output.
 * @see reconnect()
 * @see forceConnect()
 */
template<typename T>
void connect(System::Output<T>& output, System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
throw(std::invalid_argument)
{
	input.lockExecutionManager();

	if (input.isConnected()) {
		throw std::invalid_argument("(systems::connect): "
		                            "Input is already connected to something. "
		                            "Use 'systems::reconnect' instead.");
	}

	input.output = &output;
	output.inputs.push_back(&input);

	input.unlockExecutionManager();
}

/** Takes a System::Input that is connected to a System::Output and reconnects
 * it to another System::Output.
 *
 * @param[in] newOutput The System::Output that \c input will be connected to.
 * @param[in] input The System::Input that will be disconnected and then
 *            connected to \c newOutput.
 * @throws std::invalid_argument in the case that \c input was not already
 *         connected to another System::Output.
 * @see connect()
 */
template<typename T>
void reconnect(System::Output<T>& newOutput, System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
throw(std::invalid_argument)
{
	input.lockExecutionManager();

	if ( !input.isConnected() ) {
		throw std::invalid_argument("(systems::reconnect): "
		                            "Input is not connected to anything. "
		                            "Use 'systems::connect' instead.");
	}

	// disconnect old output
	input.output->inputs.remove(&input);

	// connect new output
	input.output = &newOutput;
	newOutput.inputs.push_back(&input);

	input.unlockExecutionManager();
}

/** Connects a System::Output to a System::Input, even if the System::Input is
 * already connected.
 *
 * This function could also be named \c connectOrReconnect().
 *
 * @param[in] output The System::Output that will source the data.
 * @param[in] input The System::Input that will receive the data.
 * @see connect()
 */
template<typename T>
void forceConnect(System::Output<T>& output, System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
{
	input.lockExecutionManager();

	if (input.isConnected()) {
		// disconnect old output
		input.output->inputs.remove(&input);
	}

	input.output = &output;
	output.inputs.push_back(&input);

	input.unlockExecutionManager();
}

/** Disconnects a System::Input from a System::Output.
 *
 * @param[in] input The System::Input that will stop receiving data from its
 *            System::Output, if any.
 */
template<typename T>
void disconnect(System::Input<T>& input)  //NOLINT: non-const reference parameter chosen to keep syntax clean
throw(std::invalid_argument)
{
	input.lockExecutionManager();

	if ( !input.isConnected() ) {
		throw std::invalid_argument("(systems::disconnect): "
		                            "Input is not connected to anything. "
		                            "Cannot disconnect.");
	}

	input.output->inputs.remove(&input);
	input.output = NULL;

	input.unlockExecutionManager();
}

template<typename T>
void disconnect(System::Output<T>& output)  //NOLINT: non-const reference parameter chosen to keep syntax clean
{
	output.getValueObject()->lockExecutionManager();

	typename std::list<System::Input<T>* >::iterator i;
	for (i = output.inputs.begin(); i != output.inputs.end(); ++i) {
		(*i)->output = NULL;
	}
	output.inputs.clear();

	output.getValueObject()->unlockExecutionManager();
}


}
}


#endif /* BARRETT_SYSTEMS_HELPERS_H_ */
