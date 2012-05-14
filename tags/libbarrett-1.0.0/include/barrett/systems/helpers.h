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

/** Functions to modify connections between \ref barrett::systems::System::Input "Input"s and \ref barrett::systems::System::Output "Output"s.
 *
 * @file helpers.h
 * @date Sep 4, 2009
 * @author Dan Cody
 */


#ifndef BARRETT_SYSTEMS_HELPERS_H_
#define BARRETT_SYSTEMS_HELPERS_H_


#include <cassert>

#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


template<typename T>
void connect(System::Output<T>& output, System::Input<T>& input)
{
	boost::lock_guard<thread::Mutex> inputLg(input.getEmMutex());
	boost::lock_guard<thread::Mutex> outputLg(output.getEmMutex());


	assert( !input.isConnected() );

	input.output = &output;
	output.inputs.push_back(input);

	input.pushExecutionManager();
}

template<typename T>
void reconnect(System::Output<T>& newOutput, System::Input<T>& input)
{
	BARRETT_SCOPED_LOCK(input.getEmMutex());

	assert(input.isConnected());

	disconnect(input);
	connect(newOutput, input);
}

template<typename T>
void forceConnect(System::Output<T>& output, System::Input<T>& input)
{
	BARRETT_SCOPED_LOCK(input.getEmMutex());

	if (input.isConnected()) {
		reconnect(output, input);
	} else {
		connect(output, input);
	}
}

template<typename T>
void disconnect(System::Input<T>& input)
{
	BARRETT_SCOPED_LOCK(input.getEmMutex());

	if (input.isConnected()) {
		input.output->inputs.erase(System::Output<T>::connected_input_list_type::s_iterator_to(input));
		input.unsetExecutionManager();
		input.output = NULL;
	}
}

template<typename T>
void disconnect(System::Output<T>& output)
{
	BARRETT_SCOPED_LOCK(output.getEmMutex());
	assert(output.parentSys != NULL);

	output.inputs.clear_and_dispose(typename System::Input<T>::DisconnectDisposer());
	output.parentSys->unsetExecutionManager();
}


}
}


#endif /* BARRETT_SYSTEMS_HELPERS_H_ */
