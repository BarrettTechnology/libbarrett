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
 * array_editor.h
 *
 *  Created on: Nov 12, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_ARRAY_EDITOR_H_
#define BARRETT_SYSTEMS_ARRAY_EDITOR_H_


#include <boost/array.hpp>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {

// TODO(dc): test!

template <typename T>
class ArrayEditor : public SingleIO<T, T> {
// IO
// protected because of variable number of inputs
protected:	boost::array<System::Input<double>*, T::SIZE> elementInputs;


public:
	explicit ArrayEditor(const std::string& sysName = "ArrayEditor");
	virtual ~ArrayEditor();

	System::Input<double>& getElementInput(const size_t i);

protected:
	virtual void operate();
	virtual bool inputsValid();

	T data;

private:
	DISALLOW_COPY_AND_ASSIGN(ArrayEditor);
};


}
}


// include template definitions
#include <barrett/systems/detail/array_editor-inl.h>


#endif /* BARRETT_SYSTEMS_ARRAY_EDITOR_H_ */
