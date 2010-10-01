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
	ArrayEditor();
	explicit ArrayEditor(const T& initialOutputValue);
	virtual ~ArrayEditor();

	System::Input<double>& getElementInput(const size_t i);

protected:
	virtual void operate();
	virtual bool inputsValid();

	void initInputs();

private:
	DISALLOW_COPY_AND_ASSIGN(ArrayEditor);
};


}
}


// include template definitions
#include <barrett/systems/detail/array_editor-inl.h>


#endif /* BARRETT_SYSTEMS_ARRAY_EDITOR_H_ */
