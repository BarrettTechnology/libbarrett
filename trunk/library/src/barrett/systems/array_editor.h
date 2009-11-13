/*
 * array_editor.h
 *
 *  Created on: Nov 12, 2009
 *      Author: dc
 */

#ifndef ARRAY_EDITOR_H_
#define ARRAY_EDITOR_H_


#include <boost/array.hpp>

#include "../detail/ca_macro.h"
#include "./abstract/system.h"
#include "./abstract/single_io.h"


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
	void initInputs();
	virtual void operate();

private:
	DISALLOW_COPY_AND_ASSIGN(ArrayEditor);
};


}
}


// include template definitions
#include "./detail/array_editor-inl.h"


#endif /* ARRAY_EDITOR_H_ */
