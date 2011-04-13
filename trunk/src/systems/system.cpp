/*
 * system.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */


#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/system.h>


namespace barrett {
namespace systems {


void System::update(uint_fast32_t ut)
{
	// Check if an update is needed
	if (ut == UT_NULL  ||  ut != updateToken) {
		updateToken = ut;
	} else {
		return;
	}

	if (inputsValid()) {
		operate();
	} else {
		invalidateOutputs();
	}
}

bool System::inputsValid() const
{
	child_input_list_type::const_iterator i(inputs.begin()), iEnd(inputs.end());
	for (; i != iEnd; ++i) {
		if ( !i->valueDefined() ) {
			return false;
		}
	}
	return true;
}

void System::invalidateOutputs()
{
	child_output_list_type::iterator i(outputs.begin()), iEnd(outputs.end());
	for (; i != iEnd; ++i) {
		i->setValueUndefined();
	}
}


template<typename T>
void System::Output<T>::Value::delegateTo(Output<T>& delegateOutput)
{
	undelegate();
	delegate = &(delegateOutput.value);
	delegate->delegators.push_back(&parentOutput);
}

template<typename T>
void System::Output<T>::Value::undelegate()
{
	if (delegate != NULL) {
		delegate->delegators.erase(delegate_output_list_type::s_iterator_to(parentOutput));
		delegate = NULL;
	}
}


namespace detail {
template<typename T> inline typename IntrusiveDelegateFunctor<T>::hook_ptr IntrusiveDelegateFunctor<T>::to_hook_ptr(IntrusiveDelegateFunctor<T>::value_type &value)
{
	return &value.delegateHook;
}
template<typename T> inline typename IntrusiveDelegateFunctor<T>::const_hook_ptr IntrusiveDelegateFunctor<T>::to_hook_ptr(const IntrusiveDelegateFunctor<T>::value_type &value)
{
	return &value.delegateHook;
}
template<typename T> inline typename IntrusiveDelegateFunctor<T>::pointer IntrusiveDelegateFunctor<T>::to_value_ptr(IntrusiveDelegateFunctor<T>::hook_ptr n)
{
	return boost::intrusive::get_parent_from_member<System::Output<T> >(n, &System::Output<T>::delegateHook);
}
template<typename T> inline typename IntrusiveDelegateFunctor<T>::const_pointer IntrusiveDelegateFunctor<T>::to_value_ptr(IntrusiveDelegateFunctor<T>::const_hook_ptr n)
{
	return boost::intrusive::get_parent_from_member<System::Output<T> >(n, &System::Output<T>::delegateHook);
}
}


}
}
