/*
 * system-inl.h
 *
 *  Created on: Feb 28, 2011
 *      Author: dc
 */

//#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/thread/null_mutex.h>


namespace barrett {
namespace systems {


inline thread::Mutex& System::getEmMutex() const
{
	return hasExecutionManager() ? getExecutionManager()->getMutex() : thread::NullMutex::aNullMutex;
}

template<typename T>
void System::Output<T>::Value::delegateTo(Output<T>& delegateOutput)
{
	undelegate();
	delegate = &(delegateOutput.value);
	delegate->delegators.push_back(parentOutput);
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
