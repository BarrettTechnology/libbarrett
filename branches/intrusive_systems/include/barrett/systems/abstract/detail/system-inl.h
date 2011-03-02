/*
 * system-inl.h
 *
 *  Created on: Feb 28, 2011
 *      Author: dc
 */

#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/thread/null_mutex.h>


namespace barrett {
namespace systems {


inline thread::Mutex& System::getEmMutex() const
{
	if (hasExecutionManager()) {
		return getExecutionManager()->getMutex();
	} else {
		return thread::NullMutex::aNullMutex;
	}
}


inline thread::Mutex& System::AbstractInput::getEmMutex() const
{
	if (parentSys != NULL) {
		return parentSys->getEmMutex();
	} else {
		return thread::NullMutex::aNullMutex;
	}
}


inline thread::Mutex& System::AbstractOutput::getEmMutex() const
{
	if (parentSys != NULL) {
		return parentSys->getEmMutex();
	} else {
		return thread::NullMutex::aNullMutex;
	}
}


template<typename T>
System::Input<T>::~Input() {
	if (parentSys == NULL) {
		assert( !isConnected() );
	} else {
		mandatoryCleanUp();
	}
}

template<typename T>
void System::Input<T>::mandatoryCleanUp()
{
	assert(parentSys != NULL);

	BARRETT_SCOPED_LOCK(getEmMutex());

	disconnect(*this);
	AbstractInput::mandatoryCleanUp();
}

template<typename T>
void System::Input<T>::pushExecutionManager()
{
	if (isConnected()) {
		output->parentSys->setExecutionManager(parentSys->em);
	}
}

template<typename T>
void System::Input<T>::unsetExecutionManager()
{
	if (isConnected()) {
		output->parentSys->unsetExecutionManager();
	}
}


template<typename T>
System::Output<T>::~Output() {
	if (parentSys == NULL) {
		assert( !isConnected() );
	} else {
		mandatoryCleanUp();
	}
}

template<typename T>
void System::Output<T>::mandatoryCleanUp()
{
	assert(parentSys != NULL);

	BARRETT_SCOPED_LOCK(getEmMutex());

	disconnect(*this);
	value.undelegate();
	value.delegators.clear_and_dispose(typename Value::UndelegateDisposer());
	AbstractOutput::mandatoryCleanUp();
}

template<typename T>
ExecutionManager* System::Output<T>::collectExecutionManager() const
{
	typename connected_input_list_type::const_iterator i(inputs.begin()), iEnd(inputs.end());
	for (; i != iEnd; ++i) {
		if (i->parentSys->hasExecutionManager()) {
			return i->parentSys->getExecutionManager();
		}
	}

	typename Value::delegate_output_list_type::const_iterator o(value.delegators.begin()), oEnd(value.delegators.end());
	for (; o != oEnd; ++o) {
		if (o->parentSys->hasExecutionManager()) {
			return o->parentSys->getExecutionManager();
		}
	}

	return NULL;
}

template<typename T>
void System::Output<T>::pushExecutionManager()
{
	if (value.delegate != NULL) {
		value.delegate->parentOutput.parentSys->setExecutionManager(parentSys->em);
	}
}

template<typename T>
void System::Output<T>::unsetExecutionManager()
{
	if (value.delegate != NULL) {
		value.delegate->parentOutput.parentSys->unsetExecutionManager();
	}
}

template<typename T>
void System::Output<T>::Value::delegateTo(Output<T>& delegateOutput)
{
	BARRETT_SCOPED_LOCK(parentOutput.getEmMutex());

	undelegate();
	delegate = &(delegateOutput.value);
	delegate->delegators.push_back(parentOutput);

	parentOutput.pushExecutionManager();
}

template<typename T>
void System::Output<T>::Value::undelegate()
{
	BARRETT_SCOPED_LOCK(parentOutput.getEmMutex());

	if (delegate != NULL) {
		delegate->delegators.erase(delegate_output_list_type::s_iterator_to(parentOutput));
		parentOutput.unsetExecutionManager();
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
