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

/** Defines barrett::systems::System.
 *
 * @file system.h
 * @date Sep 4, 2009
 * @author Dan Cody
 */


#ifndef BARRETT_SYSTEMS_ABSTRACT_SYSTEM_H_
#define BARRETT_SYSTEMS_ABSTRACT_SYSTEM_H_


#include <string>
#include <cassert>

#include <boost/intrusive/list.hpp>
#include <boost/intrusive/parent_from_member.hpp>

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace systems {


class ExecutionManager;


#define DECLARE_HELPER_FRIENDS  \
	template<typename T2> friend void connect(System::Output<T2>& output, System::Input<T2>& input);  \
	template<typename T2> friend void disconnect(System::Input<T2>& input);  \
	template<typename T2> friend void disconnect(System::Output<T2>& output)


class System {
private:
	typedef uint_fast32_t update_token_type;

public:
	// Forward decls
	class AbstractInput;
	template<typename T> class Input;
	class AbstractOutput;
	template<typename T> class Output;


	explicit System(const std::string& sysName = "System") :
			name(sysName), em(NULL), emDirect(false), ut(UT_NULL) {}
	virtual ~System() { mandatoryCleanUp(); }

	void setName(const std::string& newName) { name = newName; }
	const std::string& getName() const { return name; }

	bool hasExecutionManager() const { return getExecutionManager() != NULL; }
	bool hasDirectExecutionManager() const { return emDirect; }
	ExecutionManager* getExecutionManager() const { return em; }
	thread::Mutex& getEmMutex() const;

protected:
	void mandatoryCleanUp();

	void update(update_token_type updateToken);

	virtual bool inputsValid() /* const */;
	virtual void operate() = 0;
	virtual void invalidateOutputs();

	// If you redefine this method, make sure to call
	// MyBaseClass::onExecutionManagerChanged() in the new version.
	virtual void onExecutionManagerChanged() {}

	std::string name;
	ExecutionManager* em;
	bool emDirect;


public:
	class AbstractInput {
	public:
		AbstractInput(System* parent);
		virtual ~AbstractInput();

		virtual bool valueDefined() const = 0;

		thread::Mutex& getEmMutex() const;

	protected:
		virtual void mandatoryCleanUp();

		System* parentSys;

	private:
		virtual void pushExecutionManager() = 0;
		virtual void unsetExecutionManager() = 0;

		typedef boost::intrusive::list_member_hook<> child_hook_type;
		child_hook_type childHook;

		friend class System;

		DISALLOW_COPY_AND_ASSIGN(AbstractInput);
	};

	class AbstractOutput {
	public:
		AbstractOutput(System* parent);
		virtual ~AbstractOutput();

		thread::Mutex& getEmMutex() const;

	protected:
		virtual void mandatoryCleanUp();

		System* parentSys;

	private:
		virtual void setValueUndefined() = 0;

		virtual ExecutionManager* collectExecutionManager() const = 0;
		virtual void pushExecutionManager() = 0;
		virtual void unsetExecutionManager() = 0;

		typedef boost::intrusive::list_member_hook<> child_hook_type;
		child_hook_type childHook;

		friend class System;

		DISALLOW_COPY_AND_ASSIGN(AbstractOutput);
	};


private:
	static const update_token_type UT_NULL = 0;
	update_token_type ut;


	void setExecutionManager(ExecutionManager* newEm);
	void unsetDirectExecutionManager();
	void unsetExecutionManager();

	typedef boost::intrusive::list_member_hook<> managed_hook_type;
	managed_hook_type managedHook;

	struct StopManagingDisposer {
		void operator() (System* sys) {
			sys->unsetDirectExecutionManager();
		}
	};

	typedef boost::intrusive::list<AbstractInput, boost::intrusive::member_hook<AbstractInput, AbstractInput::child_hook_type, &AbstractInput::childHook> > child_input_list_type;
	child_input_list_type inputs;

	typedef boost::intrusive::list<AbstractOutput, boost::intrusive::member_hook<AbstractOutput, AbstractOutput::child_hook_type, &AbstractOutput::childHook> > child_output_list_type;
	child_output_list_type outputs;


	friend class ExecutionManager;
	DECLARE_HELPER_FRIENDS;


	DISALLOW_COPY_AND_ASSIGN(System);
};


template<typename T> class System::Input : public System::AbstractInput {
public:
	Input(System* parent) : AbstractInput(parent), output(NULL) {}
	virtual ~Input();

	bool isConnected() const { return output != NULL; }
	virtual bool valueDefined() const {
		assert(parentSys != NULL);
		return parentSys->hasExecutionManager()  &&  isConnected()  &&  output->getValueObject()->updateData(parentSys->ut);
	}

	const T& getValue() const {
		assert(valueDefined());

		// valueDefined() calls Output<T>::Value::updateData() for us. Make
		// sure it gets called even if NDEBUG is defined.
#ifdef NDEBUG
		output->getValueObject()->updateData(parentSys->ut);
#endif

		return *(output->getValueObject()->getData());
	}

protected:
	virtual void mandatoryCleanUp();

	Output<T>* output;

private:
	virtual void pushExecutionManager();
	virtual void unsetExecutionManager();

	typedef boost::intrusive::list_member_hook<> connected_hook_type;
	connected_hook_type connectedHook;

	struct DisconnectDisposer {
		void operator() (Input<T>* input) {
			input->output = NULL;
		}
	};

	friend class Output<T>;
	DECLARE_HELPER_FRIENDS;

	DISALLOW_COPY_AND_ASSIGN(Input);
};


namespace detail {
template<typename T> struct IntrusiveDelegateFunctor {
	//Required types
	typedef boost::intrusive::list_member_hook<> hook_type;
	typedef hook_type* hook_ptr;
	typedef const hook_type* const_hook_ptr;
	typedef System::Output<T> value_type;
	typedef value_type* pointer;
	typedef const value_type* const_pointer;

	//Required static functions
	static hook_ptr to_hook_ptr(value_type &value);
	static const_hook_ptr to_hook_ptr(const value_type &value);
	static pointer to_value_ptr(hook_ptr n);
	static const_pointer to_value_ptr(const_hook_ptr n);
};
}

template<typename T> class System::Output : public System::AbstractOutput {
public:
	class Value;

	Output(System* parent, Value** valueHandle) : AbstractOutput(parent), value(this) {
		*valueHandle = &value;
	}
	virtual ~Output();

	// TODO(dc): How should isConnected() treat delegation?
	bool isConnected() const {
		return !inputs.empty()  ||  !value.delegators.empty();
	}

protected:
	virtual void mandatoryCleanUp();

	Value* getValueObject() {
		// TODO(dc): check for cyclic delegation?
		Value* v = &value;
		while (v->delegate != NULL) {
			v = v->delegate;
		}

		return v;
	}

	Value value;

private:
	virtual void setValueUndefined() {
		value.setUndefined();
	}

	virtual ExecutionManager* collectExecutionManager() const;
	virtual void pushExecutionManager();
	virtual void unsetExecutionManager();


	typename detail::IntrusiveDelegateFunctor<T>::hook_type delegateHook;

	typedef boost::intrusive::list<Input<T>, boost::intrusive::member_hook<Input<T>, typename Input<T>::connected_hook_type, &Input<T>::connectedHook> > connected_input_list_type;
	connected_input_list_type inputs;


	friend class Input<T>;
	friend class detail::IntrusiveDelegateFunctor<T>;
	DECLARE_HELPER_FRIENDS;

	DISALLOW_COPY_AND_ASSIGN(Output);
};


template<typename T> class System::Output<T>::Value {
public:
	void setData(const T* newData) { data = newData; }
	void setUndefined() { data = NULL; }

	bool isDefined() const { return data != NULL; }
	const T* getData() const { return data; }

	void delegateTo(Output<T>& delegateOutput);
	void undelegate();

protected:
	Output<T>& parentOutput;
	Value* delegate;
	bool defined;
	const T* data;

private:
	explicit Value(Output<T>* parent) : parentOutput(*parent), delegate(NULL), data(NULL) {}

	bool updateData(update_token_type updateToken) {
		assert(parentOutput.parentSys != NULL);  // TODO(dc): Is this assertion helpful?

		parentOutput.parentSys->update(updateToken);
		return isDefined();
	}

	typedef boost::intrusive::list<Output<T>, boost::intrusive::function_hook<typename detail::IntrusiveDelegateFunctor<T> > > delegate_output_list_type;
	delegate_output_list_type delegators;

	struct UndelegateDisposer {
		void operator() (Output<T>* output) {
			// No need to update the ExecutionManager, this Disposer is only used in ~Output()
			//output->unsetExecutionManager();
			output->value.delegate = NULL;
		}
	};

	friend class Input<T>;
	friend class Output<T>;

	DISALLOW_COPY_AND_ASSIGN(Value);
};


#undef DECLARE_HELPER_FRIENDS


}
}


// include template definitions
#include <barrett/systems/abstract/detail/system-inl.h>

// Always include helper definitions to avoid linker errors
#include <barrett/systems/helpers.h>


#endif /* BARRETT_SYSTEMS_ABSTRACT_SYSTEM_H_ */
