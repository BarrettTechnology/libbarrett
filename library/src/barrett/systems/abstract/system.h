/*
 * system.h
 *
 *  Created on: Sep 4, 2009
 *      Author: dc
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_


#include <stdexcept>
#include <list>
#include <string>
#include "../../detail/ca_macro.h"


namespace barrett {
namespace systems {


class System {
public:
	// An abstract class to allow collections of, and operations on
	// generic Inputs
	class AbstractInput {
	protected:
		System& parent;
	public:
		explicit AbstractInput(System* parentSys) :
			parent(*parentSys) {}
		virtual ~AbstractInput() = 0;  // make this class polymorphic

		virtual bool isConnected() const = 0;
	private:
		DISALLOW_COPY_AND_ASSIGN(AbstractInput);
	};

	// An abstract class to allow collections of, and operations on
	// generic Outputs
	class AbstractOutput {
	public:
		AbstractOutput() {}
		virtual ~AbstractOutput() = 0;  // make this class polymorphic
	private:
		DISALLOW_COPY_AND_ASSIGN(AbstractOutput);
	};


	template<typename T> class Input;
	template<typename T> class Output;


	template<typename T>
	class Input : public AbstractInput {
	public:
		class ValueUndefinedError : public std::runtime_error {
		public:
			explicit ValueUndefinedError(const std::string& msg) :
				std::runtime_error(msg) {}
		};

		explicit Input(System* parentSys) :
			AbstractInput(parentSys), output(NULL) {}
		virtual ~Input() {}

		virtual bool isConnected() const;

		bool valueDefined() const;
		const T& getValue() const throw(std::logic_error, ValueUndefinedError);

	protected:
		Output<T>* output;

		// FIXME: do we intend for Input to be subclassed?
		virtual void onValueChanged() const;

	// friends:
		friend class Output<T>;

		template<typename T2>
		friend void connect(System::Output<T2>& output, System::Input<T2>& input)  //NOLINT: see ../helpers.h
		throw(std::invalid_argument);

		template<typename T2>
		friend void reconnect(System::Output<T2>& newOutput, System::Input<T2>& input)  //NOLINT: see ../helpers.h
		throw(std::invalid_argument);

		template<typename T2>
		friend void forceConnect(System::Output<T2>& output, System::Input<T2>& input);  //NOLINT: see ../helpers.h

		template<typename T2>
		friend void disconnect(System::Input<T2>& input)  //NOLINT: see ../helpers.h
		throw(std::invalid_argument);

	private:
		DISALLOW_COPY_AND_ASSIGN(Input);
	};


	template<typename T>
	class Output : public AbstractOutput {
	public:
		class Value {
		public:
			Value(const Output<T>& parentOutput, const T& initialValue) :
				parent(parentOutput), value(new T(initialValue)) {}
			explicit Value(const Output<T>& parentOutput) :
				parent(parentOutput), value(NULL) {}
			~Value();

			void setValue(const T& newValue);
			void setValueUndefined();
		protected:
			const Output<T>& parent;
			T* value;
		private:
			friend class Input<T>;
			DISALLOW_COPY_AND_ASSIGN(Value);
		};

		explicit Output(Value** valuePtr);
		Output(const T& initialValue, Value** valuePtr);
		virtual ~Output() {}

	protected:
		Value value;
		std::list<Input<T>* > inputs;

		void addInput(const Input<T>& input);
		void removeInput(const Input<T>& input);

		void notifyInputs() const;

	// friends:
		friend class Input<T>;

		template<typename T2>
		friend void connect(System::Output<T2>& output, System::Input<T2>& input)  //NOLINT: see ../helpers.h
		throw(std::invalid_argument);

		template<typename T2>
		friend void reconnect(System::Output<T2>& newOutput, System::Input<T2>& input)  //NOLINT: see ../helpers.h
		throw(std::invalid_argument);

		template<typename T2>
		friend void forceConnect(System::Output<T2>& output, System::Input<T2>& input);  //NOLINT: see ../helpers.h

		template<typename T2>
		friend void disconnect(System::Input<T2>& input)  //NOLINT: see ../helpers.h
		throw(std::invalid_argument);

	private:
		DISALLOW_COPY_AND_ASSIGN(Output);
	};

	System() {}
	virtual ~System() {}

protected:
	// use inputs/state/environment to update outputs
	virtual void operate() = 0;

private:
	DISALLOW_COPY_AND_ASSIGN(System);
};


}
}


// include template definitions
#include "./detail/system-input-inl.h"
#include "./detail/system-output-inl.h"
#include "./detail/system-output-value-inl.h"


#endif /* SYSTEM_H_ */