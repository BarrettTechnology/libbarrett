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


#include <stdexcept>
#include <list>
#include <vector>

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace systems {


class ExecutionManager;


/** An abstract class that encapsulates a chunk of functionality and its various inputs and outputs.
 *
 * @section sec_metaphor Metaphor
 *
 * The System class is named in the same metaphor as the \c system from the phrase "signals and systems", where:
 *   - a \c signal represents the flow of specific information and
 *   - a \c system represents some operation on the \c signals that flow into it.
 *   .
 * A System is a \c block in a "block diagram".
 *
 * A System can have (at its option):
 *   - some number of \ref Input "Input"s
 *   - some number of \ref Output "Output"s
 *   - state (in the form of member variables)
 *   - an asynchronous interface (in the form of member functions)
 *   - etc.
 *
 *
 * @section sec_interface Interface
 *
 * All that a System must have is an operate() member function. The operate() function is where the guts go. It is where a System should use its
 * \ref Input "Input"s and state to update the values of its \ref Output "Output"s. The operate() function should expect to be called automatically when
 * its \ref Input "Input"s have new information. (If greater control is required over when operate() is called, simply reimplement inputsValid().) The method
 * should also be real-time safe and thread-safe.
 *
 * And that's the entire interface for a System: an operate() function and \ref Input "Input"s and \ref Output "Output"s as necessary. A System
 * doesn't need to worry about where its input data is coming from or where its output data is going or when it should perform its operation; \c libbarrett and
 * the user work together to define that.
 *
 * To route information among a set of \ref System "System"s, the library provides a \ref helpers.h "set of functions" (such as barrett::systems::connect())
 * that operate on \ref Input "Input"s and \ref Output "Output"s.
 *
 * @note
 * In general, the types, number, and names of a System's \ref Input "Input"s and \ref Output "Output"s are integral to a user's understanding of how to
 * interact with that System. Thus, we consider a System's \ref Input "Input"s and \ref Output "Output"s to be a part of the public interface. You will see
 * that most of the Systems provided by the library define their \ref Input "Input"s and \ref Output "Output"s as public data members.
 *
 *
 * @section sec_goals Goals
 *
 * The aim of this support structure is to:
 *   - intuitively describe a main-loop using a syntax that mimics the natural language of the problem
 *   - minimize boilerplate for the user
 *   - encourage the user to write clean, reusable code
 *   - make it easier for users to "enter the main-loop" rather than interacting solely with a limited asynchronous interface.
 *
 * The aim of this library is not to be a full-featured block diagraming solution. As such, degenerate conditions (eg. internal
 * cyclic dependencies) are generally not supported.
 *
 *
 * @section sec_examples Examples
 *
 * The following is a a minimal example showing the definition and usage of a new type of System called \c PolynomialEvaluator.
 * @include polynomial_evaluator.cpp
 *
 * Executing this code prints the following to \c stdout:
@verbatim
Result: 2
Result: 0.75
Result: 1
Result: 2.75
Result: 6
@endverbatim
 *
 * \c PolynomialEvaluator could also be implemented as a SingleIO system which defines and initializes the \c input, \c output, and \c outputValue members for us:
@code
#include <vector>
#include <barrett/systems/abstract/single_io.h>

using namespace std;
using namespace barrett;


class PolynomialEvaluator : public systems::SingleIO<double, double> {
public:
	explicit PolynomialEvaluator(const vector<double>& coefficients) :
		systems::SingleIO<double, double>(), coeff(coefficients) {}
	virtual ~PolynomialEvaluator() {}

protected:
	virtual void operate() {
		double x = input.getValue();
		double result = 0.0;

		for (int i = coeff.size()-1; i >= 0; --i) {
			result = coeff[i] + x*result;
		}

		outputValue->setValue(result);
	}

	vector<double> coeff;
};
@endcode
 *
 * @see Input
 * @see Output
 * @see Output::Value
 */
class System {
public:
	/** An abstract class to allow collections of, and operations on
	 * generic \ref Input "Input"s.
	 *
	 * @see Input
	 */
	class AbstractInput {
	public:
		/// @param parentSys A pointer to the System that should be notified
		///        when this Input's value changes.
		explicit AbstractInput(System* parentSys);
		virtual ~AbstractInput();

		void disconnectFromParentSystem() {
			parentSystem = NULL;
		}

		/** Tests if this Input is connected to an Output.
		 *
		 * @retval true if the Input is connected
		 * @retval false otherwise.
		 */
		virtual bool isConnected() const = 0;

		/** Tests if this Input has a defined value.
		 *
		 * An Input's value may be undefined because is it not connected to an Output, or because its Output's value is undefined.
		 *
		 * @retval true if the Input's value is defined
		 * @retval false otherwise.
		 *
		 * @see Output::Value::setValueUndefined()
		 */
		virtual bool valueDefined() const = 0;

		thread::Mutex& getEmMutex();

	protected:
		/// The System that should be notified when this Input's value changes.
		System* parentSystem;

	private:
		DISALLOW_COPY_AND_ASSIGN(AbstractInput);
	};

	/** An abstract class to allow collections of, and operations on
	 * generic \ref Output "Output"s.
	 *
	 * @see Output
	 */
	class AbstractOutput {
	public:
		/** An abstract class to allow collections of, and operations on
		 * generic \ref Output::Value "Output::Value"s.
		 *
		 * @see Output::Value
		 */
		class AbstractValue {
		public:
			explicit AbstractValue(System* parentSys);
			virtual ~AbstractValue();

			void disconnectFromParentSystem() {
				parentSystem = NULL;
			}

			/** Sets the value of the associated Output to a state representing the lack of a value.
			 *
			 * @see Input::valueDefined()
			 */
			virtual void setValueUndefined() = 0;

			thread::Mutex& getEmMutex();

		protected:
			/// The System that should be notified when this Output's value changes.
			System* parentSystem;

		protected:
			DISALLOW_COPY_AND_ASSIGN(AbstractValue);
		};


		AbstractOutput() {}
		virtual ~AbstractOutput();

		/** Tests if this Output is connected to any Inputs.
		 *
		 * @retval true if the Output is connected
		 * @retval false otherwise.
		 */
		virtual bool isConnected() const = 0;

	private:
		DISALLOW_COPY_AND_ASSIGN(AbstractOutput);
	};


	template<typename T> class Input;
	template<typename T> class Output;


	/** The pipe through which data flows into a System.
	 *
	 * Inputs and Outputs transmit a particular type of data that is specified by the template parameter, \c T. An Output can be \ref systems::connect "connect"ed
	 * to an Input that transmits the same data-type. After being \ref systems::connect "connect"ed, any data pushed into the Output by calling
	 * Output::Value::setValue() (see the documentation on Output for more on the Output::Value class) can be accessed by the Input by calling
	 * Input::getValue(). Generally, Output::Value::setValue() and Input::getValue() are called from the System::operate() function of the System that owns the
	 * Input or Output.
	 *
	 * An Output can be connected to many Inputs, but an Input can be connected to only one Output.
	 *
	 *
	 * @section sec_example Example
	 *
	 * The following is an example of a System that has an Input. The Input is named \c idInput, and it transmits data of type \c int. The Input constructor needs
	 * a pointer to the System it should notify when the Input has new data available; we want it to notify this System, so we pass it the \c this pointer.
	 *
	 * @include butterfly_factory.h
	 *
	 *
	 * @tparam T The type of data conveyed.
	 * @see System
	 * @see Output
	 */
	template<typename T>
	class Input : public AbstractInput {
	public:
		/// @param parentSys A pointer to the System that should be notified
		///        when this Input's value changes.
		explicit Input(System* parentSys) :
			AbstractInput(parentSys), output(NULL) {}
		virtual ~Input();

		virtual bool isConnected() const;
		virtual bool valueDefined() const;

		/** Retrieves the Input's value.
		 *
		 * @return The value of the Input as set by the associated Output.
		 * @throws std::logic_error if \c this->valueDefined() would have returned \c false.
		 */
		const T& getValue() const throw(std::logic_error);

	protected:
		Output<T>* output;  ///< Pointer to the associated Output.

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

		template<typename T2>
		friend void disconnect(System::Output<T2>& output);  //NOLINT: see ../helpers.h

	private:
		DISALLOW_COPY_AND_ASSIGN(Input);
	};


	/** The pipe through which data flows out of a System.
	 *
	 * Inputs and Outputs transmit a particular type of data that is specified by the template parameter, \c T. An Output can be \ref systems::connect "connect"ed
	 * to an Input that transmits the same data-type. After being \ref systems::connect "connect"ed, any data pushed into the Output by calling
	 * Output::Value::setValue() (see below for more on the Output::Value class) can be accessed by the Input by calling
	 * Input::getValue(). Generally, Output::Value::setValue() and Input::getValue() are called from the System::operate() function of the System that owns the
	 * Input or Output.
	 *
	 * An Output can be connected to many Inputs, but an Input can be connected to only one Output.
	 *
	 *
	 * @section sec_outputvalue The Output class vs. the Output::Value class
	 *
	 * In laying out the concept of an Output above, we have overlooked an important detail. A conceptual output actually has two very different interfaces:
	 *   - the one that lets you route the information (connecting, disconnecting, etc.)
	 *   - the one that lets you modify the information (setting the output's value).
	 *   .
	 * The former should be accessible to many clients distributed through out the code base. The latter should (in general) be tightly controlled: only the
	 * creator of the output should be allowed to change its value.
	 *
	 * To address this, a conceptual output's interface is split over two separate classes:
	 *   - the Output class that handles the routing interface
	 *   - the Output::Value class that handles the modification interface.
	 *   .
	 * This separates in code the two distinct ways of interacting with a conceptual output.
	 *
	 * When an Output is instantiated, an Output::Value object owned by the Output is also instantiated. The Output keeps its Output::Value object private, except
	 * that a pointer to the Output::Value object is returned by the Output's constructor. This is the only way to get access to the Output::Value object. The
	 * separation between the two interfaces means that a System can freely give out pointers and references to its Outputs (for routing purposes) without giving
	 * out permission to change the \e value of its Outputs.
	 *
	 *
	 * @section sec_example Example
	 *
	 * The following is an example of a System that has an Output. The Output is named \c butterflyOutput, and it transmits data of type \c Butterfly. The
	 * \c butterflyOutputValue object is \c protected and is not given out by the System.
	 *
	 * @include butterfly_factory.h
	 *
	 *
	 * @tparam T The type of data conveyed.
	 * @see System
	 * @see Input
	 * @see Output::Value
	 */
	template<typename T>
	class Output : public AbstractOutput {
	public:

		/** Provides the interface to modify an Output's value.
		 *
		 * An Output::Value object should generally be kept private by the creator of the Output. For more information, see the documentation for Output.
		 *
		 * @see Output
		 */
		class Value : public AbstractValue {
		public:
			virtual ~Value();

			/// Sets the value of the associated Output
			void setValue(const T& newValue);

			/// @copybrief AbstractValue::setValueUndefined()
			/// @copydetails AbstractValue::setValueUndefined()
			virtual void setValueUndefined();

			// TODO(dc): check for self-delegation (direct and indirect)
			/** Make the associated Output transparent by delegating value queries to another Output.
			 *
			 * Once this method is called on an Output's Value object, requests for this Output's value (via connected Inputs) will return instead the value of
			 * the \c delegate Output. This behavior will continue until undelegate(), setValue(), or setValueUndefined() are called.
			 *
			 * Delegations can be chained. Of course, cyclic delegate dependencies are disallowed.
			 *
			 * @param[in] delegate The Output that value queries should be delegated to.
			 */
			void delegateTo(Output<T>& delegate);

			/** Stop delegating to another Output and resume answering value queries with this Output's Value object.
			 *
			 * Calling this method on an Output that is not currently delegating has no effect.
			 */
			void undelegate();

		protected:
			void updateValue();

			Output<T>& parentOutput;
			Value* delegate;
			T* value;

		private:
			Value(System* parentSystem, Output<T>& parentOutput) :
				AbstractValue(parentSystem), parentOutput(parentOutput),
				delegate(NULL), value(NULL) {}

			friend class Input<T>;
			friend class Output;

			DISALLOW_COPY_AND_ASSIGN(Value);
		};

		/**
		 * @details
		 * Instantiates an Output and yields a handle to the new Output's Value object. The Output's initial value will be \ref Input::valueDefined() "undefined".
		 *
		 * @param[out] valuePtr will be filled with a pointer to the Value object. The Value object is owned by the Output.
		 */
		Output(System* parentSystem, Value** valuePtr);

		virtual ~Output();

		virtual bool isConnected() const;

	protected:
		Value* getValueObject();

		Value value;
		std::list<Input<T>* > inputs;
		mutable std::list<Output<T>* > delegators;

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

		template<typename T2>
		friend void disconnect(System::Output<T2>& output);  //NOLINT: see ../helpers.h

	private:
		DISALLOW_COPY_AND_ASSIGN(Output);
	};


	explicit System(bool updateEveryExecutionCycle = false);
	virtual ~System();

	bool isExecutionManaged() const;
	virtual void setExecutionManager(ExecutionManager* newEm);
	ExecutionManager* getExecutionManager() const;
	thread::Mutex& getEmMutex();

	static ExecutionManager* defaultExecutionManager;

protected:
	virtual void update();

	/** Tests if the System's Inputs are in a valid state.
	 *
	 * This function is used as a gate to control the execution of the operate() function. The default behavior is to return \c true only if all of a System's
	 * Inputs have defined values (<tt>Input::valueDefined() == true</tt>). If a different behavior is desired, this method should be re-implemented.
	 *
	 * @retval true if it is ok to execute the operate() function
	 * @retval false if the operate() function should not currently be executed.
	 *
	 * @see operate()
	 */
	virtual bool inputsValid();

	/** The guts of a System.
	 *
	 * This is where a System should update its Outputs using information from its Inputs, state, and environment.
	 *
	 * This function will be called automatically. When new Input data is available, the return value of inputsValid() will be checked. If it returns \c true,
	 * operate() will be called. If it returns \c false, the call will be skipped (until new Input data is available) and invalidateOutputs() will be called instead.
	 *
	 * This function should be real-time safe and thread-safe.
	 *
	 * @see inputsValid()
	 * @see invalidateOutputs()
	 */
	virtual void operate() = 0;

	/** Called instead of operate() if the System's Inputs are not in a valid state.
	 *
	 * The validity of the Inputs is determined by calling inputsValid().
	 *
	 * The default behavior is to call Output::Value::setValueUndefined() on all of the System's Outputs.
	 * If a different behavior is desired, this method should be re-implemented.
	 *
	 * @see operate()
	 */
	virtual void invalidateOutputs();

	virtual bool updateEveryExecutionCycle();

private:
	mutable std::vector<AbstractInput*> inputs;
	mutable std::vector<AbstractOutput::AbstractValue*> outputValues;

	ExecutionManager* executionManager;
	bool alwaysUpdate;

	friend class ExecutionManager;

	DISALLOW_COPY_AND_ASSIGN(System);
};


}
}


// include template definitions
#include <barrett/systems/abstract/detail/system-inl.h>
#include <barrett/systems/abstract/detail/system-input-inl.h>
#include <barrett/systems/abstract/detail/system-output-inl.h>
#include <barrett/systems/abstract/detail/system-output-value-inl.h>


#endif /* BARRETT_SYSTEMS_ABSTRACT_SYSTEM_H_ */
