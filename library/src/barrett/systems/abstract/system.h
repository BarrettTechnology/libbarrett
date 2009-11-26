/** Defines barrett::systems::System.
 *
 * @file system.h
 * @date Sep 4, 2009
 * @author Dan Cody
 */

/* Copyright 2009 Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */


#ifndef SYSTEM_H_
#define SYSTEM_H_


#include <stdexcept>
#include <list>
#include <vector>
#include <string>
#include "../../detail/ca_macro.h"


namespace barrett {
namespace systems {


/** An abstract class that encapsulates a chunk of functionality and its various inputs and outputs.
 *
 * \section  sec_metaphor Metaphor
 *
 * The System class is named in the same metaphor as the \c system from the phrase "signals and %systems", where:
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
 * \section sec_interface Interface
 *
 * All that a System must have is an operate() member function. The operate() function is where the guts go. It is where a System should use its
 * \ref Input "Input"s and state to update the values of its \ref Output "Output"s. The operate() function should expect to be called automatically when
 * its Input s have new information. (If greater control is required over when operate() is called, simply reimplement inputsValid().) It should also expect to
 * be called from the real-time control thread.
 *
 * And that's the entire interface for a System: an operate() function and \ref Input "Input"s and \ref Output "Output"s as necessary. A System
 * doesn't need to worry about where its input data is coming from or where its output data is going or when it should perform its operation; libbarrett and
 * the user work together to define that.
 *
 * To rout information among a set of \ref System "System"s, the library provides a \ref helpers.h "set of functions" (such as barrett::systems::connect())
 * that operate on \ref Input "Input"s and \ref Output "Output"s.
 *
 *
 * \section sec_goals Goals
 *
 * The aim of this support structure is to:
 *   - intuitively describe a main-loop using a syntax that mimics the natural language of the problem
 *   - minimize boilerplate for the user
 *   - encourage the user to write clean, reusable code
 *   - make it easier for users to "enter the main-loop" rather than interacting solely with a limited asynchronous interface.
 *
 * The aim of this library is not to be a full-featured block diagraming solution. As such, things like internal cyclic dependencies are not supported.
 *
 *
 * \section sec_examples Examples
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
		 */
		virtual bool valueDefined() const = 0;

	protected:
		/// The System that should be notified when this Input's value changes.
		System& parent;

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
		AbstractOutput() {}

		/// @details Virtual in order to make this class polymorphic.
		virtual ~AbstractOutput() = 0;

	private:
		DISALLOW_COPY_AND_ASSIGN(AbstractOutput);
	};


	template<typename T> class Input;
	template<typename T> class Output;


	/** The means by which data flows in to a System.
	 *
	 * An Output can be \ref systems::connect "connected" to an Input, at which point the Output's value can be accessed by the Input. An Output can be connected to many Inputs, but an Input
	 * can only be connected to one Output.
	 *
	 * @tparam T The type of data conveyed.
	 */
	template<typename T>
	class Input : public AbstractInput {
	public:
		/// @param parentSys A pointer to the System that should be notified
		///        when this Input's value changes.
		explicit Input(System* parentSys) :
			AbstractInput(parentSys), output(NULL) {}
		virtual ~Input() {}

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

		/// %Callback from Output::notifyListeners().
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


	/** The means by which data flows out of a System.
	 *
	 * @copydetails Input
	 */
	template<typename T>
	class Output : public AbstractOutput {
	public:
		class Value {
		public:
			Value(const Output<T>& parentOutput, const T& initialValue) :
				parent(parentOutput), delegate(NULL),
				value(new T(initialValue)) {}
			explicit Value(const Output<T>& parentOutput) :
				parent(parentOutput), delegate(NULL), value(NULL) {}
			~Value();

			void setValue(const T& newValue);
			void setValueUndefined();

			// TODO(dc): check for self-delegation (direct and indirect)
			void delegateTo(const Output<T>& delegate);
			void undelegate();
		protected:
			const Output<T>& parent;
			const Value* delegate;
			T* value;
		private:
			friend class Input<T>;
			friend class Output;

			DISALLOW_COPY_AND_ASSIGN(Value);
		};

		explicit Output(Value** valuePtr);
		Output(const T& initialValue, Value** valuePtr);
		virtual ~Output();

	protected:
		const Value* getValueObject() const;

		void addInput(const Input<T>& input);
		void removeInput(const Input<T>& input);
		void addDelegator(const Output<T>& output) const;
		void removeDelegator(const Output<T>& output) const;
		void notifyListeners() const;

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

	private:
		DISALLOW_COPY_AND_ASSIGN(Output);
	};

	System() :
		inputs() {}
	virtual ~System() {}

protected:
	// use inputs/state/environment to update outputs
	virtual void operate() = 0;

	virtual bool inputsValid();

private:
	mutable std::vector<AbstractInput*> inputs;

	DISALLOW_COPY_AND_ASSIGN(System);
};


}
}


// include template definitions
#include "./detail/system-inl.h"
#include "./detail/system-input-inl.h"
#include "./detail/system-output-inl.h"
#include "./detail/system-output-value-inl.h"


#endif /* SYSTEM_H_ */
