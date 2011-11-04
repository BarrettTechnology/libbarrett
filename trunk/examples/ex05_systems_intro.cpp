/* ex05_systems_intro.cpp
 *
 * This example introduces libbarrett's concept of "Systems". Systems are used
 * to describe the calculations that are performed in the real time control-loop
 * that runs the WAM. The code below defines and uses a simple System. For
 * clarity, this example does not execute any code in real time.
 *
 * Systems are designed to make it easy to have low level control over the
 * behavior of the WAM's control loop with out getting bogged down in the
 * details of real time programming. It's not necessary to understand Systems if
 * you don't need this functionality.
 *
 * Systems are analogous to blocks in a block diagram. They have zero or more
 * System::Inputs and zero or more System::Outputs. Inputs and Outputs are pipes
 * through which data flow. An Output can be connected to many Inputs using the
 * systems::connect() function. An Input can only be connected to one Output at
 * a time.
 *
 * Each System encodes a specific "behavior": an operation that it performs on
 * the data it receives from its Inputs in order to produce the data it puts
 * into its Outputs. The creator of the System is responsible for implementing
 * this behavior in the System::operate() function. libbarrett has many built-in
 * Systems that live in the barrett/systems/ folder, such as:
 *   - systems::Callback for quickly wrapping a function in a System
 *   - systems::ExposedOutput for taking a piece of data and making it available
 *     to other Systems (see example below)
 *   - systems::FirstOrderFilter for filtering a stream of data
 *   - systems::Gain for multiplying by a constant
 *   - systems::PIDController for making simple feedback control systems
 *   - systems::Summer for adding two or more streams of data
 *   - systems::Wam for interacting with a WAM in real time (can also be used to
 *     interact asynchronously, as shown in Examples 1 through 4)
 *
 * A System's operate() function is never called directly. Instead, a
 * systems::ExecutionManager handles the task of figuring out which operate()
 * functions need to be called and making sure that the necessary Input data is
 * valid and updated in time. (Much of this default behavior can be controlled
 * by re-implementing virtual functions from the systems::System class.) There
 * are multiple kinds of ExecutionManager, the main ones being
 * systems::ManualExecutionManager and systems::RealTimeExectuionManager. The
 * simple example below uses a ManualExecutionManager. The WAM's real time
 * control loop uses a RealTimeExecutionManager.
 */


#include <vector>
#include <string>

// Base class: barrett::systems::System
#include <barrett/systems/abstract/system.h>

// Includes the rest of the barrett::systems namespace, which contains all
// non-abstract Systems
#include <barrett/systems.h>


using namespace barrett;


// A System that evaluates a polynomial of the form:
//    result = c[0] + c[1]*x + c[2]*x^2 + ... + c[n]*x^n
// for a given input value, x, and vector of coefficients, c.
class PolynomialEvaluator : public systems::System {
// IO
// Marked as "public" because Inputs and Output are (except in special cases)
// part of a System's public interface.
public:		Input<double> input;
public:		Output<double> output;

// Marked as "protected" because this object lets us change the value of an
// Output, which should only be allowed from within this System.
protected:	Output<double>::Value* outputValue;

public:
	// Every System has a human readable name. It's good practice to provide an
	// appropriate default. Notice that outputValue is associated with output
	// via output's constructor.
	explicit PolynomialEvaluator(const std::vector<double>& coefficients,
			const std::string& sysName = "PolynomialEvaluator") :
		systems::System(sysName), input(this), output(this, &outputValue),
		coeff(coefficients) {}

	// Every System is required to call System::mandatoryCleanUp() in its
	// destructor, preferably as early as possible. It's common for libbarrett
	// to be used in a multi-threaded environment. This function cleans up all
	// of libbarrett's references to this System so that the library won't try
	// to interact with it from Thread A while it's in the process of being
	// destroyed in Thread B. If you forget this, you may occasionally see your
	// program crash with the message: "Pure virtual function called".
	virtual ~PolynomialEvaluator() { mandatoryCleanUp(); }

protected:
	double result;

	// Implement System::operate(). The operate() function must be declared with
	// the "protected" access specifier.
	virtual void operate() {
		const double& x = input.getValue();  // Pull data from the input
		result = 0.0;

        // Operate on the input value and state
		for (int i = coeff.size()-1; i >= 0; --i) {
			result = coeff[i] + x*result;
		}

		outputValue->setData(&result);  // Push data into the output
	}

	std::vector<double> coeff;
};


int main() {
	// Make vector of coefficients
	double coeffArray[] = { 1.0, 2.0, 3.0 };
	std::vector<double> coeff(coeffArray,
			coeffArray + sizeof(coeffArray) / sizeof(double));

	// Create execution manager
	systems::ManualExecutionManager mem;

	// Instantiate Systems
	systems::ExposedOutput<double> eoSys;
	PolynomialEvaluator peSys(coeff);
	systems::PrintToStream<double> printSys(&mem, "Result: ");

	// Make connections between Systems
	systems::connect(eoSys.output, peSys.input);
	systems::connect(peSys.output, printSys.input);


    // Push data into peSys' input and run an execution cycle,
	// causing peSys::operate() to be called
	eoSys.setValue(-1.0);
	mem.runExecutionCycle();
	eoSys.setValue(-0.5);
	mem.runExecutionCycle();
	eoSys.setValue(0.0);
	mem.runExecutionCycle();
	eoSys.setValue(0.5);
	mem.runExecutionCycle();
	eoSys.setValue(1.0);
	mem.runExecutionCycle();
	

	return 0;
}
