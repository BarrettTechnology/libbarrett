#include <vector>
#include <barrett/systems/abstract/system.h>  // our base class
#include <barrett/systems.h>  // includes all non-abstract Systems

using namespace std;
using namespace barrett;


// A System that evaluates a polynomial of the form:
//    result = c[0] + c[1]*x + c[2]*x^2 + ... + c[n]*x^n
// for a given input value, x, and vector of coefficients, c.
class PolynomialEvaluator : public systems::System {
// IO
public:		Input<double> input;
public:		Output<double> output;
protected:	Output<double>::Value* outputValue;

public:
	explicit PolynomialEvaluator(const vector<double>& coefficients) :
		input(this), output(this, &outputValue), coeff(coefficients) {}
	virtual ~PolynomialEvaluator() {}

protected:
	// implement System::operate()
	virtual void operate() {
		double x = input.getValue();  // read our input
		double result = 0.0;

        // operate on our input value and state
		for (int i = coeff.size()-1; i >= 0; --i) {
			result = coeff[i] + x*result;
		}

		outputValue->setValue(result);  // update our output
	}

	vector<double> coeff;
};


int main() {
	// make vector of coefficients
	double coeffArray[] = { 1, 2, 3 };
	vector<double> coeff(coeffArray,
			coeffArray + sizeof(coeffArray) / sizeof(double));

	// install execution manager
	systems::ManualExecutionManager mem(0.0);
	systems::System::defaultExecutionManager = &mem;

	// instantiate Systems
	systems::ExposedOutput<double> eoSys;
	PolynomialEvaluator peSys(coeff);
	systems::PrintToStream<double> printSys("Result: ");

	// make connections between Systems
	systems::connect(eoSys.output, peSys.input);
	systems::connect(peSys.output, printSys.input);

    // push data into peSys' input and run an execution cycle,
	// causing peSys::operate() to be called
	eoSys.setValue(-1);
	mem.runExecutionCycle();
	eoSys.setValue(-0.5);
	mem.runExecutionCycle();
	eoSys.setValue(0);
	mem.runExecutionCycle();
	eoSys.setValue(0.5);
	mem.runExecutionCycle();
	eoSys.setValue(1);
	mem.runExecutionCycle();
	
	return 0;
}
