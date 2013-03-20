/*
 * cv_moves.cpp
 *
 *  Created on: Feb 22, 2012
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;


template<typename T1, typename T2, typename OutputType>
class Multiplier : public systems::System, public systems::SingleOutput<OutputType> {
public:	Input<T1> input1;
public:	Input<T2> input2;

public:
	Multiplier(std::string sysName = "Multiplier") :
		systems::System(sysName), systems::SingleOutput<OutputType>(this), input1(this), input2(this)
	{}
	virtual ~Multiplier() { mandatoryCleanUp(); }

protected:
	OutputType data;
	virtual void operate() {
		data = input1.getValue() * input2.getValue();
		this->outputValue->setData(&data);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Multiplier);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // In case we use vectorizable Eigen types.
};


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();


	const double SPEED = 0.2;  // m/s
	const double TD = 0.5;  // s

	systems::ExposedOutput<cp_type> direction(cp_type(0.0,0.0,1.0).normalized());
	systems::ExposedOutput<cp_type> offset;
	systems::Ramp param(pm.getExecutionManager(), SPEED);
	Multiplier<double,cp_type, cp_type> linear;
	systems::Summer<cp_type> affine;

	connect(param.output, linear.input1);
	connect(direction.output, linear.input2);
	connect(linear.output, affine.getInput(0));
	connect(offset.output, affine.getInput(1));

	while (true) {
		printf("Press [Enter] to move.");
		waitForEnter();

		param.stop();  // Double check that it's stopped
		param.setOutput(0.0);
		offset.setValue(wam.getToolPosition());

		wam.trackReferenceSignal(affine.output);
		param.smoothStart(0.5);

		printf("Press [Enter] to stop.");
		waitForEnter();
		param.smoothStop(TD);

		printf("Press [Enter] to idle.");
		waitForEnter();
		wam.idle();
	}

	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
