/*
 * test.cpp
 *
 *  Created on: Mar 12, 2010
 *      Author: dc
 */


#include <iostream>

#include <boost/bind.hpp>

#include <barrett/exception.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/wam.h>


namespace barrett {
namespace systems {

template<size_t DOF>
class MMTest : public SingleIO<units::JointPositions<DOF>, units::JointPositions<DOF> > {
public:
	MMTest() {}
	virtual ~MMTest() {}

protected:
	virtual void operate() {
		this->outputValue->setValue(this->input.getValue());
	}

private:
	DISALLOW_COPY_AND_ASSIGN(MMTest);
};


template<typename OutputType, typename InputType1, typename InputType2>
class Multiplier : public System, public SingleOutput<OutputType> {
// IO
public:	Input<InputType1> input1;
public:	Input<InputType2> input2;


public:
	Multiplier() :
		System(), SingleOutput<OutputType>(this),
		input1(this), input2(this) {}
	virtual ~Multiplier() {}

protected:
	virtual void operate() {
		this->outputValue->setValue(input1.getValue() * input2.getValue());
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Multiplier);
};


template<size_t DOF>
class CrossFader : public System, public SingleInput<units::JointPositions<DOF> > {
public:
	typedef math::Array<DOF> a_type;

// IO
public:		Output<a_type> positiveOutput;
protected:	typename Output<a_type>::Value* positiveOutputValue;
public:		Output<a_type> negativeOutput;
protected:	typename Output<a_type>::Value* negativeOutputValue;


public:
	CrossFader() :
		System(), SingleInput<units::JointPositions<DOF> >(this),
		positiveOutput(this, &positiveOutputValue),
		negativeOutput(this, &negativeOutputValue),
		one(1.0), posThreashold(0.01) {}
	virtual ~CrossFader() {}

protected:
	virtual void operate() {
		tmpOutput = math::saturate(100.0 * math::deadband(math::abs(static_cast<a_type>(this->input.getValue())), posThreashold), one);

		positiveOutputValue->setValue(tmpOutput);
		negativeOutputValue->setValue(one - tmpOutput);
	}

	a_type one;
	a_type posThreashold, velThreashold;
	a_type tmpOutput;

private:
	DISALLOW_COPY_AND_ASSIGN(CrossFader);
};

}
}


namespace math = barrett::math;
namespace units = barrett::units;
namespace systems = barrett::systems;
using barrett::Wam;
using systems::connect;
using systems::reconnect;
using systems::disconnect;


const size_t DOF = 7;
const double T_s = 0.002;


typedef math::Array<DOF> a_type;
typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


jv_type p2vFunc(const jp_type& cutoff, const jv_type& vel, const jp_type& error) {
	return math::saturate(30.0*vel * error, static_cast<a_type>(vel));
}

int main(int argc, char** argv) {
	barrett::installExceptionHandler();  // give us pretty stack traces when things die

	libconfig::Config config;
	config.readFile("/etc/wam/wamg-new.config");
	systems::RealTimeExecutionManager rtem(T_s, false);
	systems::System::defaultExecutionManager = &rtem;


	jp_type cutoff(0.1);
	jv_type vel;
	vel << 0.3, 0.3, 0.5, 0.5, 0.7, 0.7, 0.7;


	// instantiate Systems
	Wam<DOF> wam(config.lookup("wam"));

	systems::ExposedOutput<jp_type> setPoint;

	std::bitset<2> polarity;
	polarity[0] = true;
	polarity[1] = false;
	systems::Summer<jp_type> subtract(polarity);

	systems::CrossFader<DOF> xf;

	systems::Callback<jp_type, jv_type> p2v(boost::bind(p2vFunc, cutoff, vel, _1));
	systems::PIDController<jv_type> jvController(config.lookup("wam.joint_velocity_control"));
	systems::Multiplier<jt_type, a_type, jt_type> velMult;

	systems::PIDController<jp_type> jpController(config.lookup("wam.joint_position_control"));
	systems::Multiplier<jt_type, a_type, jt_type> posMult;

	systems::Summer<jt_type> jtSum;


	// configure Systems
	setPoint.setValue(jp_type(0.0));


    // connect Systems
	connect(setPoint.output, subtract.getInput(0));
	connect(wam.jpOutput, subtract.getInput(1));

	connect(subtract.output, xf.input);
	connect(xf.positiveOutput, velMult.input1);
	connect(xf.negativeOutput, posMult.input1);

	connect(subtract.output, p2v.input);
	connect(p2v.output, jvController.referenceInput);
	connect(wam.jvOutput, jvController.feedbackInput);
	connect(jvController.controlOutput, velMult.input2);

	connect(setPoint.output, jpController.referenceInput);
	connect(wam.jpOutput, jpController.feedbackInput);
	connect(jpController.controlOutput, posMult.input2);

	connect(velMult.output, jtSum.getInput(0));
	connect(posMult.output, jtSum.getInput(1));


	// start the main loop!
	rtem.start();

	bool going = true;
	std::string line;

	bool grav = false, holding = false;
	barrett::math::Array<DOF> gainTmp;

	while (going) {
		std::cout << ">>> ";
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'g':
			grav = !grav;
			wam.gravityCompensate(grav);
			break;

		case 'h':
			holding = !holding;
			if (holding) {
				connect(jtSum.output, wam.input);
			} else {
				disconnect(wam.input);
			}
			break;

		case 's':
			setPoint.setValue(wam.getJointPositions());
			break;

		case 't':
			size_t jointIndex;
			{
				size_t jointNumber;
				std::cout << "\tJoint: ";
				std::cin >> jointNumber;
				jointIndex = jointNumber - 1;

				if (jointIndex >= DOF) {
					std::cout << "\tBad joint number: " << jointNumber;
					break;
				}
			}

			char gainId;
			std::cout << "\tGain identifier (p, i, or d): ";
			std::cin >> line;
			gainId = line[0];

			std::cout << "\tCurrent value: ";
			switch (gainId) {
			case 'p':
				gainTmp = jvController.getKp();
				break;
			case 'i':
				gainTmp = jvController.getKi();
				break;
			case 'd':
				gainTmp = jvController.getKd();
				break;

			default:
				std::cout << "\tBad gain identifier.";
			}
			std::cout << gainTmp[jointIndex] << std::endl;

			std::cout << "\tNew value: ";
			std::cin >> gainTmp[jointIndex];
			switch (gainId) {
			case 'p':
				jvController.setKp(gainTmp);
				break;
			case 'i':
				jvController.setKi(gainTmp);
				break;
			case 'd':
				jvController.setKd(gainTmp);
				break;

			default:
				std::cout << "\tBad gain identifier.";
			}

			break;

		case 'q':
		case 'x':
			going = false;
			break;

		default:
			break;
		}
	}


	rtem.stop();
	return 0;
}
