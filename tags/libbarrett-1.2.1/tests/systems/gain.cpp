/*
 * gain.cpp
 *
 *  Created on: Sep 28, 2009
 *      Author: dc
 */

#include <ostream>
#include <gtest/gtest.h>
#include <barrett/systems/gain.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/helpers.h>
#include "./exposed_io_system.h"


namespace {
using namespace barrett;


class GainSystemTest : public ::testing::Test {
public:
	GainSystemTest() {
		mem.startManaging(eios);
	}

protected:
	systems::ManualExecutionManager mem;
	ExposedIOSystem<double> eios;
};


TEST_F(GainSystemTest, OutputInitiallyUndefined) {
	systems::Gain<double> gainSys(12.5);

	EXPECT_FALSE(gainSys.input.valueDefined())
		<< "value defined without input";
}

TEST_F(GainSystemTest, ConnectsIO) {
	systems::Gain<double> gainSys(1.0);

	systems::connect(eios.output, gainSys.input);
	systems::connect(gainSys.output, eios.input);

	checkConnected(mem, &eios, eios, 3463.2);
}

TEST_F(GainSystemTest, MultipliesInput) {
	systems::Gain<double> gainSys(14.2);

	systems::connect(eios.output, gainSys.input);
	systems::connect(gainSys.output, eios.input);

	eios.setOutputValue(-38.52);
	mem.runExecutionCycle();
	EXPECT_EQ(14.2 * -38.52, eios.getInputValue());
}

TEST_F(GainSystemTest, SetGain) {
	systems::Gain<double> gainSys(14.2);

	systems::connect(eios.output, gainSys.input);
	systems::connect(gainSys.output, eios.input);

	eios.setOutputValue(-38.52);
	mem.runExecutionCycle();
	EXPECT_EQ(14.2 * -38.52, eios.getInputValue());

	gainSys.setGain(-3.8);
	mem.runExecutionCycle();
	EXPECT_EQ(-3.8 * -38.52, eios.getInputValue());
}




using std::ostream;
class A;
class B;
class C;

class A {
	friend const C operator * (const B& b, const A& a);
private:
	float value;
public:
	A() : value(0.0) {}
	explicit A(float value) :
		value(value) {}
};

class B {
	friend const C operator * (const B& b, const A& a);
private:
	float value;
public:
	explicit B(float value) :
		value(value) {}
};
class C {
	friend ostream& operator<<(ostream& os, C c);
private:
	float value;
public:
	C() : value(0.0) {}
	explicit C(float value) :
		value(value) {}
	bool operator== (const C& other) const {
		return value == other.value;
	}
};
const C operator* (const B& b, const A& a) {
	return C(a.value * b.value);
}
ostream& operator<<(ostream& os, C c) {
	os << c.value;
	return os;
}

// mostly, we just want this to compile
TEST_F(GainSystemTest, IGOCanBeDifferentTypes) {
	systems::Gain<A, B, C> gainSys(B(-3.0));
	ExposedIOSystem<A> out;
	ExposedIOSystem<C> in;

	mem.startManaging(in);
	systems::connect(gainSys.output, in.input);
	systems::connect(out.output, gainSys.input);

	out.setOutputValue(A(9.0));
	mem.runExecutionCycle();
	EXPECT_EQ(B(-3.0) * A(9.0), in.getInputValue())
		<< "did multiplication wrong";
}


}
