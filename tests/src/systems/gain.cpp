/*
 * gain.cpp
 *
 *  Created on: Sep 28, 2009
 *      Author: dc
 */

#include <ostream>
#include <gtest/gtest.h>
#include <barrett/systems.h>
#include "./exposed_io_system.h"


namespace {
using namespace barrett;


TEST(GainSystemTest, OutputInitiallyUndefined) {
	systems::Gain<double> gainsys(12.5);

	EXPECT_FALSE(gainsys.input.valueDefined())
		<< "value defined without input";
}

TEST(GainSystemTest, ConnectsIO) {
	systems::Gain<double> gainsys(1.0);
	ExposedIOSystem<double> eios;

	systems::connect(eios.output, gainsys.input);
	systems::connect(gainsys.output, eios.input);

	checkConnected(&eios, eios, 3463.2);
}

TEST(GainSystemTest, MultipliesInput) {
	systems::Gain<double> gainsys(14.2);
	ExposedIOSystem<double> eios;

	systems::connect(eios.output, gainsys.input);
	systems::connect(gainsys.output, eios.input);

	eios.setOutputValue(-38.52);
	EXPECT_EQ(14.2 * -38.52, eios.getInputValue());
}


using std::ostream;
class A;
class B;
class C;

class A {
	friend const C operator * (const A& a, const B& b);
private:
	float value;
public:
	explicit A(float value) :
		value(value) {}
};

class B {
	friend const C operator * (const A& a, const B& b);
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
	explicit C(float value) :
		value(value) {}
	bool operator== (const C& other) const {
		return value == other.value;
	}
};
const C operator* (const A& a, const B& b) {
	return C(a.value * b.value);
}
ostream& operator<<(ostream& os, C c) {
	os << c.value;
	return os;
}

// mostly, we just want this to compile
TEST(GainSystemTest, IGOCanBeDifferentTypes) {
	systems::Gain<A, B, C> gainsys(B(-3.0));
	ExposedIOSystem<A> insys;
	ExposedIOSystem<C> outsys;

	systems::connect(insys.output, gainsys.input);
	systems::connect(gainsys.output, outsys.input);

	insys.setOutputValue(A(9.0));
	EXPECT_EQ(A(9.0) * B(-3.0), outsys.getInputValue())
		<< "did multiplication wrong";
}


}
