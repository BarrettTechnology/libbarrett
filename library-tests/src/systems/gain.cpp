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


TEST(GainSystemTest, OutputInitiallyUndefined) {
	Systems::Gain<double> gainsys(12.5);

	EXPECT_FALSE(gainsys.input.valueDefined())
		<< "value defined without input";
}

TEST(GainSystemTest, ConnectsIO) {
	Systems::Gain<double> gainsys(1.0);
	Systems::ExposedIO<double> eios;

	Systems::connect(eios.output, gainsys.input);
	Systems::connect(gainsys.output, eios.input);

	Systems::checkConnected(&eios, eios, 3463.2);
}

TEST(GainSystemTest, MultipliesInput) {
	Systems::Gain<double> gainsys(14.2);
	Systems::ExposedIO<double> eios;

	Systems::connect(eios.output, gainsys.input);
	Systems::connect(gainsys.output, eios.input);

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

// mostly we just want this to compile
TEST(GainSystemTest, IGOCanBeDifferentTypes) {
	Systems::Gain<A, B, C> gainsys(B(-3.0));
	Systems::ExposedIO<A> insys;
	Systems::ExposedIO<C> outsys;

	Systems::connect(insys.output, gainsys.input);
	Systems::connect(gainsys.output, outsys.input);

	insys.setOutputValue(A(9.0));
	EXPECT_EQ(A(9.0) * B(-3.0), outsys.getInputValue())
		<< "did multiplication wrong";
}


}
