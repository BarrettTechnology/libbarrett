/*
 * callback.cpp
 *
 *  Created on: Dec 23, 2009
 *      Author: dc
 */

#include <boost/ref.hpp>
#include <boost/function.hpp>

#include <gtest/gtest.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/callback.h>

#include "./exposed_io_system.h"


namespace {
using namespace barrett;

double f1(const double& x) {
	return 10.0 * x;
}

double f2(double x) {
	return 20.0 * x;
}

class Functor1 {
public:
	Functor1() {}

	double operator () (const double& x) {
		return 30.0 * x;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Functor1);
};

class Functor2 {
public:
	Functor2() {}

	double operator () (double x) {
		return 40.0 * x;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Functor2);
};


TEST(CallbackTest, ExactFunctionPointer) {
	double (*fPtr)(const double&) = f1;

	ExposedIOSystem<double> eios;
	systems::Callback<double> cbs(fPtr);

	systems::connect(eios.output, cbs.input);
	systems::connect(cbs.output, eios.input);

	eios.setOutputValue(5.0);
	EXPECT_EQ(f1(5.0), eios.getInputValue());
}

TEST(CallbackTest, EquivalentFunctionPointer) {
	double (*fPtr)(double) = f2;

	ExposedIOSystem<double> eios;
	systems::Callback<double> cbs(fPtr);

	systems::connect(eios.output, cbs.input);
	systems::connect(cbs.output, eios.input);

	eios.setOutputValue(5.0);
	EXPECT_EQ(f2(5.0), eios.getInputValue());
}

TEST(CallbackTest, ExactFunctor) {
	Functor1 fObj;

	ExposedIOSystem<double> eios;
	systems::Callback<double> cbs(boost::ref(fObj));

	systems::connect(eios.output, cbs.input);
	systems::connect(cbs.output, eios.input);

	eios.setOutputValue(5.0);
	EXPECT_EQ(fObj(5.0), eios.getInputValue());
}

TEST(CallbackTest, EquivalentFunctor) {
	Functor2 fObj;

	ExposedIOSystem<double> eios;
	systems::Callback<double> cbs(boost::ref(fObj));

	systems::connect(eios.output, cbs.input);
	systems::connect(cbs.output, eios.input);

	eios.setOutputValue(5.0);
	EXPECT_EQ(fObj(5.0), eios.getInputValue());
}


}
