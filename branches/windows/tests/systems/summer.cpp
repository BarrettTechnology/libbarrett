/*
 * summer.cpp
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <barrett/detail/stl_utils.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/helpers.h>
#include <barrett/systems/constant.h>
#include <barrett/systems/summer.h>

#include "exposed_io_system.h"


namespace {
using namespace barrett;


// TODO(dc): actually test this
//class SummerTest : public ::testing::Test {
//protected:
//	systems::Summer<double> summer;
//};

template<typename T, size_t N>
void testSummer(systems::Summer<T,N>& summer, const std::vector<T>& inputs, T expected) {
	std::vector<systems::Constant<T>*> constants(N, NULL);

	for (size_t i = 0; i < N; ++i) {
		constants[i] = new systems::Constant<T>(inputs[i]);
		systems::connect(constants[i]->output, summer.getInput(i));
	}

	systems::ManualExecutionManager mem;
	ExposedIOSystem<T> eios;
	mem.startManaging(eios);
	systems::connect(summer.output, eios.input);

	mem.runExecutionCycle();
	EXPECT_EQ(expected, eios.getInputValue());

	detail::purge(constants);
}


TEST(SummerTest, Double2) {
	const size_t N = 2;

	{
		systems::Summer<double,N> s;
		std::vector<double> inputs(N, 1.0);
		testSummer(s, inputs, N*1.0);
	}
	{
		systems::Summer<double,N> s(std::string("--"));
		std::vector<double> inputs(N, 1.0);
		testSummer(s, inputs, N*-1.0);
	}
	{
		systems::Summer<double,N> s("+-");
		std::vector<double> inputs(N, 1.0);
		testSummer(s, inputs, 0.0);
	}
}

TEST(SummerTest, Double6) {
	const size_t N = 6;

	{
		systems::Summer<double,N> s;
		std::vector<double> inputs(N, 1.0);
		testSummer(s, inputs, N*1.0);
	}
	{
		systems::Summer<double,N> s(std::string("------"));
		std::vector<double> inputs(N, 1.0);
		testSummer(s, inputs, N*-1.0);
	}
	{
		systems::Summer<double,N> s("+--++-");
		std::vector<double> inputs(N, 1.0);
		testSummer(s, inputs, 0.0);
	}
}


}
