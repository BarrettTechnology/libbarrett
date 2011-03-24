/*
 * ExposedIOSystem.h
 *
 *  Created on: Sep 18, 2009
 *      Author: dc
 */

#ifndef EXPOSEDIOSYSTEM_H_
#define EXPOSEDIOSYSTEM_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/abstract/single_io.h>


template<typename T>
class ExposedIOSystem : public barrett::systems::SingleIO<T, T> {
public:
	mutable bool operateCalled;
	mutable bool executionManagerChanged;

	ExposedIOSystem(const std::string& sysName = "ExposedIOSystem") :
		barrett::systems::SingleIO<T, T>(sysName),
		operateCalled(false), executionManagerChanged(false), data() {}
	virtual ~ExposedIOSystem() { this->mandatoryCleanUp(); }

	const T& getInputValue() const {
		return this->input.getValue();
	}

	bool inputValueDefined() const {
		return this->input.valueDefined();
	}

	void setOutputValue(const T& value) {
		data = value;
		this->outputValue->setData(&data);
	}

	void setOutputValueUndefined() {
		this->outputValue->setUndefined();
	}

	void delegateOutputValueTo(barrett::systems::System::Output<T>& delegate) {
		this->outputValue->delegateTo(delegate);
	}

	void undelegate() {
		this->outputValue->undelegate();
	}

protected:
	// This System has no invalid Input state.
	virtual bool inputsValid() {
		return true;
	}

	virtual void operate() {
		operateCalled = true;
	}

	// This System's Outputs are not a function of its Inputs.
	virtual void invalidateOutputs() {
		/* do nothing */
	}

	virtual void onExecutionManagerChanged() {
		// First, call super
		barrett::systems::SingleIO<T, T>::onExecutionManagerChanged();

		executionManagerChanged = true;
	}

	T data;

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedIOSystem);
};


template<typename T>
void checkConnected(barrett::systems::ManualExecutionManager& mem,
					ExposedIOSystem<T>* outSys,
					const ExposedIOSystem<T>& inSys,
					const T& value)
{
	EXPECT_TRUE(inSys.input.isConnected());
	EXPECT_TRUE((outSys->output.isConnected()));

	outSys->operateCalled = false;
	outSys->setOutputValue(value);
	mem.runExecutionCycle();

	EXPECT_TRUE(inSys.inputValueDefined()) << "input value undefined";
	EXPECT_TRUE(outSys->operateCalled) << "System.operate() wasn't called";
	EXPECT_EQ(value, inSys.getInputValue()) << "input has the wrong value";
}

template<typename T>
void checkNotConnected(barrett::systems::ManualExecutionManager& mem,
					   ExposedIOSystem<T>* outSys,
					   const ExposedIOSystem<T>& inSys,
					   const T& value)
{
	outSys->operateCalled = false;
	outSys->setOutputValue(value);
	mem.runExecutionCycle();

	EXPECT_FALSE(outSys->operateCalled) << "System.operate() was called";
}

template<typename T>
void checkDisconnected(const ExposedIOSystem<T>& inSys)
{
	EXPECT_FALSE(inSys.inputValueDefined()) << "input value defined";
	EXPECT_FALSE(inSys.input.isConnected()) << "input thinks it has an output";
}



#endif /* EXPOSEDIOSYSTEM_H_ */
