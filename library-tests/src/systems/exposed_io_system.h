/*
 * ExposedIOSystem.h
 *
 *  Created on: Sep 18, 2009
 *      Author: dc
 */

#ifndef EXPOSEDIOSYSTEM_H_
#define EXPOSEDIOSYSTEM_H_


#include <barrett/systems.h>
#include <barrett/detail/ca_macro.h>


template<typename T>
class ExposedIOSystem : public barrett::systems::SingleIO<T, T> {
public:
	mutable bool operateCalled;

	ExposedIOSystem() :
		operateCalled(false) {}

	explicit ExposedIOSystem(const T& initialValue) :
		barrett::systems::SingleIO<T, T>(initialValue), operateCalled(false) {}

	const T& getInputValue() const {
		return this->input.getValue();
	}

	bool inputValueDefined() const {
		return this->input.valueDefined();
	}

	void setOutputValue(const T& value) {
		this->outputValue->setValue(value);
	}

	void setOutputValueUndefined() {
		this->outputValue->setValueUndefined();
	}

protected:
	virtual void operate() {
		operateCalled = true;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ExposedIOSystem);
};


template<typename T>
void checkConnected(ExposedIOSystem<T>* outSys,
					const ExposedIOSystem<T>& inSys,
					const T& value)
{
	inSys.operateCalled = false;
	outSys->setOutputValue(value);

	EXPECT_TRUE(inSys.operateCalled) << "System.operate() wasn't called";
	EXPECT_TRUE(inSys.inputValueDefined()) << "input value undefined";
	EXPECT_EQ(value, inSys.getInputValue()) << "input has the wrong value";
}

template<typename T>
void checkNotConnected(ExposedIOSystem<T>* outSys,
					   const ExposedIOSystem<T>& inSys,
					   const T& value)
{
	inSys.operateCalled = false;
	outSys->setOutputValue(value);

	EXPECT_FALSE(inSys.operateCalled) << "System.operate() was called";
}

template<typename T>
void checkDisconnected(const ExposedIOSystem<T>& inSys)
{
	EXPECT_FALSE(inSys.inputValueDefined()) << "input value defined";
	EXPECT_THROW(inSys.getInputValue(), std::logic_error)
			<< "input thinks it has an output";
}



#endif /* EXPOSEDIOSYSTEM_H_ */
