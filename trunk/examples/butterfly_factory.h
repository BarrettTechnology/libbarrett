#ifndef BUTTERFLY_FACTORY_H_
#define BUTTERFLY_FACTORY_H_


#include <barrett/systems/abstract/system.h>


class Butterfly {
public:
	int ID;
};


class ButterflyFactory : public barrett::systems::System {
// IO
public:		Input<int> idInput;
public:		Output<Butterfly> butterflyOutput;
protected:	Output<Butterfly>::Value* butterflyOutputValue;

public:
	ButterflyFactory() :
		idInput(this), butterflyOutput(&butterflyOutputValue) {}

protected:
	virtual void operate() {
		int id = idInput.getValue();

		Butterfly butterfly;
		butterfly.ID = id;

		butterflyOutputValue->setValue(butterfly);
	}
};


#endif /* BUTTERFLY_FACTORY_H_ */
