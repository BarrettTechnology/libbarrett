/*
 * wam.h
 *
 *  Created on: Sep 25, 2009
 *      Author: dc
 */

#ifndef WAM_H_
#define WAM_H_


#include <map>

#include "./detail/ca_macro.h"
#include "./units.h"
//#include "./systems/abstract/system.h"
#include "./systems/abstract/single_io.h"


namespace barrett {


class Wam : public systems::SingleIO<units::JointTorques,
									 units::JointAngles> {
public:
	int operateCount;

	Wam();
	virtual ~Wam();

	void gravityCompensate(bool compensate = true);
	void moveHome();
	void idle();

	static int handleCallback(struct bt_wam_local* wamLocal);

protected:
	struct bt_wam* wam;
	struct bt_wam_local* wamLocal;

	virtual void readSensors();
	virtual void operate();


	static std::map<struct bt_wam_local*, Wam*> activeWams;

private:
	DISALLOW_COPY_AND_ASSIGN(Wam);
};


}


#endif /* WAM_H_ */
