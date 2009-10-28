/*
 * wam.h
 *
 *  Created on: Sep 25, 2009
 *      Author: dc
 */

#ifndef WAM_H_
#define WAM_H_


#include <map>

#include <barrett/wam/wam.h>
#include <barrett/wam/wam_local.h>

#include "./detail/ca_macro.h"
#include "./units.h"
//#include "./systems/abstract/system.h"
#include "./systems/abstract/single_io.h"


namespace barrett {


template<size_t DOF>
class Wam : public systems::SingleIO<units::JointTorques<DOF>,
									 units::JointAngles<DOF> > {
public:
	typedef units::JointTorques<DOF> jt_type;
	typedef units::JointAngles<DOF> ja_type;


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


	static std::map<struct bt_wam_local*, Wam<DOF>*> activeWams;

private:
	DISALLOW_COPY_AND_ASSIGN(Wam);
};


}


// include template definitions
#include "./detail/wam-inl.h"


#endif /* WAM_H_ */
