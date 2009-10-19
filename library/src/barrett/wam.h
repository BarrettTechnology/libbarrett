/*
 * wam.h
 *
 *  Created on: Sep 25, 2009
 *      Author: dc
 */

#ifndef WAM_H_
#define WAM_H_

#include "systems/abstract/system.h"
#include "./ca_macro.h"


namespace barrett {


class Wam: public Systems::System {
public:
	Wam();
	virtual ~Wam();

protected:
	struct bt_wam* wam;
	struct bt_wam_local* wam_local;

	virtual void operate() {}

private:
	DISALLOW_COPY_AND_ASSIGN(Wam);
};


}


#endif /* WAM_H_ */
