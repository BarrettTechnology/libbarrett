/*
 * joint_space.h
 *
 *  Created on: Oct 20, 2009
 *      Author: dc
 */

#ifndef JOINT_SPACE_H_
#define JOINT_SPACE_H_


#include "../units.h"


namespace barrett {
namespace coordinate_systems {


class joint_space {
public:
	typedef units::JointAngles position;
	typedef units::JointTorques effort;
};


}
}


#endif /* JOINT_SPACE_H_ */
