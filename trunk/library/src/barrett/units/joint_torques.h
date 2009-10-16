/*
 * joint_torque.h
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */

#ifndef JOINT_TORQUE_H_
#define JOINT_TORQUE_H_


#include <vector>


namespace barrett {
namespace units {


// TODO(dc): this should come from some config file or something...
//size_t DOF = 7;  // this isn't runtime adjustable


class JointTorques : public std::vector<double> {
public:
	JointTorques() :
//		std::vector<double>(DOF) {}
		std::vector<double>() {}
};


}
}


#endif /* JOINT_TORQUE_H_ */
