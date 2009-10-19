/*
 * joint_torques.cpp
 *
 *  Created on: Oct 17, 2009
 *      Author: dc
 */


#include <stdexcept>
#include <iostream>
#include "../joint_torques.h"


namespace barrett {
namespace units {


// TODO(dc): this should come from some config file or something...
size_t DOF = 12;  // this is deliberately some number other than 7


bool operator== (const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument)
{
	size_t size = jt1.size();
	if (size != jt2.size()) {
		throw std::invalid_argument("(operator==(JointTorques, JointTorques)):"
				"JointTorques of different sizes are incomparable.");
	}

	for (size_t i = 0; i < size; ++i) {
		if (jt1[i] != jt2[i]) {
			return false;
		}
	}

	return true;
}

bool operator!= (const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument)
{
	return !(jt1 == jt2);
}

std::ostream& operator<< (std::ostream& os, const JointTorques& jt) {
	os << "[";

	units::JointTorques::const_iterator i = jt.begin();
	while (i != jt.end()) {
		os << *i;
		if (++i != jt.end()) {
			os << ", ";
		}
	}

	os << "]";
	return os;
}


}
}
