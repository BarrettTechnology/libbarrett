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


void throwIfDifferentSized(const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument)
{
	if (jt1.size() != jt2.size()) {
		throw std::invalid_argument("(barrett::units::JointTorques)):"
				"JointTorques of different sizes are incomparable.");
	}
}


const JointTorques operator- (const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument)
{
	throwIfDifferentSized(jt1, jt2);

	JointTorques result;
	for (size_t i = 0; i < jt1.size(); ++i) {
		result[i] = jt1[i] - jt2[i];
	}

	return result;
}

bool operator== (const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument)
{
	throwIfDifferentSized(jt1, jt2);

	for (size_t i = 0; i < jt1.size(); ++i) {
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
