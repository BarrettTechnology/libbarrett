/*
 * joint_torque.h
 *
 *  Created on: Oct 16, 2009
 *      Author: dc
 */

#ifndef JOINT_TORQUES_H_
#define JOINT_TORQUES_H_


#include <stdexcept>
#include <iostream>
#include <vector>


namespace barrett {
namespace units {


// TODO(dc): this should come from some config file or something...
extern size_t DOF;


// measured in Newton * meters
// NB: protected inheritance means JointTorques doesn't have a vector's
// interface
class JointTorques : protected std::vector<double> {
public:
	JointTorques() :
		std::vector<double>(DOF) {}
	JointTorques(double jt_array[]) :  //NOLINT: ctor deliberately not marked explicit
		std::vector<double>(jt_array, jt_array + DOF) {}

//	JointTorques& operator=(const double)

	// expose portions of vector's interface that don't let users resize
	using std::vector<double>::operator=;

	using std::vector<double>::begin;
	using std::vector<double>::end;
	using std::vector<double>::rbegin;
	using std::vector<double>::rend;

	using std::vector<double>::size;

	using std::vector<double>::operator[];
	using std::vector<double>::at;
	using std::vector<double>::front;
	using std::vector<double>::back;

	using std::vector<double>::reference;
	using std::vector<double>::const_reference;
	using std::vector<double>::iterator;
	using std::vector<double>::const_iterator;
	using std::vector<double>::size_type;
	using std::vector<double>::difference_type;
	using std::vector<double>::value_type;
	using std::vector<double>::pointer;
	using std::vector<double>::const_pointer;
	using std::vector<double>::reverse_iterator;
	using std::vector<double>::const_reverse_iterator;
};


const JointTorques operator- (const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument);

bool operator== (const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument);
bool operator!= (const JointTorques& jt1, const JointTorques& jt2)
throw(std::invalid_argument);

std::ostream& operator<< (std::ostream& os, const JointTorques& jt);


}
}


#endif /* JOINT_TORQUES_H_ */
