/*
 * dynamic_dof.cpp
 *
 *  Created on: Aug 16, 2010
 *      Author: dc
 */

#include <iostream>
#include <cstdio>

#include <boost/mpl/if.hpp>
#include <Eigen/Core>
#include <barrett/units.h>


using namespace barrett;
using namespace units::typedefs;



//template<typename Units = void, bool V = false>
//class Matrix : public Eigen::Matrix<double, Eigen::Dynamic,boost::mpl::if_c<V, boost::mpl::int_<1>, boost::mpl::int_<Eigen::Dynamic> >::type::value, Eigen::RowMajorBit> {
//};

int main() {
	jp_type blah(4,1);
	blah[3] = 4;
	std::printf("%f\n", blah[3]);

//	Child c;
//	c.i = 3;

//	int i = boost::mpl::if_c<false, boost::mpl::int_<1>, boost::mpl::int_<Eigen::Dynamic> >::type::value;


//	std::printf("%d\n", i);

	return 0;
}
