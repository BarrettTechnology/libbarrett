/*
 * stl_utils.cpp
 *
 *  Created on: Nov 5, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>


namespace barrett {
namespace detail {


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


}
}
