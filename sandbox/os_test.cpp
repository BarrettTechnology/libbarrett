/*
 * os_test.cpp
 *
 *  Created on: Mar 28, 2012
 *      Author: dc
 */

#include <stdexcept>
#include <string>
#include <cassert>

#include <barrett/exception.h>
#include <barrett/os.h>


using namespace barrett;


int main() {
	installExceptionHandler();


	logMessage("literal string\n");

	const char cStr[] = "char array";
	logMessage(cStr, true);

	bool exception = false;
	try {
		logMessage(std::string("STL string")).raise<std::logic_error>();
	} catch (std::logic_error e) {
		exception = true;
	}
	assert(exception);

	logMessage("%d string formatted with %s", true) % 1 % "boost";
	std::cerr << "omg\n";

	(logMessage("%d string formatted with %s", true) % 2 % "boost").raise<std::runtime_error>();

	return 0;
}
