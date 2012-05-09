/*
 * os_test.cpp
 *
 *  Created on: Mar 28, 2012
 *      Author: dc
 */

#include <stdexcept>
#include <string>
#include <cassert>

#include <boost/date_time.hpp>

#include <barrett/exception.h>
#include <barrett/os.h>


using namespace barrett;
namespace pt = boost::posix_time;

int main() {
	installExceptionHandler();


	// btsleep()
	pt::ptime startTime;

	startTime = pt::microsec_clock::local_time();
	btsleep(1.24);
	std::cout << pt::microsec_clock::local_time() - startTime << "\n";

	startTime = pt::microsec_clock::local_time();
	btsleep(1e-4);
	std::cout << pt::microsec_clock::local_time() - startTime << "\n";

	startTime = pt::microsec_clock::local_time();
	btsleep(3);
	std::cout << pt::microsec_clock::local_time() - startTime << "\n";

	// Fails if the thread is not realtime.
//	startTime = pt::microsec_clock::local_time();
//	btsleepRT(1e-3);
//	std::cout << pt::microsec_clock::local_time() - startTime << "\n";



	// logMessage()
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
