/*
 * wam.cpp
 *
 *  Created on: Sep 25, 2009
 *      Author: dc
 */


#include <stdexcept>

#include <sys/mman.h>
#include <syslog.h>

#include <barrett/wam/wam.h>
#include <barrett/wam/wam_local.h>

#include "./wam.h"


namespace barrett {
// int bt_wam_local_set_callback(struct bt_wam_local * wam,
//                               int (*callback)(struct bt_wam_local * wam));


Wam::Wam() :
	wam(NULL), wam_local(NULL)
{
	// initialize syslog
	openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

	// lock memory
	mlockall(MCL_CURRENT | MCL_FUTURE);

	// open the WAM
	bt_wam_create(&wam, "wam4");
	if (wam == NULL) {
		// TODO(dc): better exception, add throw declaration to function def
		throw std::runtime_error("Couldn't make WAM");
	}

	wam_local = bt_wam_get_local(wam);

}

Wam::~Wam()
{
	bt_wam_destroy(wam);
}


}
