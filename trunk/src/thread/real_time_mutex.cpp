/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 *
 */

/*
 * @file real_time_mutex.cpp
 * @date 12/15/2009
 * @author Dan Cody
 *  
 */

#include <iostream>
#include <stdexcept>

#include <barrett/os.h>
#include <barrett/thread/real_time_mutex.h>


#ifdef BARRETT_XENOMAI
	#include "real_time_mutex_impl-xenomai.cpp"
#else
	#include <boost/thread/recursive_mutex.hpp>

	namespace barrett {
	namespace thread {
	namespace detail {
		class mutex_impl : public boost::recursive_mutex {};
	}
	}
	}
#endif


namespace barrett {
namespace thread {


RealTimeMutex::RealTimeMutex() :
	mutex(NULL), lockCount(0)
{
	mutex = new detail::mutex_impl;
}

RealTimeMutex::~RealTimeMutex()
{
	delete mutex;
	mutex = NULL;
}

void RealTimeMutex::lock()
{
	mutex->lock();
	++lockCount;
}

bool RealTimeMutex::try_lock()
{
	if (mutex->try_lock()) {
		++lockCount;
		return true;
	} else {
		return false;
	}
}

void RealTimeMutex::unlock()
{
	--lockCount;
	mutex->unlock();
}

int RealTimeMutex::fullUnlock()
{
	int lc = lockCount;
	if (lc <= 0) {
		(logMessage("thread::RealTimeMutex::%s Bad lockCount value.  lockCount = %d") %__func__ %lc).raise<std::logic_error>();
	}

	while (lockCount > 1) {
		unlock();
	}
	unlock();

	return lc;
}

void RealTimeMutex::relock(int lc)
{
	lock();
	while (lockCount != lc) {
		lock();
	}
}


}
}
