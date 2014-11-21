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
 * @file real_time_mutex_impl-xenomai.cpp
 * @date 06/20/2013
 * @author Dan Cody
 * 
 */

#include <native/task.h>
#include <native/mutex.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/os.h>


namespace barrett {
namespace thread {
namespace detail {


class mutex_impl {
public:
	mutex_impl() :
		lockCount(0), leaveWarnSwitchOn(false)
	{
		int ret = rt_mutex_create(&m, NULL);
		if (ret != 0) {
			(logMessage("thread::detail::mutex_impl::%s: Could not create RT_MUTEX: (%d) %s")
					% __func__ % -ret % strerror(-ret)).raise<std::logic_error>();
		}
	}

	~mutex_impl() {
		int ret = rt_mutex_delete(&m);

		if (ret != 0) {
			// Don't throw exceptions from a dtor!
			logMessage("thread::detail::mutex_impl::%s: Could not delete RT_MUTEX: (%d) %s")
					% __func__ % -ret % strerror(-ret);
		}
	}

	void lock() {
		int ret = acquireWrapper(true);
		if (ret != 0) {
			logMessage("thread::detail::mutex_impl::lock(): %s returned %d")
					% __func__ % ret;
			throw boost::thread_resource_error(ret);
		}
	}

	bool try_lock() {
		return acquireWrapper(false) == 0;
	}

	void unlock() {
		--lockCount;
		bool changeMode = lockCount == 0 && !leaveWarnSwitchOn;

		int ret = rt_mutex_release(&m);
		if (ret != 0) {
			(logMessage("thread::detail::mutex_impl::%s:  Could not release RT_MUTEX: (%d) %s")
					% __func__ % -ret % strerror(-ret)).raise<std::logic_error>();
		}

		if (changeMode) {
			ret = rt_task_set_mode(T_WARNSW, 0, NULL);
			if (ret != 0) {
				throw std::runtime_error("thread::detail::mutex_impl::unlock(): Could not clear T_WARNSW mode.");
			}
		}
	}

protected:
	int acquireWrapper(bool blocking)
	{
		const RTIME timeout = blocking ? TM_INFINITE : TM_NONBLOCK;
		int ret;

		ret = rt_mutex_acquire(&m, timeout);
		if (ret == -EPERM) {
			// become real-time, then try again

			// Allocate a new RT_TASK struct, and then forget the pointer. This
			// leak allows us to avoid ownership issues for the RT_TASK, which
			// shouldn't necessarily be deleted when the mutex is released, or when
			// the mutex is deleted, etc.. It is a small overhead that happens (at
			// most) once per thread. If needed, we can always get the pointer back
			// by calling rt_task_self().
			rt_task_shadow(new RT_TASK, NULL, 10, 0);

			ret = rt_mutex_acquire(&m, timeout);
		}
		if (ret != 0) {
			return ret;
		}

		if (lockCount == 0) {
			int oldMode;
			ret = rt_task_set_mode(0, T_WARNSW, &oldMode);
			if (ret != 0) {
				throw std::runtime_error("thread::detail::mutex_impl::acquireWrapper(): Could not set T_WARNSW mode.");
			}
			leaveWarnSwitchOn = oldMode & T_WARNSW;
		}
		++lockCount;

		return ret;
	}

	RT_MUTEX m;
	int lockCount;
	bool leaveWarnSwitchOn;

private:
	DISALLOW_COPY_AND_ASSIGN(mutex_impl);
};


}
}
}
