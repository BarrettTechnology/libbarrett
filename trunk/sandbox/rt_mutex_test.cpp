/*
 * rt_mutex_test.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: dc
 */

#include <iostream>
#include <cassert>
#include <vector>

#include <sys/mman.h>
#include <native/task.h>

#include <barrett/thread/real_time_mutex.h>


using namespace barrett;


int test(thread::RealTimeMutex& m) {
	int mode;


	m.lock();
	rt_task_set_mode(0,0, &mode);
	assert((mode & T_PRIMARY)  &&  (mode & T_WARNSW));

	m.lock();
	rt_task_set_mode(0,0, &mode);
	assert((mode & T_PRIMARY)  &&  (mode & T_WARNSW));

	m.lock();
	rt_task_set_mode(0,0, &mode);
	assert((mode & T_PRIMARY)  &&  (mode & T_WARNSW));


	m.unlock();
	rt_task_set_mode(0,0, &mode);
	assert((mode & T_PRIMARY)  &&  (mode & T_WARNSW));

	m.unlock();
	rt_task_set_mode(0,0, &mode);
	assert((mode & T_PRIMARY)  &&  (mode & T_WARNSW));

	m.unlock();
	rt_task_set_mode(0,0, &mode);


	printf("%d %d\n", mode & T_PRIMARY, mode & T_WARNSW);
	return mode;
}

int main() {
/*
	thread::RealTimeMutex m;

	std::vector<int> modes(17, 0);
	size_t i = 0;
	size_t j = 0;

//	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_shadow(new RT_TASK, NULL, 10, 0);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(0,T_WARNSW, NULL);

//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);
//	rt_task_set_mode(0,0, &modes[i++]);
//	usleep(100); printf("%d\n", j++);

	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(0,T_PRIMARY, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(T_PRIMARY,0, &modes[i++]);
	usleep(100); printf("%d\n", j++);
	rt_task_set_mode(0,0, &modes[i++]);
	usleep(100); printf("%d\n", j++);


	for (i = 0; i < modes.size(); ++i) {
		printf("%d %d\n", modes[i] & T_PRIMARY, modes[i] & T_WARNSW);
	}
*/

	int mode;
	thread::RealTimeMutex m;

	// Not an RT task yet.
	mode = test(m);
	assert((~mode & T_PRIMARY)  &&  (~mode & T_WARNSW));

	rt_task_set_mode(0, T_WARNSW, NULL);
	mode = test(m);
	assert((~mode & T_PRIMARY)  &&  (mode & T_WARNSW));

	rt_task_set_mode(0, T_PRIMARY, NULL);
	mode = test(m);
	assert((mode & T_PRIMARY)  &&  (mode & T_WARNSW));

	rt_task_set_mode(T_WARNSW, 0, NULL);
	mode = test(m);
	assert((mode & T_PRIMARY)  &&  (~mode & T_WARNSW));

	return 0;
}
