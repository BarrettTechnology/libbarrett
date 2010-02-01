/*
 * real_time_execution_manager.h
 *
 *  Created on: Dec 10, 2009
 *      Author: dc
 */

#ifndef REAL_TIME_EXECUTION_MANAGER_H_
#define REAL_TIME_EXECUTION_MANAGER_H_


#include <libconfig.h>  // TODO(dc): remove this once everything uses the C++ version
#include <libconfig.h++>

#include "../detail/ca_macro.h"
#include "./abstract/execution_manager.h"


// forward declarations from Xenomai's <native/task.h>
struct rt_task_placeholder;
typedef struct rt_task_placeholder RT_TASK;

//typedef long long unsigned int RTIME;


namespace barrett {
namespace systems {


namespace detail {
extern "C" {

void rtemEntryPoint(void* cookie);

}
}


// TODO(dc): add a configuration file interface

class RealTimeExecutionManager : public ExecutionManager {
public:
	explicit RealTimeExecutionManager(double period_s, int rt_priority = 50);
	explicit RealTimeExecutionManager(const libconfig::Setting& setting);  //TODO(dc): test!
	virtual ~RealTimeExecutionManager();

	void start();
	bool isRunning();
	void stop();

protected:
	RT_TASK* task;
	int priority;
	bool running, stopRunning;

private:
	void init();

	friend void detail::rtemEntryPoint(void* cookie);

	DISALLOW_COPY_AND_ASSIGN(RealTimeExecutionManager);
};


}
}


#endif /* REAL_TIME_EXECUTION_MANAGER_H_ */
