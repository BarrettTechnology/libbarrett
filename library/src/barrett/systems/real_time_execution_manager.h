/*
 * real_time_execution_manager.h
 *
 *  Created on: Dec 10, 2009
 *      Author: dc
 */

#ifndef REAL_TIME_EXECUTION_MANAGER_H_
#define REAL_TIME_EXECUTION_MANAGER_H_


//#include <native/task.h>

#include "../detail/ca_macro.h"
#include "./abstract/execution_manager.h"


// forward declarations from Xenomai's <native/task.h>
struct rt_task_placeholder;
typedef struct rt_task_placeholder RT_TASK;


namespace barrett {
namespace systems {


namespace detail {
extern "C" {

void rtemEntry(void* cookie);

}
}


class RealTimeExecutionManager : public ExecutionManager {
public:
	RealTimeExecutionManager();
	virtual ~RealTimeExecutionManager();

	void start();

protected:
	RT_TASK* task;

private:
	friend void detail::rtemEntry(void* cookie);

	DISALLOW_COPY_AND_ASSIGN(RealTimeExecutionManager);
};


}
}


#endif /* REAL_TIME_EXECUTION_MANAGER_H_ */
