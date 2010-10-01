/*
 * manual_execution_manager.h
 *
 *  Created on: Dec 9, 2009
 *      Author: dc
 */

#ifndef MANUAL_EXECUTION_MANAGER_H_
#define MANUAL_EXECUTION_MANAGER_H_


#include <libconfig.h++>

#include "../detail/ca_macro.h"
#include "./abstract/execution_manager.h"


namespace barrett {
namespace systems {


class ManualExecutionManager : public ExecutionManager {
public:
	explicit ManualExecutionManager(double period_s) :
		ExecutionManager(period_s) {}
	explicit ManualExecutionManager(const libconfig::Setting& setting) :
		ExecutionManager(setting) {}

	virtual ~ManualExecutionManager() {}

	using ExecutionManager::runExecutionCycle;
	using ExecutionManager::resetExecutionCycle;
	using ExecutionManager::update;

private:
	DISALLOW_COPY_AND_ASSIGN(ManualExecutionManager);
};


}
}


#endif /* MANUAL_EXECUTION_MANAGER_H_ */
