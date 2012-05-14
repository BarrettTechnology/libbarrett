/*
 * disable_secondary_mode_warning.h
 *
 *  Created on: Dec 14, 2010
 *      Author: dc
 */

#ifndef BARRETT_THREAD_DISABLE_SECONDARY_MODE_WARNING_H_
#define BARRETT_THREAD_DISABLE_SECONDARY_MODE_WARNING_H_


#include <barrett/detail/ca_macro.h>


namespace barrett {
namespace thread{


class DisableSecondaryModeWarning {
public:
	DisableSecondaryModeWarning();
	~DisableSecondaryModeWarning();

protected:
	bool leaveWarnSwitchOn;

private:
	DISALLOW_COPY_AND_ASSIGN(DisableSecondaryModeWarning);
};


}
}


#endif /* BARRETT_THREAD_DISABLE_SECONDARY_MODE_WARNING_H_ */
