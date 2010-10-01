/*
 * libconfig_utils.h
 *
 *  Created on: Feb 1, 2010
 *      Author: dc
 */

#ifndef LIBCONFIG_UTILS_H_
#define LIBCONFIG_UTILS_H_


#include <libconfig.h++>


namespace barrett {
namespace detail {


inline double numericToDouble(const libconfig::Setting& setting)
{
	switch (setting.getType()) {
	case libconfig::Setting::TypeInt:
		return static_cast<int>(setting);
		break;

	case libconfig::Setting::TypeInt64:
		return static_cast<long long>(setting);
		break;

	default:
		return setting;
		break;
	}

}


}
}


#endif /* LIBCONFIG_UTILS_H_ */
