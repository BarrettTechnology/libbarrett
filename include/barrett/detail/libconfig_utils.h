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

/**
 * @file libconfig_utils.h
 * @date 02/01/2010
 * @author Dan Cody
 * 
 */

#ifndef BARRETT_DETAIL_LIBCONFIG_UTILS_H_
#define BARRETT_DETAIL_LIBCONFIG_UTILS_H_


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


#endif /* BARRETT_DETAIL_LIBCONFIG_UTILS_H_ */
