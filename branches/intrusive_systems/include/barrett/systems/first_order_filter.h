/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * first_order_filter.h
 *
 *  Created on: Nov 18, 2009
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_FIRST_ORDER_FILTER_H_
#define BARRETT_SYSTEMS_FIRST_ORDER_FILTER_H_


#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/first_order_filter.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


template<typename T>
class FirstOrderFilter : public SingleIO<T, T>, protected math::FirstOrderFilter<T> {
public:
	explicit FirstOrderFilter(const std::string& sysName = "FirstOrderFilter");
	explicit FirstOrderFilter(const libconfig::Setting& setting, const std::string& sysName = "FirstOrderFilter");
	virtual ~FirstOrderFilter() { this->mandatoryCleanUp(); }

	using math::FirstOrderFilter<T>::setFromConfig;
	using math::FirstOrderFilter<T>::setLowPass;
	using math::FirstOrderFilter<T>::setHighPass;
	using math::FirstOrderFilter<T>::setZPK;
	using math::FirstOrderFilter<T>::setIntegrator;
	using math::FirstOrderFilter<T>::setParameters;

protected:
	virtual void operate();

	virtual void onExecutionManagerChanged() {
		SingleIO<T, T>::onExecutionManagerChanged();  // First, call super
		getSamplePeriodFromEM();
	}
	void getSamplePeriodFromEM();

private:
	DISALLOW_COPY_AND_ASSIGN(FirstOrderFilter);
};


}
}


// include template definitions
#include <barrett/systems/detail/first_order_filter-inl.h>


#endif /* BARRETT_SYSTEMS_FIRST_ORDER_FILTER_H_ */
