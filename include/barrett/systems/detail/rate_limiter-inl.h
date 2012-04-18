/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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
 * rate_limiter-inl.h
 *
 *  Created on: Apr 12, 2012
 *      Author: dc
 */

#include <barrett/systems/rate_limiter.h>


namespace barrett {
namespace systems {

template<typename T, typename MathTraits>
void RateLimiter<T,MathTraits>::operate()
{
	data = this->input.getValue();
	this->outputValue->setData(&data);
}

// TODO(dc): anyway to remove the code duplication with PIDController?
template<typename T, typename MathTraits>
void RateLimiter<T,MathTraits>::getSamplePeriodFromEM()
{
	if (this->hasExecutionManager()) {
		assert(this->getExecutionManager()->getPeriod() > 0.0);
		T_s = this->getExecutionManager()->getPeriod();
	} else {
		T_s = 0.0;
	}
}


}
}
