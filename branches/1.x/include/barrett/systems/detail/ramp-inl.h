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
 * ramp-inl.h
 *
 *  Created on: May 18, 2011
 *      Author: dc
 */


namespace barrett {
namespace systems {


inline bool Ramp::isRunning() {
	return curGain != 0.0  ||  finalGain != 0.0;
}

inline void Ramp::start() {
	// curGain is written and read in operate(), so it needs to be locked.
	BARRETT_SCOPED_LOCK(getEmMutex());
	curGain = finalGain = gain;
}
inline void Ramp::stop() {
	// curGain is written and read in operate(), so it needs to be locked.
	BARRETT_SCOPED_LOCK(getEmMutex());
	curGain = finalGain = 0.0;
}
inline void Ramp::setSlope(double slope) {
	gain = slope;
	if (isRunning()) {
		start();
	}
}

inline void Ramp::reset() {
	setOutput(0.0);
}
inline void Ramp::setOutput(double newOutput) {
	// y is written and read in operate(), so it needs to be locked.
	BARRETT_SCOPED_LOCK(getEmMutex());
	y = newOutput;
}

inline void Ramp::smoothStart(double transitionDuration) {
	finalGain = gain;
	setCurvature(transitionDuration);
}
inline void Ramp::smoothStop(double transitionDuration) {
	finalGain = 0.0;
	setCurvature(transitionDuration);
}
inline void Ramp::smoothSetSlope(double slope, double transitionDuration) {
	gain = slope;
	if (isRunning()) {
		smoothStart(transitionDuration);
	}
}


inline void Ramp::setCurvature(double transitionDuration) {
	assert(transitionDuration > 0.0);

	// curvature is written and read in operate(), so it needs to be locked.
	BARRETT_SCOPED_LOCK(getEmMutex());
	curvature = (finalGain - curGain) / transitionDuration;
}


}
}
