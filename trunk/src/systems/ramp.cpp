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
/*
 * @file ramp.cpp
 * @date 05/18/2011
 * @author Dan Cody
 *  
 */

#include <barrett/systems/ramp.h>


namespace barrett {
namespace systems {


Ramp::Ramp(ExecutionManager* em, double slope, const std::string& sysName) :
	System(sysName), SingleOutput<double>(this),
	T_s(0.0), gain(slope), finalGain(0.0), curGain(0.0), curvature(0.0), y(0.0)
{
	// Update every execution cycle so the ramp stays current even if the
	// data isn't used for a time.
	if (em != NULL) {
		em->startManaging(*this);
	}

	getSamplePeriodFromEM();
}
Ramp::~Ramp() {
	mandatoryCleanUp();
}


void Ramp::onExecutionManagerChanged() {
	System::onExecutionManagerChanged();  // First, call super
	getSamplePeriodFromEM();
}

void Ramp::operate() {
	if (isRunning()) {
		if (curvature != 0.0) {
			curGain += T_s * curvature;
			if ((curvature > 0.0  &&  curGain >= finalGain)  ||
				(curvature < 0.0  &&  curGain <= finalGain)) {
				curvature = 0.0;
				curGain = finalGain;
			}
		}

		y += T_s * curGain;
	}

	outputValue->setData(&y);
}

void Ramp::getSamplePeriodFromEM()
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
