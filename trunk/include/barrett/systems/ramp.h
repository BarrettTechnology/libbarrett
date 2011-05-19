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
 * ramp.h
 *
 *  Created on: Jan 5, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_RAMP_H_
#define BARRETT_SYSTEMS_RAMP_H_


#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>


namespace barrett {
namespace systems {


// TODO(dc): add a configuration file interface


class Ramp : public System, public SingleOutput<double> {
public:
	explicit Ramp(ExecutionManager* em, double slope = 1.0,
			const std::string& sysName = "Ramp");
	virtual ~Ramp();

	bool isRunning();

	void start();
	void stop();
	void setSlope(double slope);

	void reset();
	void setOutput(double newOutput);

	void smoothStart(double transitionDuration);
	void smoothStop(double transitionDuration);
	void smoothSetSlope(double slope, double transitionDuration);

protected:
	virtual void onExecutionManagerChanged();
	void getSamplePeriodFromEM();

	virtual void operate();

	void setCurvature(double transitionDuration);


	double T_s;
	double gain, finalGain, curGain, curvature;
	double y;

private:
	DISALLOW_COPY_AND_ASSIGN(Ramp);
};


}
}


// include template definitions
#include <barrett/systems/detail/ramp-inl.h>


#endif /* BARRETT_SYSTEMS_RAMP_H_ */
