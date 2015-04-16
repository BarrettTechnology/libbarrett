/*
	Copyright 2014, 2015 Barrett Technology <support@barrett.com>

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
 * modXYZ.h
 *
 *  Created on: Mar 27, 2015
 *      Author: hm
 */
#ifndef BARRETT_SYSTEMS_MODXYZ_H_
#define BARRETT_SYSTEMS_MODXYZ_H_

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>

namespace barrett {
namespace systems {

template <typename T>
class modXYZ : public systems::SingleIO<T, T> {
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
public:
	modXYZ(const std::string& sysName = "modXYZ") : systems::SingleIO<T, T>(sysName), x(false), y(false), z(false), off_x(0.0), off_y(0.0), off_z(0.0) {}

	void negX(){
	  x = true;
	}
	void negY(){
	  y = true;
	}
	void negZ(){
	  z = true;
	}
  void xOffset(double val){
    off_x = val;
  }
  void yOffset(double val){
    off_y = val;
  }
  void zOffset(double val){
    off_z = val;
  }
protected:
	bool x, y, z;
	double off_x, off_y, off_z;
	T xyz, mod_xyz;

	virtual void operate() {
		xyz = this->input.getValue();
		mod_xyz = xyz;
		if(x)
			mod_xyz[0] = -xyz[0];
		if(y)
			mod_xyz[1] = -xyz[1];
		if(z)
			mod_xyz[2] = -xyz[2];
		mod_xyz[0] = mod_xyz[0] + off_x;
		mod_xyz[1] = mod_xyz[1] + off_y;
		mod_xyz[2] = mod_xyz[2] + off_z;
		this->outputValue->setData(&mod_xyz);	
	}
private:
 DISALLOW_COPY_AND_ASSIGN(modXYZ);
};

}
}

#endif /* BARRETT_SYSTEMS_MODXYZ_H_ */

