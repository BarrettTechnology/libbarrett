/*
 * HAPI_haptics.cpp
 *
 *  Created on: Apr 13, 2010
 *      Author: dc
 */

#include <iostream>
#include <string>

#include <HAPI/HAPIHapticsDevice.h>
#include <HAPI/HapticPrimitive.h>
#include <HAPI/FrictionSurface.h>
#include <HAPI/GodObjectRenderer.h>

#include <barrett/systems.h>
#include <barrett/wam.h>


using namespace HAPI;
using namespace barrett;
using systems::connect;

const size_t DOF = 4;
const double T_s = 0.002;

typedef Wam<DOF>::jt_type jt_type;
typedef Wam<DOF>::jp_type jp_type;
typedef Wam<DOF>::jv_type jv_type;


void waitForEnter() {
	static std::string line;
	std::getline(std::cin, line);
}


class WamHapticsDevice : public HAPIHapticsDevice {
protected:
	systems::RealTimeExecutionManager* rtem;
	systems::ExposedOutput<units::CartesianForce::type>* tf;
	systems::ToolForceToJointTorques<DOF>* tf2jt;
	Wam<DOF>* wam;

	virtual bool initHapticsDevice(int _thread_frequency = 500) {
		libconfig::Config config;
		config.readFile("/etc/wam/wam4-new.config");

		rtem = new systems::RealTimeExecutionManager(T_s);
		systems::System::defaultExecutionManager = rtem;

		tf = new systems::ExposedOutput<units::CartesianForce::type>;
		tf2jt = new systems::ToolForceToJointTorques<DOF>;
		wam = new Wam<DOF>(config.lookup("wam"));

		connect(tf->output, tf2jt->input);
		connect(wam->kinematicsBase.kinOutput, tf2jt->kinInput);
		connect(tf2jt->output, wam->input);

		tf->setValue(units::CartesianForce::type(0.0));

		rtem->start();

		std::cout << "Press [Enter] to compensate for gravity.\n";
		waitForEnter();

		wam->gravityCompensate();

		return true;
	}

	virtual bool releaseHapticsDevice() {
		std::cout << "Shift-idle, then press [Enter].\n";
		waitForEnter();

		rtem->stop();

		delete wam;
		delete tf2jt;
		delete tf;
		delete rtem;

		return true;
	}

	virtual void updateDeviceValues(DeviceValues &dv, HAPITime dt) {
		units::CartesianForce::type cf(wam->tpController.feedbackInput.getValue());
		dv.position[0] = cf[0];
		dv.position[1] = cf[1];
		dv.position[2] = cf[2];
	}

	virtual void sendOutput(DeviceOutput &dv, HAPITime dt) {
		units::CartesianForce::type cf;
		cf[0] = dv.force[0];
		cf[1] = dv.force[1];
		cf[2] = dv.force[2];
		tf->setValue(cf);
	}
};


int main() {
	  // Get a connected device.
	  WamHapticsDevice hd;

	  // The haptics renderer to use.
	  hd.setHapticsRenderer( new GodObjectRenderer() );

	  // Init the device.
	  if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
	    cerr << hd.getLastErrorMsg() << endl;
	    return 0;
	  }
	  // Enable the device
	  hd.enableDevice();

	  // Creating a default surface.
	  HAPISurfaceObject * my_surface = new FrictionSurface(1);
	  // Creating a sphere with radius 0.05 and center in (0, 0, 0). Units in m.
	  HapticPrimitive *my_haptic_sphere =  new HapticPrimitive(
	    new Collision::Sphere( Vec3( 0, 0, 1 ), 0.5 ),
	    my_surface, Collision::FRONT );

	  // Add the shape to be rendered on the device.
	  hd.addShape( my_haptic_sphere );

	  // Transfer objects (shapes) to the haptics loop.
	  hd.transferObjects();

	  // Wait for user input.
	  cout << "Provided you have a haptics device connected you should ";
	  cout << "find a sphere in the middle of the device workspace."
	       <<endl << endl;
	  string temp_string;
	  cerr << "Press ENTER to exit" << endl;
	  getline( cin, temp_string );

	  // Disable device.
	  hd.disableDevice();

	  // Release device.
	  hd.releaseDevice();
	return 0;
}
