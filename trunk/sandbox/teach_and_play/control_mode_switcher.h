/*
 * control_mode_switcher.h
 *
 *  Created on: Jul 24, 2012
 *      Author: dc
 */

#ifndef CONTROL_MODE_SWITCHER_H_
#define CONTROL_MODE_SWITCHER_H_


#include <iostream>
#include <cassert>

#include <libconfig.h++>

#include <barrett/os.h>
#include <barrett/units.h>
#include <barrett/math/utils.h>
#include <barrett/products/product_manager.h>
#include <barrett/products/puck.h>
#include <barrett/systems/helpers.h>
#include <barrett/systems/wam.h>
#include <barrett/systems/gain.h>


template<size_t DOF>
class ControlModeSwitcher {
public:
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	enum ControlMode { CURRENT, VOLTAGE };

	ControlModeSwitcher(barrett::ProductManager& pm_, barrett::systems::Wam<DOF>& wam_, const libconfig::Setting& setting_) :
		pm(pm_), wam(wam_), setting(setting_), mode(VOLTAGE), cGain(setting["current_gain"]), vGain(setting["voltage_gain"]), torqueGainSys(vGain)
	{
		for (size_t i = 0; i < DOF; ++i) {
			assert(wam.getLowLevelWam().getPucks()[i]->getVers() >= 200);
		}

		barrett::systems::connect(wam.jtSum.output, torqueGainSys.input);
		barrett::systems::reconnect(torqueGainSys.output, wam.llww.input);
		currentControl();
	}
	~ControlModeSwitcher() {
		currentControl();  // Revert back to current control.
		barrett::systems::reconnect(wam.jtSum.output, wam.llww.input);
	}

	enum ControlMode getMode() const { return mode; }
	double getCurrentGain() const { return cGain; }
	double getVoltageGain() const { return vGain; }

	void currentControl() {
		if (getMode() == CURRENT) {
			return;
		}

		for (size_t i = 0; i < DOF; ++i) {
			wam.getLowLevelWam().getPucks()[i]->resetProperty(barrett::Puck::IKCOR);
			wam.getLowLevelWam().getPucks()[i]->resetProperty(barrett::Puck::IKP);
			wam.getLowLevelWam().getPucks()[i]->resetProperty(barrett::Puck::IKI);
		}

		const libconfig::Setting& wamSetting = pm.getConfig().lookup(pm.getWamDefaultConfigPath());

		wam.jpController.setFromConfig(wamSetting["joint_position_control"]);
		wam.jpController.resetIntegrator();

		wam.tpController.setFromConfig(wamSetting["tool_position_control"]);
		wam.tpController.resetIntegrator();
		wam.tpoTpController.setFromConfig(wamSetting["tool_position_control"]);
		wam.tpoTpController.resetIntegrator();

		wam.toController.setFromConfig(wamSetting["tool_orientation_control"]);
		wam.tpoToController.setFromConfig(wamSetting["tool_orientation_control"]);

		torqueGainSys.setGain(cGain);

		mode = CURRENT;
	}
	void voltageControl() {
		if (getMode() == VOLTAGE) {
			return;
		}

		wam.getLowLevelWam().getPuckGroup().setProperty(barrett::Puck::IKI, 0);
		wam.getLowLevelWam().getPuckGroup().setProperty(barrett::Puck::IKP, 8000);
		wam.getLowLevelWam().getPuckGroup().setProperty(barrett::Puck::IKCOR, 0);

		wam.jpController.setFromConfig(setting["voltage_joint_position_control"]);
		wam.jpController.resetIntegrator();

		wam.tpController.setFromConfig(setting["voltage_tool_position_control"]);
		wam.tpController.resetIntegrator();
		wam.tpoTpController.setFromConfig(setting["voltage_tool_position_control"]);
		wam.tpoTpController.resetIntegrator();

		wam.toController.setFromConfig(setting["voltage_tool_orientation_control"]);
		wam.tpoToController.setFromConfig(setting["voltage_tool_orientation_control"]);

		torqueGainSys.setGain(vGain);

		mode = VOLTAGE;
	}


	static const double MAX_SCALE = 4.0;
	static const double MIN_SCALE = 0.25;

	void calculateTorqueGain() {
		libconfig::Config config;
		config.readFile("/etc/barrett/calibration.conf");
		libconfig::Setting& setting = config.lookup("gravitycal")[pm.getWamDefaultConfigPath()];

		int scaleCount = 0;
		double scaleSum = 0.0;
		double maxScale = MIN_SCALE;
		double minScale = MAX_SCALE;

		for (int i = 0; i < setting.getLength(); ++i) {
			wam.moveTo(jp_type(setting[i]));
			barrett::btsleep(1.0);

			jt_type gravity = wam.jtSum.getInput(barrett::systems::Wam<DOF>::GRAVITY_INPUT).getValue();
			jt_type supporting = wam.llww.input.getValue();
			double scale = (gravity.dot(supporting) / gravity.norm()) / gravity.norm();
			jt_type error = supporting - scale*gravity;

			printf("Calibration Pose - %d of %d", i+1, setting.getLength());

			bool good = true;
			if (scale < MIN_SCALE  ||  scale > MAX_SCALE) {
				printf("Scale is out of range.\n");
				good = false;
			}
			if (error.norm() > 0.4 * supporting.norm()) {
				//printf("Error is too big.\n");
				good = false;
			}

			if (good) {
				scaleCount++;
				scaleSum += scale;
				maxScale = barrett::math::max(maxScale, scale);
				minScale = barrett::math::min(minScale, scale);
			}

			printf("\n");
		}

		assert(scaleCount >= 3);
		double meanScale = scaleSum/scaleCount;
		printf("SCALE: %f (+%f, -%f)\n", meanScale, maxScale - meanScale, meanScale - minScale);

		switch (getMode()) {
		case CURRENT:
			cGain = meanScale;
			torqueGainSys.setGain(cGain);
			break;
		case VOLTAGE:
			vGain = meanScale;
			torqueGainSys.setGain(vGain);
			break;
		default:
			assert(false);
			break;
		}
	}

protected:
	barrett::ProductManager& pm;
	barrett::systems::Wam<DOF>& wam;
	const libconfig::Setting& setting;

	enum ControlMode mode;
	double cGain, vGain;
	barrett::systems::Gain<jt_type, double> torqueGainSys;
};


#endif /* CONTROL_MODE_SWITCHER_H_ */
