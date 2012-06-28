/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * low_level_wam-inl.h
 *
 *  Created on: Nov 2, 2010
 *      Author: cd
 *      Author: dc
 */


#include <stdexcept>
#include <vector>
#include <limits>
#include <cmath>
#include <cassert>

#include <native/timer.h>

#include <boost/static_assert.hpp>
#include <Eigen/LU>
#include <libconfig.h++>

#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/puck_group.h>
#include <barrett/products/abstract/multi_puck_product.h>

#include <barrett/products/low_level_wam.h>


namespace barrett {


template<size_t DOF>
const enum Puck::Property LowLevelWam<DOF>::props[] = { Puck::P, Puck::T };


template<size_t DOF>
LowLevelWam<DOF>::LowLevelWam(const std::vector<Puck*>& _pucks, SafetyModule* _safetyModule, const libconfig::Setting& setting, std::vector<int> torqueGroupIds) :
	MultiPuckProduct(DOF, _pucks, PuckGroup::BGRP_WAM, props, sizeof(props)/sizeof(props[0]), "LowLevelWam::LowLevelWam()"),
	safetyModule(_safetyModule), torqueGroups(),
	home(setting["home"]), j2mp(setting["j2mp"]),
	noJointEncoders(true), positionSensor(PS_MOTOR_ENCODER),
	lastUpdate(0), torquePropId(group.getPropertyId(Puck::T))
{
	logMessage("  Config setting: %s => \"%s\"") % setting.getSourceFile() % setting.getPath();


	group.setProperty(Puck::MODE, MotorPuck::MODE_IDLE);  // Make sure the Pucks are IDLE


	// Zero-compensation?
	bool zeroCompensation = setting.exists("zeroangle");
	if (!zeroCompensation) {
		logMessage("  Missing \"zeroangle\" vector: no zero-compensation");
	}


	// Compute motor/joint transforms
	Eigen::LU<typename sqm_type::Base> lu(j2mp);
	if (!lu.isInvertible()) {
		(logMessage("LowLevelWam::%s(): j2mp matrix is not invertible.")
				% __func__).template raise<std::runtime_error>();
	}
	lu.computeInverse(&m2jp);
	j2mt = m2jp.transpose();


	// Setup torque groups
	// If no IDs are provided, use the defaults
	if (torqueGroupIds.size() == 0) {
		torqueGroupIds.push_back(PuckGroup::BGRP_LOWER_WAM);
		torqueGroupIds.push_back(PuckGroup::BGRP_UPPER_WAM);
	}
	size_t numTorqueGroups = ceil(static_cast<double>(DOF)/MotorPuck::PUCKS_PER_TORQUE_GROUP);
	if (numTorqueGroups > torqueGroupIds.size()) {
		(logMessage("LowLevelWam::%s(): Too few torque group IDs. "
				"Need %d torque groups, only %d IDs provided.")
				% __func__ % numTorqueGroups % torqueGroupIds.size())
				.template raise<std::logic_error>();
	}

	size_t i = 0;
	for (size_t g = 0; g < numTorqueGroups; ++g) {
		std::vector<Puck*> tgPucks;
		while (tgPucks.size() < 4  &&  i < DOF) {
			tgPucks.push_back(pucks[i]);
			++i;
		}
		torqueGroups.push_back(new PuckGroup(torqueGroupIds[g], tgPucks));
	}


	// Compute puck/joint transforms
	// (See DC notebook 1p25)
	v_type cpr;
	for (size_t i = 0; i < DOF; ++i) {
		cpr[i] = motorPucks[i].getCountsPerRad();
	}
	j2pp = cpr.asDiagonal() * j2mp;

	v_type rpc;
	for (size_t i = 0; i < DOF; ++i) {
		rpc[i] = motorPucks[i].getRadsPerCount();
	}
	p2jp = m2jp * rpc.asDiagonal();

	v_type ipnm;
	for (size_t i = 0; i < DOF; ++i) {
		ipnm[i] = motorPucks[i].getIpnm();
	}
	j2pt = ipnm.asDiagonal() * j2mt;


	// Zero the WAM?
	if (safetyModule == NULL) {
		logMessage("  No safetyModule: WAM might not be zeroed");
	} else if (safetyModule->wamIsZeroed()) {
		logMessage("  WAM was already zeroed");
	} else if (zeroCompensation) {
		v_type zeroAngle(setting["zeroangle"]);

		v_type currentAngle;
		for (size_t i = 0; i < DOF; ++i) {
			currentAngle[i] = motorPucks[i].counts2rad(pucks[i]->getProperty(Puck::MECH));
		}

		v_type errorAngle = (j2mp*home + zeroAngle) - currentAngle;
		for (size_t i = 0; i < DOF; ++i) {
			while (errorAngle[i] > M_PI) {
				errorAngle[i] -= 2*M_PI;
			}
			while (errorAngle[i] < -M_PI) {
				errorAngle[i] += 2*M_PI;
			}
		}

		// Check for exclusions
		for (size_t i = 0; i < DOF; ++i) {
			// If VERS < 118, then nothing useful is exposed on MECH; don't compensate
			if (pucks[i]->getVers() < 118) {
				logMessage("  No zero-compensation for Puck %d: old firmware") % pucks[i]->getId();
				errorAngle[i] = 0;
				continue;
			}

			// If not ROLE & 256, then it's not an absolute encoder; don't compensate
			if ( !(pucks[i]->hasOption(Puck::RO_MagEncOnSerial)) ) {
				logMessage("  No zero-compensation for Puck %d: no absolute encoder") % pucks[i]->getId();
				errorAngle[i] = 0;
				continue;
			}

			// If the calibration data is out of range, don't compensate
			if (zeroAngle[i] > 2*M_PI  ||  zeroAngle[i] < 0) {
				logMessage("  No zero-compensation for Puck %d: bad calibration data") % pucks[i]->getId();
				errorAngle[i] = 0;
				continue;
			}
		}

		definePosition(home - m2jp*errorAngle);
		logMessage("  WAM zeroed with zero-compensation");
	} else {
		definePosition(home);
		logMessage("  WAM zeroed without zero-compensation");
	}


	// Joint encoders
	setPositionSensor(PS_MOTOR_ENCODER);  // Use motor encoders by default
	int numJe = 0;
	int numInitializedJe = 0;

	for (size_t i = 0; i < DOF; ++i) {
		if (pucks[i]->hasOption(Puck::RO_OpticalEncOnEnc)) {
			++numJe;
			if (motorPucks[i].foundIndexPulse()) {
				++numInitializedJe;
			}
		}
	}

	// If there are joint encoders, look up counts per revolution from the config file
	if (numJe != 0) {
		noJointEncoders = false;
		logMessage("  Found %d joint encoders (%d are initialized)") % numJe % numInitializedJe;

		v_type jointEncoderCpr(setting["joint_encoder_counts"]);
		for (size_t i = 0; i < DOF; ++i) {
			if (jointEncoderCpr[i] == 0.0) {
				jointEncoder2jp[i] = 0.0;
			} else {
				jointEncoder2jp[i] = 2*M_PI / jointEncoderCpr[i];
			}
		}
	}


	// Prevent velocity faults on startup due to WAM motion while no program was running.
	if (safetyModule != NULL) {
		safetyModule->ignoreNextVelocityFault();
	}


	// Get good initial values for jp_1 and lastUpdate
	update();
	jv_best.setZero();
}

template<size_t DOF>
LowLevelWam<DOF>::~LowLevelWam()
{
	detail::purge(torqueGroups);
}


template<size_t DOF>
inline const typename LowLevelWam<DOF>::jp_type& LowLevelWam<DOF>::getJointPositions(enum PositionSensor sensor) const {
	switch (sensor) {
	case PS_MOTOR_ENCODER:
		return jp_motorEncoder;
		break;
	case PS_JOINT_ENCODER:
		return jp_jointEncoder;
		break;
	default:
		return jp_best;
	}
}

template<size_t DOF>
void LowLevelWam<DOF>::setPositionSensor(enum PositionSensor sensor)
{
	switch (sensor) {
	case PS_MOTOR_ENCODER:
		useJointEncoder.assign(false);
		break;
	case PS_JOINT_ENCODER:
		if ( !hasJointEncoders() ) {
			(logMessage("LowLevelWam::%s: This WAM is not equipped with joint encoders.")
					% __func__).template raise<std::logic_error>();
		}
		for (size_t i = 0; i < DOF; ++i) {
			if (pucks[i]->hasOption(Puck::RO_OpticalEncOnEnc)  &&  motorPucks[i].foundIndexPulse()) {
				useJointEncoder[i] = true;
			} else {
				useJointEncoder[i] = false;
			}
		}
		break;
	default:
		(logMessage("LowLevelWam::%s: Bad sensor value: %d")
				% __func__ % sensor).template raise<std::logic_error>();
		break;
	}

	positionSensor = sensor;
}


template<size_t DOF>
void LowLevelWam<DOF>::update()
{
	RTIME now = rt_timer_read();

	if (noJointEncoders) {
		group.getProperty<MotorPuck::MotorPositionParser<double> >(Puck::P, pp.data(), true);
		jp_motorEncoder = p2jp * pp;  // Convert from Puck positions to joint positions
		jp_best = jp_motorEncoder;
	} else {
		// Make sure the reinterpret_cast below makes sense.
		BOOST_STATIC_ASSERT(sizeof(MotorPuck::CombinedPositionParser<double>::result_type) == 2*sizeof(double));

		// PuckGroup::getProperty() will fill pp_jep.data() with 2*DOF doubles:
		// Primary Encoder 1, Secondary Encoder 1, Primary Encoder 2, Secondary Encoder 2, ...
		group.getProperty<MotorPuck::CombinedPositionParser<double> >(
				Puck::P,
				reinterpret_cast<MotorPuck::CombinedPositionParser<double>::result_type*>(pp_jep.data()),
				true);
		jp_motorEncoder = p2jp * pp_jep.col(0);

		for (size_t i = 0; i < DOF; ++i) {
			if (pp_jep(i,1) == std::numeric_limits<double>::max()) {
				jp_jointEncoder[i] = 0.0;
				jp_best[i] = jp_motorEncoder[i];
			} else {
				jp_jointEncoder[i] = jointEncoder2jp[i] * pp_jep(i,1);
				if (useJointEncoder[i]) {
					jp_best[i] = jp_jointEncoder[i];
				} else {
					jp_best[i] = jp_motorEncoder[i];
				}
			}
		}
	}

	jv_best = (jp_best - jp_best_1) / (1e-9 * (now - lastUpdate));
	// TODO(dc): Detect unreasonably large velocities

	jp_best_1 = jp_best;
	lastUpdate = now;
}

template<size_t DOF>
void LowLevelWam<DOF>::setTorques(const jt_type& jt)
{
	// Get around C++ address-of-static-member weirdness...
	static const size_t PUCKS_PER_TORQUE_GROUP = MotorPuck::PUCKS_PER_TORQUE_GROUP;

	pt = j2pt * jt;  // Convert from joint torques to Puck torques

	size_t i = 0;
	for (size_t g = 0; g < torqueGroups.size(); ++g) {
		MotorPuck::sendPackedTorques(bus, torqueGroups[g]->getId(), torquePropId, pt.data()+i, std::min(PUCKS_PER_TORQUE_GROUP, DOF-i));
		i += PUCKS_PER_TORQUE_GROUP;
	}
}

template<size_t DOF>
void LowLevelWam<DOF>::definePosition(const jp_type& jp)
{
	// Tell the safety logic to ignore the next velocity fault
	// (the position will appear to change rapidly)
	if (safetyModule != NULL) {
		safetyModule->ignoreNextVelocityFault();
	}

	pp = j2pp * jp;  // Convert from joint positions to Puck positions

	{
		// Synchronize with execution-cycle
		BARRETT_SCOPED_LOCK(bus.getMutex());
		for (size_t i = 0; i < DOF; ++i) {
			pucks[i]->setProperty(Puck::P, floor(pp[i]));
		}
	}

	// Record the fact that the WAM has been zeroed
	if (safetyModule != NULL) {
		safetyModule->setWamZeroed();
	}
}


}
