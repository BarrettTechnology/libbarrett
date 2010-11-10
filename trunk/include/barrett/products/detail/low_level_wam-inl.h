/*
 * low_level_wam-inl.h
 *
 *  Created on: Nov 2, 2010
 *      Author: dc
 */


#include <stdexcept>
#include <vector>
#include <cmath>

#include <syslog.h>

#include <native/timer.h>

#include <Eigen/LU>
#include <libconfig.h++>

#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/products/puck.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/puck_group.h>

#include <barrett/products/low_level_wam.h>


namespace barrett {


template<size_t DOF>
LowLevelWam<DOF>::LowLevelWam(const std::vector<Puck*>& genericPucks, SafetyModule* _safetyModule, const libconfig::Setting& setting, std::vector<int> torqueGroupIds) :
	bus(genericPucks.at(0)->getBus()), pucks(DOF), safetyModule(_safetyModule), wamGroup(PuckGroup::BGRP_WAM, genericPucks), torqueGroups(), home(setting["home"]), j2mp(setting["j2mp"]), lastUpdate(0), torquePropId(-1)
{
	syslog(LOG_ERR, "LowLevelWam::LowLevelWam(%s => \"%s\")", setting.getSourceFile(), setting.getPath().c_str());

	// Check number of Pucks
	if (genericPucks.size() != DOF) {
		syslog(LOG_ERR, "  Expected a vector of %d Pucks, got %d", DOF, pucks.size());
		throw std::invalid_argument("LowLevelWam::LowLevelWam(): Wrong number of Pucks. Check /var/log/syslog for details.");
	}

	// Zero-compensation?
	bool zeroCompensation = setting.exists("zeroangle");
	if (!zeroCompensation) {
		syslog(LOG_ERR, "  Missing \"zeroangle\" vector: no zero-compensation");
	}

	// Compute motor/joint transforms
	Eigen::LU<typename sqm_type::Base> lu(j2mp);
	if (!lu.isInvertible()) {
		syslog(LOG_ERR, "  j2mp matrix is not invertible");
		throw std::runtime_error("LowLevelWam::LowLevelWam(): j2mp matrix is not invertible.");
	}
	lu.computeInverse(&m2jp);
	j2mt = m2jp.transpose();


	// Initialize MotorPucks
	Puck::wake(genericPucks);  // Make sure Pucks are awake
	for (size_t i = 0; i < DOF; ++i) {
		pucks[i].setPuck(genericPucks[i]);
	}

	// Setup PuckGroups
	// If no IDs are provided, use the defaults
	if (torqueGroupIds.size() == 0) {
		torqueGroupIds.push_back(PuckGroup::BGRP_LOWER_WAM);
		torqueGroupIds.push_back(PuckGroup::BGRP_UPPER_WAM);
	}
	size_t numTorqueGroups = ceil(static_cast<double>(DOF)/MotorPuck::PUCKS_PER_TORQUE_GROUP);
	if (numTorqueGroups > torqueGroupIds.size()) {
		syslog(LOG_ERR, "  Need %d torque groups, only %d IDs provided", numTorqueGroups, torqueGroupIds.size());
		throw std::logic_error("LowLevelWam::LowLevelWam(): Too few torque group IDs provided. Check /var/log/syslog for details.");
	}

	size_t i = 0;
	for (size_t g = 0; g < numTorqueGroups; ++g) {
		std::vector<Puck*> tgPucks;
		while (tgPucks.size() < 4  &&  i < DOF) {
			tgPucks.push_back(genericPucks[i]);
			++i;
		}
		torqueGroups.push_back(new PuckGroup(torqueGroupIds[g], tgPucks));
	}

	// Verify properties
	bool err = false;
	std::vector<enum Puck::Property> props;
	props.push_back(Puck::P);
	props.push_back(Puck::T);
	for (size_t i = 0; i < props.size(); ++i) {
		if ( !wamGroup.verifyProperty(props[i]) ) {
			err = true;
			syslog(LOG_ERR, "  Incompatible property: %s", Puck::getPropertyStr(props[i]));
		}
	}
	if (err) {
		syslog(LOG_ERR, "  Some Pucks might...");
		syslog(LOG_ERR, "    a) still be in Monitor");
		syslog(LOG_ERR, "    b) have incompatible firmware versions");
		syslog(LOG_ERR, "    c) have incompatible ROLEs");
		throw std::runtime_error("LowLevelWam::LowLevelWam(): WAM Pucks have incompatible property lists. Check /var/log/syslog for details.");
	}
	torquePropId = wamGroup.getPropertyId(Puck::T);


	// Compute puck/joint transforms
	// (See DC notebook 1p25)
	v_type cpr;
	for (size_t i = 0; i < DOF; ++i) {
		cpr[i] = pucks[i].getCountsPerRad();
	}
	j2pp = cpr.asDiagonal() * j2mp;

	v_type rpc;
	for (size_t i = 0; i < DOF; ++i) {
		rpc[i] = pucks[i].getRadsPerCount();
	}
	p2jp = m2jp * rpc.asDiagonal();

	v_type ipnm;
	for (size_t i = 0; i < DOF; ++i) {
		ipnm[i] = pucks[i].getIpnm();
	}
	j2pt = ipnm.asDiagonal() * j2mt;


	// Zero the WAM?
	if (safetyModule == NULL) {
		syslog(LOG_ERR, "  No safetyModule: WAM may not be zeroed");
	} else if (safetyModule->wamIsZeroed()) {
		syslog(LOG_ERR, "  WAM was already zeroed");
	} else if (zeroCompensation) {
		v_type zeroAngle(setting["zeroangle"]);

		v_type currentAngle;
		for (size_t i = 0; i < DOF; ++i) {
			currentAngle[i] = pucks[i].counts2rad(pucks[i].getProperty(Puck::MECH));
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
			if (pucks[i].getVers() < 118) {
				syslog(LOG_ERR, "  No zero-compensation for Puck %d: old firmware", pucks[i].getId());
				errorAngle[i] = 0;
				continue;
			}

			// If not ROLE & 256, then it's not an absolute encoder; don't compensate
			if ( !(pucks[i].getRole() & 256) ) {
				syslog(LOG_ERR, "  No zero-compensation for Puck %d: no absolute encoder", pucks[i].getId());
				errorAngle[i] = 0;
				continue;
			}

			// If the calibration data is out of range, don't compensate
			if (zeroAngle[i] > 2*M_PI  ||  zeroAngle[i] < 0) {
				syslog(LOG_ERR, "  No zero-compensation for Puck %d: bad calibration data", pucks[i].getId());
				errorAngle[i] = 0;
				continue;
			}
		}

		definePosition(home - m2jp*errorAngle);
		syslog(LOG_ERR, "  WAM zeroed with zero-compensation");
	} else {
		definePosition(home);
		syslog(LOG_ERR, "  WAM zeroed without zero-compensation");
	}


	// Get good initial values for jp_1 and lastUpdate
	update();
	jv.setZero();
}

template<size_t DOF>
LowLevelWam<DOF>::~LowLevelWam()
{
	detail::purge(torqueGroups);
}

template<size_t DOF>
void LowLevelWam<DOF>::update()
{
	RTIME now = rt_timer_read();
	wamGroup.getProperty<MotorPuck::PositionParser>(Puck::P, pp.data(), true);

	jp = p2jp * pp;  // Convert from Puck positions to joint positions
	jv = (jp - jp_1) / (1e-9 * (now - lastUpdate));
	// TODO(dc): Detect unreasonably large velocities

	jp_1 = jp;
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
	// Tell the safety logic to ignore the next several velocity faults
	// (the position will appear to change rapidly)
	if (safetyModule != NULL) {
		safetyModule->setProperty(Puck::IFAULT, SafetyModule::VELOCITY_FAULT_HISTORY_BUFFER_SIZE);
	}

	pp = j2pp * jp;  // Convert from joint positions to Puck positions

	{
		// Synchronize with execution-cycle
		SCOPED_LOCK(bus.getMutex());
		for (size_t i = 0; i < DOF; ++i) {
			pucks[i].setProperty(Puck::P, floor(pp[i]));
		}
	}

	// Record the fact that the WAM has been zeroed
	if (safetyModule != NULL) {
		safetyModule->setWamZeroed();
	}
}


}
