/*
 * low_level_wam-inl.h
 *
 *  Created on: Nov 2, 2010
 *      Author: cd
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
#include <barrett/products/abstract/multi_puck_product.h>

#include <barrett/products/low_level_wam.h>


namespace barrett {


template<size_t DOF>
const enum Puck::Property LowLevelWam<DOF>::props[] = { Puck::P, Puck::T };


template<size_t DOF>
LowLevelWam<DOF>::LowLevelWam(const std::vector<Puck*>& _pucks, SafetyModule* _safetyModule, const libconfig::Setting& setting, std::vector<int> torqueGroupIds) :
	MultiPuckProduct(DOF, _pucks, PuckGroup::BGRP_WAM, props, sizeof(props)/sizeof(props[0]), "LowLevelWam::LowLevelWam()"),
	safetyModule(_safetyModule), torqueGroups(), home(setting["home"]), j2mp(setting["j2mp"]), lastUpdate(0), torquePropId(group.getPropertyId(Puck::T))
{
	syslog(LOG_ERR, "  Config setting: %s => \"%s\"", setting.getSourceFile(), setting.getPath().c_str());


	group.setProperty(Puck::MODE, MotorPuck::MODE_IDLE);  // Make sure the Pucks are IDLE


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


	// Setup torque groups
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


	// Init velocity/acceration Kalman filter
	double dt = 0.000;  // Place holder for clarity only. The values of these elements are calculated in update().
	A << 1, dt, 0.5 * dt*dt,
		 0,  1,          dt,
		 0,  0,           1;
	C << 1, 0, 0;
	Q << 0, 0,   0,
		 0, 0,   0,
		 0, 0, 100;
	R = 5e-6;


	// Zero the WAM?
	if (safetyModule == NULL) {
		syslog(LOG_ERR, "  No safetyModule: WAM may not be zeroed");
	} else if (safetyModule->wamIsZeroed()) {
		syslog(LOG_ERR, "  WAM was already zeroed");
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
				syslog(LOG_ERR, "  No zero-compensation for Puck %d: old firmware", pucks[i]->getId());
				errorAngle[i] = 0;
				continue;
			}

			// If not ROLE & 256, then it's not an absolute encoder; don't compensate
			if ( !(pucks[i]->hasOption(Puck::RO_MagEncOnSerial)) ) {
				syslog(LOG_ERR, "  No zero-compensation for Puck %d: no absolute encoder", pucks[i]->getId());
				errorAngle[i] = 0;
				continue;
			}

			// If the calibration data is out of range, don't compensate
			if (zeroAngle[i] > 2*M_PI  ||  zeroAngle[i] < 0) {
				syslog(LOG_ERR, "  No zero-compensation for Puck %d: bad calibration data", pucks[i]->getId());
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


	// Get good initial value for jp and lastUpdate...
	update();

	// ... then reset state.
	for (size_t i = 0; i < DOF; ++i) {
		X_1[i] << jp[i],
				      0,
				      0;
		P_1[i] << 1e-9,    0, 0,
				     0, 5e-4, 0,
				     0,    0, 1;
	}
	jv.setZero();
	ja.setZero();
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

	group.getProperty<MotorPuck::MotorPositionParser<double> >(Puck::P, pp.data(), true);

	jp = p2jp * pp;  // Convert from Puck positions to joint positions


	// Kalman filter
	double dt = 1e-9 * (now - lastUpdate);
	A(0,1) = dt;
	A(0,2) = 0.5 * dt*dt;
	A(1,2) = dt;

	for (size_t i = 0; i < DOF; ++i) {
		X_priori = A * X_1[i];
		P_priori = A * P_1[i] * A.transpose() + Q;

		// Hard-coded C
		Kal = P_priori.col(0) / (P_priori(0,0) + R);
		X_1[i] = X_priori + Kal * (jp[i] - X_priori[0]);
		P_1[i] = P_priori - Kal * C * P_priori;

		jv[i] = X_1[i][1];
		ja[i] = X_1[i][2];
	}


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
		safetyModule->getPuck()->setProperty(Puck::IFAULT, SafetyModule::VELOCITY_FAULT_HISTORY_BUFFER_SIZE);
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
