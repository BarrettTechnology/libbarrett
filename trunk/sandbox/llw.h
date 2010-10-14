/*
 * llw.h
 *
 *  Created on: Oct 13, 2010
 *      Author: cd
 *      Author: dc
 */

#ifndef LLW_H_
#define LLW_H_


#include <stdexcept>
#include <cmath>

#include <syslog.h>

#include <Eigen/LU>
#include <libconfig.h++>

#include <barrett/puck.h>
#include <barrett/puck_group.h>
#include <barrett/units.h>


namespace barrett {


class WamPuck {
public:
	WamPuck(Puck* puck = NULL) {
		setPuck(puck);
	}

	void setPuck(Puck* puck, bool autoUpdate = true) {
		p = puck;
		if (autoUpdate) {
			update();
		}
	}
	Puck* getPuck() const { return p; }
	void update() {
		if (p == NULL) {
			// for debugging
			cts = 4096;
		} else {
			cts = getProperty(Puck::CTS);
		}

		radsPerCount = 2*M_PI / cts;
		countsPerRad = cts / (2*M_PI);
	}

	int getProperty(enum Puck::Property prop) const {
		return p->getProperty(prop);
	}
	void setProperty(enum Puck::Property prop, int value) const {
		p->setProperty(prop, value);
	}

	int getId() const { return p->getId(); }
	int getVers() const { return p->getRole(); }
	int getRole() const { return p->getRole(); }

	int getCts() const { return cts; }

	double counts2rad(int counts) const { return radsPerCount * counts; }
	int rad2counts(double rad) const { return floor(countsPerRad * rad); }

protected:
	Puck* p;

	int cts;
	double radsPerCount, countsPerRad;
};


template<size_t DOF>
class LLW {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	LLW(const std::vector<Puck*>& normalPucks, Puck* _safetyPuck, const libconfig::Setting& setting) :
		pucks(DOF), safetyPuck(_safetyPuck), home(setting["home"]), j2mp(setting["j2mp"]), m2jp(), j2mt()
	{
		syslog(LOG_ERR, "LLW::LLW(%s => \"%s\")", setting.getSourceFile(), setting.getPath().c_str());

		// Check number of Pucks
		if (normalPucks.size() != DOF) {
			syslog(LOG_ERR, "  Expected a vector of %d Pucks, got %d", DOF, pucks.size());
			throw std::invalid_argument("LLW::LLW(): Wrong number of Pucks. Check /var/log/syslog for details.");
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
			throw std::runtime_error("LLW::LLW(): j2mp matrix is not invertible.");
		}
		lu.computeInverse(&m2jp);
		j2mt = m2jp.transpose();


		// Initialize WamPucks
		Puck::wake(normalPucks);  // Make sure Pucks are awake
		for (size_t i = 0; i < DOF; ++i) {
			pucks[i].setPuck(normalPucks[i]);
		}

		// Zero the WAM?
		if (safetyPuck == NULL) {
			syslog(LOG_ERR, "  No safetyPuck: WAM may not be zeroed");
		} else if (safetyPuck->getProperty(Puck::ZERO)) {
			syslog(LOG_ERR, "  WAM was already zeroed");
		} else if (zeroCompensation) {
			// TODO(dc): test

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
				if (pucks[i].getRole() & 256) {
					syslog(LOG_ERR, "  No zero-compensation for Puck %d: no absolute encoder", pucks[i].getId());
					errorAngle[i] = 0;
					continue;
				}

				// If there was an error during the calibration process, don't compensate
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
	}
	~LLW() {}

	// TODO(dc): test
	void definePosition(jp_type jp) {
		v_type mp = j2mp * jp;  // Convert from joint space to motor space

		// Tell the safety logic to ignore the next several faults
		// (the position will appear to be changing rapidly)
		if (safetyPuck != NULL) {
			safetyPuck->setProperty(Puck::IFAULT, 8);  // TODO(dc): Why 8?
		}

		for (size_t i = 0; i < DOF; ++i) {
			pucks[i].setProperty(Puck::P, pucks[i].rad2counts(mp[i]));
			usleep(1000);  // TODO(dc): necessary?
		}

		// Record the fact that the WAM has been zeroed
		if (safetyPuck != NULL) {
			safetyPuck->setProperty(Puck::ZERO, 1);
		}
	}

protected:
	std::vector<WamPuck> pucks;
	Puck* safetyPuck;

	jp_type home;
	sqm_type j2mp, m2jp, j2mt;

private:
	DISALLOW_COPY_AND_ASSIGN(LLW);
};


}


#endif /* LLW_H_ */
