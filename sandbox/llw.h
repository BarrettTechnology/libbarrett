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
#include <syslog.h>

#include <Eigen/LU>
#include <libconfig.h++>

#include <barrett/puck.h>
#include <barrett/puck_group.h>
#include <barrett/units.h>


namespace barrett {


template<size_t DOF>
class LLW {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	LLW(const std::vector<Puck*>& wamPucks, Puck* _safetyPuck, const libconfig::Setting& setting) :
		pucks(wamPucks), safetyPuck(_safetyPuck), home(setting["home"]), j2mp(setting["j2mp"]), m2jp(), j2mt()
	{
		syslog(LOG_ERR, "LLW::LLW(%s => \"%s\")", setting.getSourceFile(), setting.getPath().c_str());

		// Check number of Pucks
		if (pucks.size() != DOF) {
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

		// Make sure Pucks are awake
		Puck::wake(pucks);

		// Zero the WAM?
		if (safetyPuck == NULL) {
			syslog(LOG_ERR, "  No safetyPuck: WAM may not be zeroed");
		} else if (safetyPuck->getProperty(Puck::ZERO)) {
			syslog(LOG_ERR, "  WAM was already zeroed");
		} else if (zeroCompensation) {
			v_type zeroAngle(setting["zeroangle"]);
			// TODO(dc): stub
			syslog(LOG_ERR, "  WAM zeroed with zero-compensation");
		} else {
			definePosition(home);
			syslog(LOG_ERR, "  WAM zeroed without zero-compensation");
		}
	}
	~LLW() {}

	void definePosition(jp_type jp) {
		// TODO(dc): stub
	}

protected:
	std::vector<Puck*> pucks;
	Puck* safetyPuck;

	jp_type home;
	sqm_type j2mp, m2jp, j2mt;

private:
	DISALLOW_COPY_AND_ASSIGN(LLW);
};


}


#endif /* LLW_H_ */
