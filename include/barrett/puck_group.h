/*
 * puck_group.h
 *
 *  Created on: Oct 7, 2010
 *      Author: dc
 */

#ifndef BARRETT_PUCK_GROUP_H_
#define BARRETT_PUCK_GROUP_H_


#include <vector>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/puck.h>


namespace barrett {


class PuckGroup {
public:
	PuckGroup(int id, const std::vector<Puck*>& pucks);
	~PuckGroup();

	int getId() const { return id; }
	const std::vector<Puck*>& getPucks() const { return pucks; }

	bool verifyProperty(enum Puck::Property prop) const;

	int getProperty(enum Puck::Property prop) const;
	void setProperty(enum Puck::Property prop, int value) const;

	int getPropertyId(enum Puck::Property prop) const {
		return pucks[0]->getPropertyId(prop);
	}
	int getPropertyIdNoThrow(enum Puck::Property prop) const {
		return pucks[0]->getPropertyIdNoThrow(prop);
	}


	enum BroadcastGroup {
		BGRP_WHOLE_BUS = Puck::GROUP_MASK | 0,  // Everything but the Safety Puck

		BGRP_WAM = Puck::GROUP_MASK | 4,  // The whole WAM (Pucks 1-7)
		BGRP_LOWER_WAM = Puck::GROUP_MASK | 1,  // A packed-torque group (Pucks 1-4)
		BGRP_UPPER_WAM = Puck::GROUP_MASK | 2,  // A packed-torque group (Pucks 5-7)

		BGRP_BHAND = Puck::GROUP_MASK | 5,  // The whole hand (Pucks 11-14)
	};

	enum FeedbackGroup {
		// When responding to requests for a normal property, Pucks send to Group 6.
		FGRP_OTHER = Puck::GROUP_MASK | 6,
		// When responding to position requests, Pucks send to Group 3.
		FGRP_POSITION = Puck::GROUP_MASK | 3,

		// On BHands with tactile sensors, TOP10 formated TACT data is sent to Group 8.
		FGRP_TACT_TOP10 = Puck::GROUP_MASK | 8,
		// On BHands with tactile sensors, FULL formated TACT data is sent to Group 9.
		FGRP_TACT_FULL = Puck::GROUP_MASK | 9,

		FGRP_FT_FORCE = Puck::GROUP_MASK | 10,  // Force data from the F/T Sensor is sent to group 10.
		FGRP_FT_TORQUE = Puck::GROUP_MASK | 11,  // Torque data from the F/T Sensor is sent to group 11.

		FGRP_RESERVED7 = Puck::GROUP_MASK | 7,  // Reserved
	};

protected:
	int id;
	std::vector<Puck*> pucks;
	const CommunicationsBus& bus;
};


}


#endif /* BARRETT_PUCK_GROUP_H_ */
