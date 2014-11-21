/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */
/**
 * @file tactile_puck.h
 * @date 11/12/2010
 * @author Dan Cody
 * 
 */

#ifndef BARRETT_PRODUCTS_TACTILE_PUCK_H_
#define BARRETT_PRODUCTS_TACTILE_PUCK_H_


#include <barrett/math/matrix.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>
#include <barrett/products/abstract/special_puck.h>


namespace barrett {


class TactilePuck : public SpecialPuck {
public:
	static const size_t NUM_SENSORS = 24;
	typedef math::Vector<NUM_SENSORS>::type v_type;

protected:
	enum TactState { NONE, TOP10_FORMAT, FULL_FORMAT, TARE };

public:
	/**
	 *
	 */
	TactilePuck(Puck* puck = NULL) : SpecialPuck() { setPuck(puck); }
	~TactilePuck() {}
/**
 *
 */	
	void setPuck(Puck* puck);
/**
 *
 */
	void tare() { p->setProperty(Puck::TACT, TARE); }
/**
 *
 */	
	void updateFull(bool realtime = false) {
		requestFull();
		receiveFull(realtime);
	}
//	void updateTop10(bool realtime = false);
/**
 *
 */
	const v_type& getFullData() const { return full; }

/**
 *
 */
	void requestFull();
/**
 *
 */
	void receiveFull(bool realtime = false);

/**
 *
 */
	struct FullTactParser {
		static int busId(int id, int propId) {
			return Puck::encodeBusId(id, PuckGroup::FGRP_TACT_FULL);
		}

		typedef v_type result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len);
	};
	// TODO(dc): Implement Top10TactParser
//	struct Top10TactParser {
//		static int busId(int id, int propId) {
//			return Puck::encodeBusId(id, PuckGroup::FGRP_TACT_TOP10);
//		}
//
//		typedef int result_type;
//		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len) { return 0; }
//	};

protected:
	const bus::CommunicationsBus* bus;
	int id;
	int propId;

	enum TactState tact;
	v_type full;
//	int top10;


	static const size_t NUM_FULL_MESSAGES = 5;
	static const size_t NUM_SENSORS_PER_FULL_MESSAGE = 5;
	static const double FULL_SCALE_FACTOR = 256.0;


	friend class Hand;
};


}


#endif /* BARRETT_PRODUCTS_TACTILE_PUCK_H_ */
