/** Defines barrett::Wam.
 *
 * @file barrett/wam.h
 * @date Sep 25, 2009
 * @author Dan Cody
 */

/* Copyright 2009 Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */


#ifndef BARRETT_WAM_H_
#define BARRETT_WAM_H_


#include <map>
#include <gsl/gsl_vector.h>

#include <barrett/wam/wam.h>
#include <barrett/wam/wam_local.h>

#include "./detail/ca_macro.h"
#include "./units.h"
#include "./systems/abstract/system.h"


namespace barrett {


/** A barrett::systems::System that represents a WAM robot.
 *
 * @tparam DOF The number of degrees of freedom (number of joints) in this WAM.
 *
 *
 * @section sec_example Example
 *
 * The code below is a simple program that uses \c libbarrett to control a WAM. The program
 *   - instantiates a Wam
 *   - turns on gravity compensation
 *   - moves the WAM to a particular pose (specified by \c setPoint)
 *   - moves the WAM back to its home position
 *   - idles the WAM and exits.
 *
 * @include hold_joint_position.cpp
 */
template<size_t DOF>
class Wam : public systems::System {
public:
	/// The appropriately sized barrett::units::JointTorques type.
	typedef units::JointTorques<DOF> jt_type;

	/// The appropriately sized barrett::units::JointPositions type.
	typedef units::JointPositions<DOF> jp_type;

	/// The appropriately sized barrett::units::JointVelocities type.
	typedef units::JointVelocities<DOF> jv_type;


/// @name Inputs and Outputs
//@{
public:		Input<jt_type> input;
public:		Output<jp_type> jpOutput;
public:		Output<jv_type> jvOutput;
//@}

/// @name Output handles
//@{
protected:	typename Output<jp_type>::Value* jpOutputValue;
protected:	typename Output<jv_type>::Value* jvOutputValue;
//@}


public:
	/** Starts the real-time thread that reads joint positions and commands
	 * joint torques.
	 *
	 * - Locks this process' memory (to prevent paging in a real-time thread)
	 * - Initializes the CAN bus
	 * - Creates a real-time thread (500 Hz update rate) to control the WAM
	 *
	 * The robot should be homed and Shift-Idled before instantiating a Wam.
	 */
	Wam();

	virtual ~Wam();  ///< Stops the real-time thread, closes the CAN bus.


	/// @name Asynchronous sensor accessors
	//@{
	jp_type getJointPositions();
	jv_type getJointVelocities();
	//@}


	/// @name Asynchronous interface
	//@{

	/** Exerts torques to compensate for gravity.
	 *
	 * @warning
	 * Turning off gravity compensation will, if no other torques are
	 * being commanded, cause the WAM to go limp. If it is not in a resting
	 * pose at that moment, the WAM will "drop" itself onto its join stops.
	 * This could damage the robot.
	 *
	 * @param[in] compensate \c true to turn gravity compensation on, \c false
	 *            to turn it off.
	 */
	void gravityCompensate(bool compensate = true);

	/** Moves the WAM to is home position (as defined in the configuration
	 * file) and then holds position.
	 */
	void moveHome();

	/** Tests if is the WAM is done moving.
	 *
	 * @retval true if the WAM has reached is destination.
	 * @retval false if the WAM is still in-route.
	 */
	bool moveIsDone();

	/// Cancels any active moves and releases any position constraints.
	void idle();

	//@}


	/// @cond DETAIL
	// TODO(dc): can we make this private?
	static int handleCallback(struct bt_wam_local* wamLocal);

	int operateCount;  ///< For debugging.
	/// @endcond

protected:
	class JTSink : public systems::System {
	public:
		JTSink(Wam<DOF>* wamPtr) :
			systems::System(true), wam(wamPtr) {}
		virtual ~JTSink() {}

	protected:
		virtual void operate() {
			++(wam->operateCount);
			if (wam->input.valueDefined()) {
				const jt_type& jt = wam->input.getValue();
				gsl_vector_add(wam->wamLocal->jtorque, jt.asGslVector());
			}

		    /* Apply the current joint torques */
		    bt_wambot_setjtor( wam->wamLocal->wambot );
		}

		Wam<DOF>* wam;
	} jtSink;
	friend class JTSink;

	virtual void readSensors();
	virtual bool inputsValid();
	virtual void operate();
	virtual void invalidateOutputs();


	struct bt_wam* wam;
	struct bt_wam_local* wamLocal;

	/// @cond DETAIL
	// TODO(dc): can we make this private?
	static std::map<struct bt_wam_local*, Wam<DOF>*> activeWams;
	/// @endcond

private:
	DISALLOW_COPY_AND_ASSIGN(Wam);
};


}


// include template definitions
#include "./detail/wam-inl.h"


#endif /* BARRETT_WAM_H_ */
