#include <iostream>
#include <string>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

#include "robust_cartesian.h"

using namespace barrett;

template<size_t DOF>
  int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    cp_type cartSetPoint;
    wam.gravityCompensate();

    /////////////////////////////////////////////////////////////////////////////////////
    // Add the following to our system to make our Cartesian controller much more robust.
    /////////////////////////////////////////////////////////////////////////////////////

    // Set up singularity avoidance springs
    jp_type joint_center(0.0); // Initialize our joint centers to 0.0 on all joints
    std::vector<double> spring_constants(DOF); // create spring constants
    std::vector<double> damping_constants(DOF); // create damping constants
    joint_center[0] = 0.0;
    joint_center[1] = 0.0;
    joint_center[2] = 0.0;
    joint_center[3] = 1.1; // J4 Joint Range Center at 1.1 Radians
    spring_constants[0] = 15.0;
    spring_constants[1] = 5.0;
    spring_constants[2] = 5.0;
    spring_constants[3] = 5.0;
    damping_constants[0] = 10.0;
    damping_constants[1] = 20.0;
    damping_constants[2] = 10.0;
    damping_constants[3] = 6.0;

    if (DOF == 7)
    {
      joint_center[4] = -1.76; // J5 Joint Range Center at -1.76 Radians
      joint_center[5] = 0.0;
      joint_center[6] = 0.0;
      spring_constants[4] = 0.5;
      spring_constants[5] = 0.25;
      spring_constants[6] = 0.25;
      damping_constants[4] = 0.05;
      damping_constants[5] = 0.05;
      damping_constants[6] = 0.0;
    }

    printf("Press Enter to Turn on Haptic Singularity Avoidance.\n");
    detail::waitForEnter();

    //Initialization Move
    jp_type wam_init = wam.getHomePosition();
    wam_init[3] -= .35;
    wam.moveTo(wam_init); // Adjust the elbow, moving the end-effector out of the haptic boundary and hold position for haptic force initialization.

    // Change decrease our tool position controller gains slightly
    cp_type cp_kp, cp_kd;
    for (size_t i = 0; i < 3; i++)
    {
      cp_kp[i] = 1500;
      cp_kd[i] = 5.0;
    }
    wam.tpController.setKp(cp_kp);
    wam.tpController.setKd(cp_kd);

    //Torque Summer from our three systems
    systems::Summer<jt_type, 4> singJTSum(true);

    // Our singularity avoidance system
    SingularityAvoid<DOF> singularityAvoid(joint_center);
    systems::connect(wam.jpOutput, singularityAvoid.input);
    systems::connect(singularityAvoid.output, singJTSum.getInput(0));

    // Attach our joint stop springs
    JointStopSprings<DOF> jointStopSprings(joint_center, spring_constants);
    systems::connect(wam.jpOutput, jointStopSprings.input);
    systems::connect(jointStopSprings.output, singJTSum.getInput(1));

    // Joint velocity damper for kinematic solutions causing velocity faults
    JVDamper<DOF> jvDamper(damping_constants);
    systems::connect(wam.jvOutput, jvDamper.input);
    systems::connect(jvDamper.output, singJTSum.getInput(2));

    // Haptic collision avoidance boundary portion
    HapticCollisionAvoid<DOF> hapticCollisionAvoid(2000);
    systems::ToolForceToJointTorques<DOF> tf2jt;
    systems::connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
    systems::connect(wam.toolPosition.output, hapticCollisionAvoid.input);
    systems::connect(hapticCollisionAvoid.output, tf2jt.input);
    systems::connect(tf2jt.output, singJTSum.getInput(3));

    systems::connect(singJTSum.output, wam.input);

    // New system watchdogs to monitor and stop trajectories if expecting a fault
    TorqueWatchdog<DOF> torqueWatchdog;
    VelocityWatchdog<DOF> velocityWatchdog;
    pm.getExecutionManager()->startManaging(torqueWatchdog);
    pm.getExecutionManager()->startManaging(velocityWatchdog);

    systems::connect(wam.jtSum.output, torqueWatchdog.input);
    systems::connect(wam.jvOutput, velocityWatchdog.input);

    /////////////////////////////////////////////////////////////////////////////////////
    // End of robust cartesian setup
    /////////////////////////////////////////////////////////////////////////////////////

    printf("Press Enter to start test.\n");
    detail::waitForEnter();

    int i;
    for (i = 1; i < 31; i++) //from random.cc
    {
      srand(time(NULL));
      double x = -1 + 2 * ((double)rand()) / RAND_MAX;
      double y = -1 + 2 * (((double)rand()) / ((double)RAND_MAX));
      double z = ((double)rand()) / ((double)RAND_MAX);
      cartSetPoint[0] = x;
      cartSetPoint[1] = y;
      cartSetPoint[2] = z;

      double reach = sqrt(x * x + y * y + z * z);

      if (reach < 0.95)
      {
        std::cout << i << ": Moving to coordinate " << x << ", " << y << ", " << z << ".\n";
        std::cout << reach << "\n";
        wam.moveTo(cartSetPoint, false);
        torqueWatchdog.activate();
        velocityWatchdog.activate(); // The situation here is interesting. A velocity fault can present itself from Cartesian end-effector or elbow velocities,
        // However, in its current capacity, we can only monitor joint velocities, or end-effector velocities.
        bool faulted = false;
        while (!wam.moveIsDone() && !faulted)
        {
          //Check our velocity and torque output to see if crossing threshold (approaching a fault situation)
          jt_type curJT = torqueWatchdog.getCurrentTorque();
          jv_type curJV = velocityWatchdog.getCurrentVelocity();
          for (size_t i = 0; i < DOF; i++)
          {
            if (curJT[i] > 30.0 || curJV[i] > 0.9)
            {
              wam.idle();
              printf("Stopping Move - Fault presented on Joint %zu - Torque: %f, Velocity: %f\n", i, curJT[i],
                     curJV[i]);
              faulted = true;
              wam.moveTo(wam.getJointPositions());
            }
          }
          btsleep(0.01);
        }
        torqueWatchdog.deactivate();
        velocityWatchdog.deactivate();
        continue;
      }
      else
        i--;

      continue;

    }

    systems::disconnect(wam.input);
    wam.moveHome();
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    return 0;

  }
