#ifndef ROBUST_CARTESIAN_H_
#define ROBUST_CARTESIAN_H_

#include <barrett/units.h>
#include <barrett/systems.h>

using namespace barrett;

// TorqueWatchdog monitoring system
template<size_t DOF>
  class TorqueWatchdog : public systems::SingleIO<typename barrett::units::JointTorques<DOF>::type,
      typename barrett::units::JointTorques<DOF>::type>
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    explicit TorqueWatchdog(const std::string& sysName = "TorqueWatchdog") :
        barrett::systems::SingleIO<jt_type, jt_type>(sysName)
    {
      watching = false;
    }

    virtual ~TorqueWatchdog()
    {
      this->mandatoryCleanUp();
    }

    void activate()
    {
      watching = true;
    }

    void deactivate()
    {
      watching = false;
    }

    jt_type getCurrentTorque()
    {
      return torque;
    }

  protected:
    virtual void operate()
    {
      if (watching)
        torque = this->input.getValue();
    }

  public:
    bool watching;
    jt_type torque;

  private:
    DISALLOW_COPY_AND_ASSIGN(TorqueWatchdog);
  };

// Joint VelocityWatchdog monitoring system
template<size_t DOF>
  class VelocityWatchdog : public systems::SingleIO<typename barrett::units::JointVelocities<DOF>::type,
      typename barrett::units::JointVelocities<DOF>::type>
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    explicit VelocityWatchdog(const std::string& sysName = "VelocityWatchdog") :
        barrett::systems::SingleIO<jv_type, jv_type>(sysName)
    {
      watching = false;
    }

    virtual ~VelocityWatchdog()
    {
      this->mandatoryCleanUp();
    }

    void activate()
    {
      watching = true;
    }

    void deactivate()
    {
      watching = false;
    }

    jv_type getCurrentVelocity()
    {
      return velocity;
    }

  protected:

    virtual void operate()
    {
      if (watching)
        velocity = this->input.getValue();
    }

  public:
    bool watching;
    jv_type velocity;

  private:
    DISALLOW_COPY_AND_ASSIGN(VelocityWatchdog);
  };

// Singularity Avoidance System
template<size_t DOF>
  class SingularityAvoid : public systems::SingleIO<typename units::JointPositions<DOF>::type,
      typename units::JointTorques<DOF>::type>
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    SingularityAvoid(const jp_type& jointCenter, const double& spring_const = 10.0, const double& singularity_buffer =
                         0.17453,
                     const std::string& sysName = "SingularityAvoid") : // Hooke's law spring const. default 10, singularity buffer of 10 degrees
        systems::SingleIO<jp_type, jt_type>(sysName),jointRangeCenter(jointCenter), singularBoundary(false), lastBoundary(
            false), singularitySpringConst(spring_const), singularityBuffer(singularity_buffer)
    {
    }

    virtual ~SingularityAvoid()
    {
      this->mandatoryCleanUp();
    }
  protected:

    virtual void operate()
    {
      wamJP = this->input.getValue();

      singularBoundary = false;
      // Check if we meet conditions to turn on haptic singularity boundary
      if ((wamJP[1] > jointRangeCenter[1] - singularityBuffer && wamJP[1] < jointRangeCenter[1] + singularityBuffer)
          && (wamJP[2] > (jointRangeCenter[2] + M_PI / 2) - singularityBuffer
              && wamJP[2] < (jointRangeCenter[2] + M_PI / 2) + singularityBuffer))
      { // J2 is within +-10 degrees of J2=0 - singularity causing && J3 is within +-10 degrees of J2=+PI/2 - singularity causing
        singularBoundary = true;
        if (!lastBoundary)
        { // Create our spring to pull J1 to perturb the controller avoiding the singularity
          if (wamJP[0] > -M_PI / 2) // We will rotate in the negative direction unless we are near the j1 negative joint stop and do not want to rotate into the stop.
            hapticSpringOrigin[0] = wamJP[0] - 0.75;
          else
            hapticSpringOrigin[0] = wamJP[0] + 0.75;
        }
        // Command the force to J1 pulling the WAM out of the singularity
        jtSingularityAvoid[0] = (hapticSpringOrigin[0] - wamJP[0]) * singularitySpringConst;
      }
      else if ((wamJP[1] > jointRangeCenter[1] - singularityBuffer && wamJP[1] < jointRangeCenter[1] + singularityBuffer)
          && (wamJP[2] > (jointRangeCenter[2] - M_PI / 2) - singularityBuffer
              && wamJP[2] < (jointRangeCenter[2] - M_PI / 2) + singularityBuffer))
      { // J2 is within +-10 degrees of J2=0 - singularity causing && J3 is within +-10 degrees of J2=-PI/2 - singularity causing
        singularBoundary = true;
        if (!lastBoundary)
        { // Create our spring to pull J1 to perturb the controller avoiding the singularity
          if (wamJP[0] < M_PI / 2) // We will rotate in the positive direction moving J3 towards its center point unless we are near the j1 positive joint stop and do not want to rotate into the stop.
            hapticSpringOrigin[0] = wamJP[0] + 0.75;
          else
            hapticSpringOrigin[0] = wamJP[0] - 0.75;
        }
        // Command the force to J1 pulling the WAM out of the singularity
        jtSingularityAvoid[0] = (hapticSpringOrigin[0] - wamJP[0]) * singularitySpringConst;
      }

      lastBoundary = singularBoundary;
      this->outputValue->setData(&jtSingularityAvoid);
    }

  public:
    jp_type wamJP, hapticSpringOrigin, jointRangeCenter;
    jt_type jtSingularityAvoid;
    bool singularBoundary, lastBoundary;
    double singularitySpringConst, singularityBuffer;
  private:
    DISALLOW_COPY_AND_ASSIGN(SingularityAvoid);
  }
  ;

// Joint Stop Avoidance Springs
template<size_t DOF>
  class JointStopSprings : public systems::SingleIO<typename units::JointPositions<DOF>::type,
      typename units::JointTorques<DOF>::type>
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    JointStopSprings(const jp_type& jointCenter, const std::vector<double>& springs, const std::string& sysName =
                         "JointStopSprings") :
        systems::SingleIO<jp_type, jt_type>(sysName),jointRangeCenter(jointCenter), springConstants(springs)
    {
    }

    virtual ~JointStopSprings()
    {
      this->mandatoryCleanUp();
    }
  protected:

    virtual void operate()
    {
      wamSpringJP = this->input.getValue();
      for (size_t i = 0; i < DOF; i++)
      {
        if (i != 1 && i != 5)
          jtSpring[i] = (jointRangeCenter[i] - wamSpringJP[i]) * springConstants[i];
        else
        {
          if (wamSpringJP[1] > 0.0) // Spring for J2 will be at 1.0 or -1.0 radians, middle of joint range in either hemisphere to avoiding pulling J2 to 0.0 (singularity causing)
            jtSpring[1] = (1.0 - wamSpringJP[1]) * springConstants[1];
          else if (wamSpringJP[1] < 0.0)
            jtSpring[1] = (-1.0 - wamSpringJP[1]) * springConstants[1];
          if (DOF != 4)
          {
            if (wamSpringJP[5] > 0.0) // Spring for J6 will be at 0.8 or -0.8 radians, middle of joint range in either hemisphere to avoiding pulling J6 to 0.0 (singularity causing)
              jtSpring[5] = (0.8 - wamSpringJP[5]) * springConstants[5];
            else if (wamSpringJP[5] < 0.0)
              jtSpring[5] = (-0.8 - wamSpringJP[5]) * springConstants[5];

            //Spring force on J4 to avoid wrist making collision with inner link
            wamSpringDiff = wamSpringJP[3] - 2.78; // Make a vector between the WAMs CP and the center of the haptic sphere
            if (wamSpringDiff > 0)
              jtSpring[3] += -100 * (wamSpringDiff);
          }
        }
      }
      this->outputValue->setData(&jtSpring);
    }

  public:
    jp_type wamSpringJP, jointRangeCenter, hapticSpringOrigin;
    jt_type jtSpring;
    double wamSpringDiff;
    std::vector<double> springConstants;
    bool singularBoundary, lastBoundary;
  private:
    DISALLOW_COPY_AND_ASSIGN(JointStopSprings);
  }
  ;

// Joint Velocity Damper System
template<size_t DOF>
  class JVDamper : public systems::SingleIO<typename units::JointVelocities<DOF>::type,
      typename units::JointTorques<DOF>::type>
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    JVDamper(const std::vector<double>& damping, const std::string& sysName = "JVDamper") :
        systems::SingleIO<jv_type, jt_type>(sysName), dampingConstants(damping)
    {
    }

    virtual ~JVDamper()
    {
      this->mandatoryCleanUp();
    }
  protected:

    virtual void operate()
    {
      wamJV = this->input.getValue();

      for (size_t d = 0; d < DOF; d++)
        jtDamping[d] = -dampingConstants[d] * wamJV[d];
      this->outputValue->setData(&jtDamping);
    }

  public:
    jv_type wamJV;
    jt_type jtDamping;
    std::vector<double> dampingConstants;
  private:
    DISALLOW_COPY_AND_ASSIGN(JVDamper);
  }
  ;

// Self Collision Haptic Boundary System
template<size_t DOF>
  class HapticCollisionAvoid : public systems::SingleIO<typename units::CartesianPosition::type,
      typename units::CartesianForce::type>
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    HapticCollisionAvoid(const double& kpIn = 3000, const double& sphereRad = 0.35, const std::string& sysName =
                             "HapticCollisionAvoid") :
        systems::SingleIO<cp_type, cf_type>(sysName), kp(kpIn), sphereRadius(sphereRad)
    {
    }

    virtual ~HapticCollisionAvoid()
    {
      this->mandatoryCleanUp();
    }
  protected:

    virtual void operate()
    {
      wamCP = this->input.getValue();
      collisionCP = wamCP;

      sphereCenter[2] = -0.18;
      collisionVec = collisionCP - sphereCenter; // Make a vector between the WAMs CP and the center of the haptic sphere

      if (collisionVec.norm() < sphereRadius)
        collisionCP = sphereCenter + sphereRadius * collisionVec / collisionVec.norm(); // Puts the position of the spring at the surface of the boundary at the collision point.
      cfHapticCmd = kp * (collisionCP - wamCP);
      this->outputValue->setData(&cfHapticCmd);
    }

  public:
    cp_type wamCP, collisionCP, sphereCenter, collisionVec;
    cf_type cfHapticCmd;
    double kp, sphereRadius;
  private:
    DISALLOW_COPY_AND_ASSIGN(HapticCollisionAvoid);
  }
  ;

#endif /* ROBUST_CARTESIAN_H_ */
