#include <cstdio>
#include <cassert>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/log.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/units.h>

#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>


using namespace barrett;
using systems::connect;


// Index pulses are 15 degrees away from the standard home position...
const double OFFSET = 15 * M_PI/180.0;
// ... plus or minus 10 degrees.
const double TOLERANCE = 10 * M_PI/180.0;

// Currently JE's are only available for the first 4 DOF
const size_t NUM_JOINT_ENCODERS = 4;

// Check if joint encoders have been initialized.
template<size_t DOF>
bool jointEncodersInitialized(systems::Wam<DOF> *wam) {
  bool initialized = true;
  for (size_t i = 0; i < NUM_JOINT_ENCODERS; ++i) {
    initialized &= wam->getLowLevelWam().getMotorPucks()[i].foundIndexPulse();
  }
  return initialized;
}

// Reset encoder correction values (ECMIN and ECMAX).
void resetEncoderCorrections(ProductManager *pm) {
  for (size_t i = 0; i < pm->getWamPucks().size(); ++i) {
    if (pm->getWamPucks()[i] != NULL) {
      pm->getWamPucks()[i]->setProperty(Puck::ECMIN, 0);
      pm->getWamPucks()[i]->setProperty(Puck::ECMAX, 0);
    }
  }
}

// Check if ECMIN and ECMAX are within reasonable bounds. Typically we expect
// them to be around 35-45, but 200 is a conservative estimate.
bool checkEncoderCorrections(ProductManager *pm, uint bound = 200) {
  int ecmin;
  int ecmax;
  for (size_t i = 0; i < pm->getWamPucks().size(); ++i) {
    std::cout << "Checking puck " << i+1 << std::endl;
    if (pm->getWamPucks()[i] != NULL) {
      ecmin = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMIN));
      ecmax = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMAX));
      std::cout << "ECMIN: " << std::setw (10) << ecmin << std::endl;
      std::cout << "ECMAX: " << std::setw (10) << ecmax << std::endl;
      if ((ecmin > bound) ||
          (ecmax > bound)) {
        return false;
      }
    }
  }
  return true;
}

void printMenu() {
  std::cout << "Commands:" << std::endl <<
    "<Enter>  print Cartesian position info" << std::endl <<
    "   j     print joint position info" << std::endl <<
    "   a     activate/deactivate joint encoders" << std::endl <<
    "   h     hold joint positions" << std::endl <<
    "   g     followed by DOF numbers to go to joint positions" << std::endl <<
    "   i     idle" << std::endl <<
    "   c     checkout" << std::endl <<
    "   q     quit" << std::endl;
}

template <size_t DOF>
void printCartPosStats(systems::Wam<DOF>& wam) {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
  const int kSamples = 50;
  std::vector<cp_type, Eigen::aligned_allocator<cp_type> > points;

  // Take a bunch of position readings and calculate the mean
  cp_type mean;
  mean << 0.0, 0.0, 0.0;
  for (int i = 0; i < kSamples; i++) {
    cp_type point = wam.getToolPosition();
    mean += point;
    points.push_back(point);
  }
  mean /= kSamples;

  // Compute the standard deviation
  cp_type std;
  std << 0.0, 0.0, 0.0;
  for (int i = 0; i < 3; i++) {
    for (size_t j = 0; j < points.size(); j++) {
      double err = points[j][i] - mean[i];
      std[i] += err * err;
    }
    std[i] = pow(std[i] / (kSamples - 1), 0.5);
  }

  std::string enc = (wam.getLowLevelWam().getPositionSensor() ==
      LowLevelWam<DOF>::PS_MOTOR_ENCODER) ? "M" : "J";
  std::cout << "CP (" << enc << " enc): " << mean << " +/- " << std << " (n = " << kSamples << ")"
    << std::endl;
}

// This function template will accept a math::Matrix with any number of rows,
// any number of columns, and any units. In other words: it will accept any
// barrett::units type.
template<int R, int C, typename Units>
bool parseDoubles(math::Matrix<R,C, Units>* dest, const std::string& str) {
  const char* cur = str.c_str();
  const char* next = cur;

  for (int i = 0; i < dest->size(); ++i) {
    (*dest)[i] = strtod(cur, (char**) &next);
    if (cur == next) {
      return false;
    } else {
      cur = next;
    }
  }

  // Make sure there are no extra numbers in the string.
  double ignore = strtod(cur, (char**) &next);
  (void)ignore;  // Prevent unused variable warnings

  if (cur != next) {
    return false;
  }

  return true;
}

  template<size_t DOF, int R, int C, typename Units>
void moveToStr(systems::Wam<DOF>& wam, math::Matrix<R,C, Units>* dest,
    const std::string& description, const std::string& str)
{
  if (parseDoubles(dest, str)) {
    std::cout << "Moving to " << description << ": " << *dest << std::endl;
    wam.moveTo(*dest);
  } else {
    std::cout << "ERROR: Please enter exactly " << DOF <<
      " numbers separated by whitespace." << std::endl;
  }
}

// A system class that outputs the joint position as calculated from the joint
// encoders, even if the robot is using its motor encoders for position.
// This shouldn't really inherit from SingleIO because it doesn't use its
// input, but I couldn't figure out how to get this to work as a pure output.
template <size_t DOF>
class JeReader : public systems::SingleIO<double, typename units::JointPositions<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
  JeReader(systems::Wam<DOF>* wam_, const std::string& sysName = "JeReader") :
    systems::SingleIO<double, jp_type>(sysName), wam(wam_), jp_from_jenc(0.0) {

    }

  virtual ~JeReader() { this->mandatoryCleanUp(); }

  protected:

  systems::Wam<DOF>* wam;
  jp_type jp_from_jenc;

  virtual void operate() {
    jp_from_jenc = wam->getLowLevelWam().getJointPositions(LowLevelWam<DOF>::PS_JOINT_ENCODER);
    this->outputValue->setData(&jp_from_jenc);
  }

  private:
  DISALLOW_COPY_AND_ASSIGN(JeReader);
};

template<size_t DOF, int R, int C, typename Units>
void je_checkout(systems::Wam<DOF>& wam, ProductManager* pm, jp_type farSideOfIndexes, const std::string& path) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  jp_type reach(0.0);
  reach[0] = -2.7;
  reach[1] = 1.8;
  reach[2] = -2.0;
  reach[3] = -0.5;

  const int LOOPS = 20;
  int start_ecmax[4] = {0, 0, 0, 0};
  int start_ecmin[4] = {0, 0, 0, 0};
  jp_type je_me_error;

  // setup temp file for logging
  char tmpFile[] = "/tmp/btXXXXXX";
  if (mkstemp(tmpFile) == -1) {
    printf("ERROR: Couldn't create temporary file!\n");
    return;
  }

  systems::Ramp time(pm->getExecutionManager(), 1.0);

  JeReader<DOF> jer(&wam);
  // The JeReader doesn't use the timer input for anything, however JeReader
  // inherets from SingleIO and SingleIO objects don't run unless they
  // are regularly seeing inputs.
  //connect(time.output, jer.input);

  systems::TupleGrouper<double, jp_type, jp_type> tg;
  connect(time.output, tg.template getInput<0>());
  connect(wam.jpOutput, tg.template getInput<1>());
  connect(jer.output, tg.template getInput<2>());

  typedef boost::tuple<double, jp_type, jp_type> tuple_type;
  const size_t PERIOD_MULTIPLIER = 1;
  systems::PeriodicDataLogger<tuple_type> logger(
      pm->getExecutionManager(),
      new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm->getExecutionManager()->getPeriod()),
      PERIOD_MULTIPLIER);

  // re-find all the indexes
  std::cout << "Clearing and re-initializing encoder error..." << std::endl;
  std::cout << "Moving home..." << std::endl;
  wam.moveHome();
  std::cout << "Resetting ECMAX and ECMIN..." << std::endl;
  resetEncoderCorrections(pm);
  std::cout << "Finding indexes..." << std::endl;
  wam.moveTo(farSideOfIndexes);
  wam.moveHome();
  wam.moveTo(farSideOfIndexes);

  // get info
  je_me_error = wam.getLowLevelWam().getJointPositions(LowLevelWam<DOF>::PS_JOINT_ENCODER) - wam.getLowLevelWam().getJointPositions(LowLevelWam<DOF>::PS_MOTOR_ENCODER);
  std::cout << "JE-ME Error:" << je_me_error << std::endl << std::endl;
  for (size_t i = 0; i < 4; ++i) {
    if (pm->getWamPucks()[i] != NULL) {
      start_ecmin[i] = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMIN));
      start_ecmax[i] = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMAX));
      std::cout << "Puck " << i+1 << ":\nECMIN:" << std::setw (10) << start_ecmin[i] << std::endl;
      std::cout << "ECMAX:" << std::setw (10) << start_ecmax[i] << std::endl;
    }
  }

  time.start();
  connect(tg.output, logger.input);
  std::cout << "Logging started..." << std::endl;

  // accumulate error
  for (size_t i = 0; i < LOOPS; ++i) {
    std::cout << "Cycle " << i+1 << " of " << LOOPS << std::endl;
    wam.moveTo(reach);
    wam.moveTo(farSideOfIndexes);
  }
  std::cout << "Done accumulating..." << std::endl;

  logger.closeLog();
  std::cout << "Logging stopped..." << std::endl;

  log::Reader<tuple_type> lr(tmpFile);
  lr.exportCSV(path.c_str());
  std::cout << "Output written to " << path << std::endl;
  std::remove(tmpFile);

  je_me_error = wam.getLowLevelWam().getJointPositions(LowLevelWam<DOF>::PS_JOINT_ENCODER) - wam.getLowLevelWam().getJointPositions(LowLevelWam<DOF>::PS_MOTOR_ENCODER);
  std::cout << "JE-ME Error:" << je_me_error << std::endl << std::endl;
  wam.moveHome();
  for (size_t i = 0; i < 4; ++i) {
    if (pm->getWamPucks()[i] != NULL) {
      start_ecmin[i] = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMIN));
      start_ecmax[i] = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMAX));
      std::cout << "Puck " << i+1 << ":\nECMIN:" << std::setw (10) << start_ecmin[i] << std::endl;
      std::cout << "ECMAX:" << std::setw (10) << start_ecmax[i] << std::endl;
    }
  }

  std::cout << "Test finished. Idling..." << std::endl;
  wam.idle();
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  if (!wam.getLowLevelWam().hasJointEncoders()) {
    std::cout << "ERROR: Joint encoders not found." << std::endl;
    return 10;
  }

  // Otherwise the "farSideOfIndexes" position might cause self-collisions.
  assert(TOLERANCE < OFFSET);

  // Define the positions at which we expect to find the index pulses for each
  // joint encoder.
  jp_type indexPositions(0.0);
  indexPositions[0] = 0.0 - OFFSET;
  indexPositions[1] = -2.0 + OFFSET;
  indexPositions[2] = 0.0 - OFFSET;
  indexPositions[3] = M_PI - OFFSET;

  jp_type jp = wam.getJointPositions();
  v_type dir = math::sign(indexPositions - jp);
  jp_type farSideOfIndexes = indexPositions + TOLERANCE * dir;

  // The wrist doesn't need to move.
  for (size_t i = NUM_JOINT_ENCODERS; i < DOF; ++i) {
    farSideOfIndexes[i] = jp[i];
  }

  pm.getSafetyModule()->waitForMode(SafetyModule::ACTIVE);
  wam.gravityCompensate();

  btsleep(0.5);

  // If not already initialized, move through the init sequence once.
  if (!jointEncodersInitialized(&wam)) {
    wam.moveTo(farSideOfIndexes);
    wam.moveHome();
  }

  // Then check the values of ECMIN and ECMAX. The first time the index pulse is
  // found, these are usually set to large values. If that is the case, they
  // should be reset and done again.
  if (!checkEncoderCorrections(&pm)) {
    resetEncoderCorrections(&pm);
    wam.moveTo(farSideOfIndexes);
    wam.moveHome();
  }

  if (!checkEncoderCorrections(&pm)) {
    resetEncoderCorrections(&pm);
    wam.moveTo(farSideOfIndexes);
    wam.moveHome();
  }

  if (!checkEncoderCorrections(&pm)) {
    resetEncoderCorrections(&pm);
    wam.moveTo(farSideOfIndexes);
    wam.moveHome();
  }

  if (!checkEncoderCorrections(&pm)) {
    resetEncoderCorrections(&pm);
    wam.moveTo(farSideOfIndexes);
    wam.moveHome();
  }

  // If ECMIN and ECMAX are still not within reasonable bounds, this suggests a
  // hardware problem.
  if (!checkEncoderCorrections(&pm)) {
    std::cout << "ERROR: Joint encoder initialization failed." << std::endl;
    return 11;
  }

  // Print out positions at zero in case they are needed for calibration.
  jp_type zero(0.0);
  wam.moveTo(zero);
  std::cout << "JP (joint encoders): " <<
    wam.getLowLevelWam().getJointPositions(LowLevelWam<DOF>::PS_JOINT_ENCODER)
    << std::endl;
  std::cout << "JP (motor encoders): " <<
    wam.getLowLevelWam().getJointPositions(LowLevelWam<DOF>::PS_MOTOR_ENCODER)
    << std::endl;

  std::cout << "Press <Enter> to continue." << std::endl;
  barrett::detail::waitForEnter();

  wam.moveHome();
  wam.idle();

  // Start main loop. This is for testing joint encoder accuracy. Options are
  // print out positions based on joint encoders and motor encoders, or quit.
  printMenu();

  std::string line;
  bool going = true;
  while (going) {
    printf(">>> ");
    std::getline(std::cin, line);

    jp_type jp_from_jenc;
    jp_type jp_from_menc;
    jp_type diff;
    double diff_norm;

    switch (line[0]) {
      case 'h':
        wam.moveTo(wam.getJointPositions());
        break;
      case 'i':
        wam.idle();
        break;
      case 'g':
        moveToStr(wam, &jp, "joint positions", line.substr(1));
        break;
      case 'a':
        jp_from_jenc = wam.getLowLevelWam().getJointPositions(
            LowLevelWam<DOF>::PS_JOINT_ENCODER);
        jp_from_menc = wam.getLowLevelWam().getJointPositions(
            LowLevelWam<DOF>::PS_MOTOR_ENCODER);
        diff = jp_from_jenc - jp_from_menc;
        diff_norm = 0;
        // only joints 1-4 have joint encoders
        for (size_t i = 0; i < 4; i++) {
          diff_norm += pow(fabs(diff[i]), 2);
        }
        diff_norm = pow(diff_norm, 0.5);
        if (diff_norm > 0.1) {
          std::cout << "WARNING: diff norm between joint and motor encoder is " <<
            "large (" << diff_norm << "). Not switching encoder control."
            << std::endl;
        } else if (wam.getLowLevelWam().getPositionSensor() ==
            LowLevelWam<DOF>::PS_MOTOR_ENCODER) {
          wam.idle();
          wam.getLowLevelWam().setPositionSensor(
              LowLevelWam<DOF>::PS_JOINT_ENCODER);
          std::cout << "Now using joint encoders." << std::endl;
        } else if (wam.getLowLevelWam().getPositionSensor() ==
            LowLevelWam<DOF>::PS_JOINT_ENCODER) {
          wam.idle();
          wam.getLowLevelWam().setPositionSensor(
              LowLevelWam<DOF>::PS_MOTOR_ENCODER);
          std::cout << "Now using motor encoders." << std::endl;
        } else {
          // This should be impossible.
          std::cout << "ERROR: Invalid Position Sensor setting detected." <<
            std::endl;
        }
        break;
      case 'q':
      case 'x':
        printf("Quitting.\n");
        wam.moveHome();
        going = false;
        break;

      case 'j':
        jp_from_jenc = wam.getLowLevelWam().getJointPositions(
            LowLevelWam<DOF>::PS_JOINT_ENCODER);
        jp_from_menc = wam.getLowLevelWam().getJointPositions(
            LowLevelWam<DOF>::PS_MOTOR_ENCODER);
        std::cout << "JP (joint encoders): " << jp_from_jenc << std::endl;
        std::cout << "JP (motor encoders): " << jp_from_menc << std::endl;
        std::cout << "JP (active): " << wam.getJointPositions() << std::endl;
        break;

      case 'c':
        je_checkout(wam, &pm, farSideOfIndexes, line.substr(2));
        break;

      default:
        if (line.size() == 0) {
          printCartPosStats(wam);
        } else {
          printMenu();
        }
        break;
    }
  }
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

  return 0;
}
