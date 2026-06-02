#include <cstdio>
#include <cassert>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <curses.h>

#include <barrett/log.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/config.h>
#include "../programs/utils.h"

#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>


using namespace barrett;
using systems::connect;

const std::string CAL_CONFIG_FILE = barrett::EtcPathRelative("calibration.conf");

// Index pulses are 15 degrees away from the standard home position...
const double OFFSET = 15 * M_PI/180.0;
// ... plus or minus 10 degrees.
const double TOLERANCE = 10 * M_PI/180.0;

// Currently JE's are only available for the first 4 DOF
const size_t NUM_JOINT_ENCODERS = 4;

// Convenience class that wraps ncurses text attributes.
class ScopedAttr {
public:
  ScopedAttr() : isOn(false), a(-1) {}
  explicit ScopedAttr(unsigned long attr, bool startOn = false) : isOn(false), a(-1) {
    set(attr, startOn);
  }
  ~ScopedAttr() { off(); }

  void set(unsigned long attr, bool startOn = true) {
    if (isOn) {
      off();
    }
    a = attr;
    if (startOn) {
      on();
    }
  }
  void on() {
    if (!isOn) {
      attron(a);
      isOn = true;
    }
  }
  void off() {
    if (isOn) {
      attroff(a);
      isOn = false;
    }
  }

protected:
  bool isOn;
  unsigned long a;
};


// Represents a generic step in the calibration process.
// Handles displaying a menu item and provides hooks for keyboard interactions
// and state transitions. Can be "selected" and/or "active".
class CalibrationStep {
public:
  explicit CalibrationStep(const std::string& title_ = "") :
    selected(false), active(false), title(title_) {}
  virtual ~CalibrationStep() {}

  void setSelected(bool on) { selected = on; }
  void setActive(bool on) {
    if (active != on) {
      active = on;
      onChangeActive();
    }
  }

  virtual void onChangeActive() {}
  virtual bool onKeyPress(enum Key k) {
    return true;
  }

  static const int L_OFFSET = 10;

  // Returns the number of lines that were displayed
  virtual int display(int top, int left) {
    ScopedAttr sa(A_BOLD);
    if (selected) {
      sa.on();
      mvprintw(top,left, "--> ] ");
    } else {
      mvprintw(top,left, "    ] ");
    }
    printw("%s", title.c_str());

    return height(top);
  }

  // Helper for calculating display heights
  static int height(int origTop) {
    return getcury(stdscr) - origTop + 1;
  }

protected:
  bool selected, active;
  std::string title;
};

// Handles actions an displays associated with adjusting the angle of a
// particular joint.
template<size_t DOF>
class AdjustJointStep : public CalibrationStep {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  static const int DEFAULT_DIGIT = -2;  // Start by adjusting the hundredth's place.
  static const int MAX_DIGIT = 0;  // Display 1's...
  static const int MIN_DIGIT = -3;  // ... through thousandth's.

public:
  explicit AdjustJointStep(const libconfig::Setting& setting, systems::Wam<DOF>* wamPtr, jp_type* calOffsetPtr, jp_type* zeroPosPtr, v_type* zeroAnglePtr) :
    CalibrationStep("Joint " + boost::lexical_cast<std::string>((int)setting[0])),
    j((int)setting[0] - 1), calPos(setting[1]), endCondition(setting[2].c_str()),
    wam(*wamPtr), calOffset(*calOffsetPtr), zeroPos(*zeroPosPtr), zeroAngle(*zeroAnglePtr),
    state(0), digit(DEFAULT_DIGIT)
  {
    assert(j >= 0);
    assert(j < DOF);
    assert(wamPtr != NULL);
    assert(calOffsetPtr != NULL);
    assert(zeroPosPtr != NULL);
    assert(zeroAnglePtr != NULL);
  }
  virtual ~AdjustJointStep() {}


  virtual void onChangeActive() {
    if (active) {
      // Reset
      state = 0;
      digit = DEFAULT_DIGIT;
    }
  }

  virtual bool onKeyPress(enum Key k) {
    switch (state) {
    // Prompt to move the WAM
    case 0:
      if (k == K_ENTER) {
        state++;
        move();
      }
      break;

    // Wait until the move is done
    case 1:
      // This transition is handled in display().
      break;

    // Adjust the offset
    default:
      switch (k) {
      case K_LEFT:
        digit = std::min(0 /*MAX_DIGIT*/, digit+1);
        break;
      case K_RIGHT:
        digit = std::max(-3 /*MIN_DIGIT*/, digit-1);
        break;

      case K_UP:
        calOffset[j] += std::pow(10.0, digit);
        move();
        break;
      case K_DOWN:
        calOffset[j] -= std::pow(10.0, digit);
        move();
        break;

      case K_ENTER:
        if (wam.moveIsDone()) {
          LowLevelWam<DOF>& llw = wam.getLowLevelWam();

          // Record actual joint position, not commanded joint position
          zeroPos[j] = wam.getJointPositions()[j];

          // Record the motor angles that affect this joint
          const sqm_type& m2jp = llw.getMotorToJointPositionTransform();
          double tolerance = m2jp.cwiseAbs().maxCoeff() * 1e-5;
          for (size_t i = 0; i < DOF; ++i) {
            // If the j,i entry is non-zero, then Motor i is in some way connected to Joint j
            if (math::abs(m2jp(j,i)) > tolerance) {
              int mech = llw.getPucks()[i]->getProperty(Puck::MECH);
              zeroAngle[i] = llw.getMotorPucks()[i].counts2rad(mech);
            }
          }

          // Log joint encoder data for factory calibration
          if (llw.hasJointEncoders()) {
            logMessage("Joint encoder angles: %s") % llw.getJointPositions(LowLevelWam<DOF>::PS_JOINT_ENCODER);
          }

          return false;  // Move on to the next joint!
        }
        break;

      default:
        break;
      }
    }

    return true;
  }

  virtual int display(int top, int left) {
    int line = top + CalibrationStep::display(top, left);  // Call super
    left += L_OFFSET;

    if (active) {
      ScopedAttr sa(A_STANDOUT, true);

      switch (state) {
      // Prompt to move the WAM
      case 0:
        mvprintw(line++,left, "Press [Enter] to move to: [%0.2f", calPos[0]);
        for (size_t i = 1; i < DOF; ++i) {
          printw(", %04.2f", calPos[i]);
        }
        printw("]");
        break;

      // Wait until the move is done
      case 1:
        mvprintw(line++,left, "Wait for the WAM to finish moving.");

        if (wam.moveIsDone()) {  // Have we arrived yet?
          state++;
        }
        break;

      // Adjust the offset
      default:
        mvprintw(line++,left, "Use arrow keys to adjust the Joint %d offset.", j+1);
        sa.off();
        line++;
        mvprintw(line++,left, "                 ");
        attron(A_UNDERLINE);
        printw("Offset (rad)");
        attroff(A_UNDERLINE);
        mvprintw(  line,left, "                    %+06.3f", calOffset[j]);

        int x = getcurx(stdscr);
        mvchgat(line, x-5-digit + ((digit<0) ? 1:0), 1, A_STANDOUT, 0, NULL);

        line += 2;
        mvprintw(line++,left, "Press [Enter] once %s.", endCondition.c_str());
        break;
      }
    }

    return height(top);
  }

protected:
  void move() {
    while ( !wam.moveIsDone() ) {
      btsleep(0.02);
    }
    wam.moveTo(jp_type(calPos + calOffset), false);
  }

  size_t j;
  const jp_type calPos;
  std::string endCondition;

  systems::Wam<DOF>& wam;
  jp_type& calOffset;
  jp_type& zeroPos;
  v_type& zeroAngle;

  int state, digit;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// The Exit menu item.
class ExitStep : public CalibrationStep {
public:
  explicit ExitStep(const std::string& title_ = "Exit") : CalibrationStep(title_) {}
  virtual ~ExitStep() {}

  virtual bool onKeyPress(enum Key k) {
    return k != K_ENTER;
  }

  virtual int display(int top, int left) {
    int line = top + CalibrationStep::display(top, left);  // Call super
    left += L_OFFSET;

    if (active) {
      ScopedAttr sa(A_STANDOUT, true);
      mvprintw(line++,left, "Press [Enter] to move the home position and record calibration data.");
    }

    return height(top);
  }
};

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
  uint ecmin;
  uint ecmax;
  for (size_t i = 0; i < pm->getWamPucks().size(); ++i) {
    std::cout << "Checking puck " << i+1 << std::endl;
    if (pm->getWamPucks()[i] != NULL) {
      ecmin = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMIN));
      ecmax = abs(pm->getWamPucks()[i]->getProperty(Puck::ECMAX));
      std::cout << "ECMIN: " << std::setw (10) << ecmin << std::endl;
      std::cout << "ECMAX: " << std::setw (10) << ecmax << std::endl;
      ///* bypassing encoder check
      if ((ecmin > bound) ||
          (ecmax > bound)) {
        return false;
      }
      //*/
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
    "   l     calibrate" << std::endl <<
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
void je_checkout(systems::Wam<DOF>& wam, ProductManager* pm, math::Matrix<R,C, Units>* farSideOfIndexes, const std::string& path) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  printf("Setting up reach...");
  jp_type reach(0.0);
  reach[0] = -2.7;
  reach[1] = 1.8;
  reach[2] = -2.0;
  reach[3] = -0.5;

  printf("Setting up loops & tracker arrays...");
  unsigned int LOOPS = 20;
  int start_ecmax[4] = {0, 0, 0, 0};
  int start_ecmin[4] = {0, 0, 0, 0};
  jp_type je_me_error;

  printf("Setting up temp file for logging...");
  // setup temp file for logging
  char tmpFile[] = "/tmp/btXXXXXX";
  if (mkstemp(tmpFile) == -1) {
    printf("ERROR: Couldn't create temporary file!\n");
    return;
  }

  printf("Setting up time system...");
  systems::Ramp time(pm->getExecutionManager(), 1.0);

  printf("Setting up joint encoder reader...");
  JeReader<DOF> jer(&wam);
  // The JeReader doesn't use the timer input for anything, however JeReader
  // inherets from SingleIO and SingleIO objects don't run unless they
  // are regularly seeing inputs.
  connect(time.output, jer.input);

  printf("Setting up logger...");
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
  wam.moveTo(*farSideOfIndexes);
  wam.moveHome();
  wam.moveTo(*farSideOfIndexes);

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
    wam.moveTo(*farSideOfIndexes);
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
int je_calibration(systems::Wam<DOF>& wam, ProductManager* pm) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	jp_type calOffset(0.0);  // The offsets as entered by the user
	jp_type zeroPos(0.0);  // Zero position angles in joint space
	v_type zeroAngle(0.0);  // Motor positions that correspond to zeroPos

	// Disable torque saturation because gravity compensation is off
  wam.jpController.setControlSignalLimit(jp_type());
	wam.moveTo(wam.getJointPositions());  // Hold position

  	// Read from config
	std::vector<CalibrationStep*> steps;
	try {
		libconfig::Config config;
		config.readFile(CAL_CONFIG_FILE.c_str());
		const libconfig::Setting& setting = config.lookup("zerocal")[pm->getWamDefaultConfigPath()];
    const unsigned int JE_DOF = 4; // number of joints with joint encoders
		assert(setting.isList());
		assert(setting.getLength() == (int)DOF);

    // Only the first four joints need JE calibration, since we dont support
    // joint encoders at joint 5 or above (circa 2024)
		for (size_t i = 0; i < JE_DOF; ++i) {
			// Populate the menu
			steps.push_back(new AdjustJointStep<DOF>(setting[i], &wam, &calOffset, &zeroPos, &zeroAngle));
		}
	} catch (libconfig::ParseException e) {
		printf(">>> CONFIG FILE ERROR on line %d of %s: \"%s\"\n", e.getLine(), CAL_CONFIG_FILE.c_str(), e.getError());
		return 1;
	} catch (libconfig::SettingNotFoundException e) {
		printf(">>> CONFIG FILE ERROR in %s: could not find \"%s\"\n", CAL_CONFIG_FILE.c_str(), e.getPath());
		return 1;
	} catch (libconfig::SettingTypeException e) {
		printf(">>> CONFIG FILE ERROR in %s: \"%s\" is the wrong type\n", CAL_CONFIG_FILE.c_str(), e.getPath());
		return 1;
	}
	steps.push_back(new ExitStep);


	// Set up ncurses
	initscr();
	curs_set(0);
	noecho();
	timeout(0);


	// Display loop
	assert(steps.size() >= 1);
	int selected = 0;
	steps[selected]->setSelected(true);
	int active = 0;
	steps[active]->setActive(true);
	int maxStep = 0;

	enum Key k;
	bool done = false;
	while (pm->getSafetyModule()->getMode() == SafetyModule::ACTIVE  &&  !done) {
		k = getKey();
		if (active == -1) {  // Menu mode
			switch (k) {
			case K_UP:
				steps[selected]->setSelected(false);
				selected = std::max(0, selected-1);
				steps[selected]->setSelected(true);
				break;
			case K_DOWN:
				steps[selected]->setSelected(false);
				selected = std::min(maxStep, selected+1);
				steps[selected]->setSelected(true);
				break;
			case K_ENTER:
				active = selected;
				steps[active]->setActive(true);
				break;
			default:
				break;
			}
		} else {  // Adjust mode
			switch (k) {
			case K_BACKSPACE:
			case K_ESCAPE:
			case K_TAB:
				steps[active]->setActive(false);
				active = -1;
				break;
			default:
				// Forward to the active item by calling onKeyPress()
				if ( !steps[active]->onKeyPress(k) ) {
					// This step is done. Move to the next one.
					steps[selected]->setSelected(false);
					steps[active]->setActive(false);
					++selected;
					if (selected > maxStep)
						maxStep = selected;
					if (selected < (int)steps.size()) {
						active = selected;
						steps[selected]->setSelected(true);
						steps[active]->setActive(true);
					} else {  // Finished the last step!
						done = true;
					}
				}
				break;
			}
		}

		// Display
		clear();
		mvprintw(0,18, "*** Barrett WAM Joint Encoder Calibration Utility ***");
		int top = 3;
		for (size_t i = 0; i < steps.size(); ++i) {
			top += steps[i]->display(top,0);
		}
		refresh();

		btsleep(0.1);  // Slow loop down to ~10Hz
	}
	endwin();  // Turn off ncurses
	detail::purge(steps);


	// Restore stdout's line buffering
	if (freopen(NULL, "w", stdout) == NULL) {
		fprintf(stderr, ">>> ERROR: freopen(stdout) failed.\n");
		return 1;
	}

	if (done) {
		printf(">>> Calibration completed!\n");

		// Move home!
		printf(">>> Moving back to home position.\n");
		wam.moveHome();
	} else {
		printf(">>> ERROR: WAM was Idled before the calibration was completed.\n");
		return 1;
	}


	pm->getSafetyModule()->waitForMode(SafetyModule::IDLE);
	// Make sure the user applies their new calibration
	pm->getSafetyModule()->setWamZeroed(false);

	return 0;
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

    std::cout << line[0] << std::endl;

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
        je_checkout(wam, &pm, &farSideOfIndexes, line.substr(2));
        break;

      case 'l':
        je_calibration(wam, &pm);
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
