/* ex01_initialize_wam.cpp
 *
 * This is the simplest possible program that runs Barrett Technology's WAM Arm
 * using the libbarrett controls library.
 *
 * Before running this program, make sure the WAM is powered on and connected to
 * the Control PC. It can be either E-stopped or Idled.
 *
 * This code initializes the arm, allows the user to Shift-activate (thereby
 * giving the robot permission to apply torques), and then (with a loop rate of
 * 500 Hz) commands the WAM to apply zero torque to all of its joints. Since the
 * motors aren't applying any torques, it will be easy to move the WAM by
 * pushing it. The arm won't move on its own, but, if it's not already resting
 * on its joint stops, gravity will cause it to fall. The program waits until
 * the user Shift-idles the WAM and then exits.
 *
 * Terminating the program early (for example, by pressing Ctrl-C) will cause a
 * heartbeat fault. Pressing an E-stop button will cause the program to
 * terminate with an unhandled exception. If either of these occur, simply
 * Shift-idle the WAM and it will again be ready to use.
 */


#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

// The file below defines Barrett's standard main() function (SMF). It helps you
// initialize Barrett's products properly without having to repeat the necessary
// code every time you make a new WAM program. See the documentation in
// barrett/standard_main_function.h for more information. The SMF will call the
// wam_main() function below once the WAM is ready to start applying torques.
#include <barrett/standard_main_function.h>


// The root namespace for libbarrett
using namespace barrett;


// In this case, wam_main() is a function template. For a brief introduction to
// C++ templates, see:
//     http://www.devarticles.com/c/a/Cplusplus/An-Introduction-to-C-plus-Templates/
//     http://www.cplusplus.com/doc/tutorial/templates/
// For a more comprehensive (and very well written) introduction to templates
// and C++ in general, see "Thinking in C++, Vol. 1" by Bruce Eckel. It's
// available for free from:
//     http://www.mindview.net/Books/TICPP/ThinkingInCPP2e.html
// The thing to remember is that DOF is a constant. The compiler will generate a
// version of this function where DOF = 7, and a version where DOF = 4. The SMF
// will choose between the two versions based on what kind of robot is present
// when the program is executed.
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {


	// The WAM is now Shift-activated and applying zero torque with a loop-rate
	// of 500Hz.


	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
