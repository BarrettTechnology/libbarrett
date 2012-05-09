/* ex03_simple_move.cpp
 *
 * This example expands on the previous one by allowing users to enter target
 * positions that the WAM will move to. Starting at whatever location it happens
 * to be in, the WAM will accelerate smoothly, move at a constant velocity, and
 * then come to a stop at the target location. Once there, it will hold position
 * until the next command.
 *
 * Target positions are specified using "barrett::units". These types are used
 * by libbarrett to distinguish between different kinds of data. For example, a
 * vector containing joint positions is different from a vector containing joint
 * torques because they have different units. barrett::units provide an easy
 * mechanism to increase code clarity by labeling one vector as containing joint
 * positions (barrett::units::JointPositions<DOF>::type) and another vector as
 * containing joint torques (barrett::units::JointTorques<DOF>::type).
 *
 * All of the built-in barrett::units types are actually just specializations of
 * barrett::math::Matrix. math::Matrix is the class template that libbarrett
 * uses to represent vectors and matrices. math::Matrix inherits from
 * Eigen::Matrix, so you can use barrett::units in the same ways you would use
 * an Eigen::Matrix. (There's no runtime penalty for using barrett::units,
 * though there is some additional memory allocated by math::Matrix in order to
 * allow interoperation with the GSL math library.) Check out barrett/units.h to
 * see what units are predefined, and feel free to create your own
 * barrett::units as you see fit!
 */


#include <iostream>
#include <string>
#include <cstdlib>  // For strtod()

// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;


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
		printf("ERROR: Please enter exactly %d numbers separated by "
				"whitespace.\n", dest->size());
	}
}

void printMenu() {
	printf("Commands:\n");
	printf("  j  Enter a joint position destination\n");
	printf("  p  Enter a tool position destination\n");
	printf("  h  Move to the home position\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	// The macro below makes a number of typedefs that allow convenient access
	// to commonly used barrett::units. For example, the typedefs establish
	// "jp_type" as a synonym for "units::JointPositions<DOF>::type". This macro
	// (along with a few others) is defined in barrett/units.h.
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// These vectors are fixed sized, stack allocated, and zero-initialized.
	jp_type jp;  // jp is a DOFx1 column vector of joint positions
	cp_type cp;  // cp is a 3x1 vector representing a Cartesian position


	wam.gravityCompensate();
	printMenu();

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {
		case 'j':
			moveToStr(wam, &jp, "joint positions", line.substr(1));
			break;

		case 'p':
			moveToStr(wam, &cp, "tool position", line.substr(1));
			break;

		case 'h':
			std::cout << "Moving to home position: "
					<< wam.getHomePosition() << std::endl;
			wam.moveHome();
			break;

		case 'i':
			printf("WAM idled.\n");
			wam.idle();
			break;

		case 'q':
		case 'x':
			printf("Quitting.\n");
			going = false;
			break;

		default:
			if (line.size() != 0) {
				printf("Unrecognized option.\n");
				printMenu();
			}
			break;
		}
	}


	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
