/*
 * moveToPose.cpp
 *
 *  Created on: May 7, 2019
 *      Author: ab
 */

#include <iostream>
#include <cassert>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <Eigen/Geometry>

#include <barrett/systems.h>
#include <barrett/math.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

#include <barrett/detail/stl_utils.h>


using namespace barrett;
using detail::waitForEnter;

void printMenu() {
  printf("Commands:\n");
  printf("  p  Print current tool pose\n");
  printf("  t  Enter a tool pose destination and move to that pose\n");
  printf("  r  Record the current pose\n");
  printf("  m  Move to recorded pose\n");
  printf("  h  Move to the home position\n");
  printf("  i  Idle (release position/orientation constraints)\n");
  printf("  ?  Print this menu\n");
  printf("  q  Quit\n");
}

bool parsePose(boost::tuple<math::Matrix<3,1,units::CartesianPosition>,
                            Eigen::Quaterniond>* dest,
               const std::string& str) {
  const char* cur = str.c_str();
  const char* next = cur;

  // Parse position
  for (int i = 0; i < 3; ++i) {
    (boost::get<0>(*dest))[i] = strtod(cur, (char**) &next);
    if (cur == next) {
      return false;
    } else {
      cur = next;
    }
  }

  // Parse orientation
  (boost::get<1>(*dest)).w() = strtod(cur, (char**) &next);
  if (cur == next) {
    return false;
  } else {
    cur = next;
  }
  for (int i = 0; i < 3; ++i) {
    (boost::get<1>(*dest)).vec()[i] = strtod(cur, (char**) &next);
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

void printPose(boost::tuple<math::Matrix<3,1,units::CartesianPosition>,
                            Eigen::Quaterniond> p) {
  std::cout << "Moving to pose: " << boost::get<0>(p) << ", [" <<
    boost::get<1>(p).w();
  for (int i = 0; i < 3; i++) {
    std::cout << ", " << boost::get<1>(p).vec()[i];
  }
  std::cout << "]" << std::endl;
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF); 
  pose_type p1;
  pose_type p2;
  pose_type p3;
  bool recorded = false;

  wam.gravityCompensate();
  printMenu();

  std::string line;
  bool going = true;
  while (going && (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)) {
    printf(">>> ");
    std::getline(std::cin, line);

    switch (line[0]) {
    case 'p':
      p3 = wam.getToolPose();
      std::cout << "Current pose is: " << boost::get<0>(p3);
      std::cout << ", [" << boost::get<1>(p3).w();
      std::cout << ", " << boost::get<1>(p3).vec()[0] << "i";
      std::cout << ", " << boost::get<1>(p3).vec()[1] << "j";
      std::cout << ", " << boost::get<1>(p3).vec()[2] << "k]";
      std::cout << std::endl;
      break;

    case 't':
      if (parsePose(&p2, line.substr(1))) {
        printPose(p2);
        wam.idle();
        wam.moveTo(p2);
      } else {
        std::cout << "ERROR: Please enter exactly 7 numbers separated by " <<
          "whitespace. (3 for position vector, 4 for orientation quaternion " <<
          "[w, x, y, z])" << std::endl;
      }
      break;

    case 'r':
      p1 = wam.getToolPose();
      recorded = true;
      break;

    case 'm':
      if (recorded) {
        printPose(p1);
        wam.idle();
        wam.moveTo(p1);
      } else {
        std::cout << "Press 'r' to record a position." << std::endl;
      }
      break;

    case 'h':
      std::cout << "Moving to home position: "
          << wam.getHomePosition() << std::endl;
      wam.idle();
      wam.moveHome();
      break;

    case 'i':
      std::cout << "WAM idled." << std::endl;
      wam.idle();
      break;

    case '?':
      printMenu();
      break;

    case 'q':
    case 'x':
      going = false;
      break;

    default:
      if (line.size() != 0) {
        std::cout << "Unrecognized option." << std::endl;
        printMenu();
      }
      break;
    }
  }

  printf("Quitting.\n");
  if (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
    wam.moveHome();
    wam.idle();
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
  }
  return 0;
}
