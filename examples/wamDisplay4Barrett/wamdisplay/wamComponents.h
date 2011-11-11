/***************************************************************************
 *   Copyright (C) 2007 by Pedro Nobre                                     *
 *   pedrognobre@gmail.com                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


/*! \file wamComponents.h
    \author Pedro Nobre <pedrognobre@gmail.com>

    \brief Geometric primitives to draw several pieces of WAM arm

    \section wamComponets

  *The wamComponents module provides a set o primitives to display several parts of the WAM arm.<BR>
  *These components where divided in such a way that all the movements the robot is capable can be represented.<BR>
  *To completely draw the robot, must be called 8 functions: one to draw the base and 7 functions
  *to draw the links connected to each joint (7 DOF).<BR>
  *The functions must be called sequentially from the base to the end effector obeying the following rules:
  *  -# Position your referential to the desired place. The base will be draw downward viewed from the top.
  *Call base();
  *  -# At the same point rotate the desired 'theta1' along z-axis (to rotate first joint)  and call
  *shoulder_1DOF(GLfloat baseDepth,GLfloat shoulderDepth,GLfloat shoulderWhidth,GLfloat shoulderHeight,GLfloat cavityDepth);
  *  -# Perform a translation along Z axis with the height of WAM arm shoulder height, then rotate 'theta2' along  -Y axis.
  *Call joint_2DOF(GLfloat cavityDepth,GLfloat diam1,GLfloat diam2,GLfloat height);
  *  -# Rotate 'theta3' along Z axis and call link_3DOF(GLfloat diam, GLfloat height, GLfloat elbow);
  *  -# Translate the referential to the elbow position and rotate 'theta4' along -Y.
  *Then call link_4DOF_WRIST(GLfloat diam, GLfloat height, GLfloat elbow);
  *  -# Translate to the 5th rotating joint position and rotate 'theta5' along z axis. Call
  *joint_5DOF_WRIST(GLfloat diam, GLfloat height);
  *  -# Translate to the 6th joint and rotate 'theta 6' along -Y. Call  joint_6DOF_WRIST(GLfloat diam, GLfloat height);
  *  -# Finally translate to the end effector position, rotate 'theta7' along Z and call endPlate(GLfloat diam);
  *
  *The transformations described earlier must be made to fit rotation angles (theta 1,2,...,7)
  *to the ones described in DH parameters in the WAM
  *arm users guide Version AD.00.<BR>
  *The parameters/dimensions passed to the functions must be consistent to the translations to each frame.
  *This way the (n+1) th frame will be correctly placed to the nth frame.
*/


#ifdef __cplusplus
extern "C" {
#endif

#ifndef WAMCOMPONENTS_H
#define WAMCOMPONENTS_H

#include <GL/glut.h>

#define DEG_TO_RAD(DEG)  ((DEG) * M_PI / 180.0)
#define RAD_TO_DEG(RAD)  ((RAD) * 180 / M_PI)

//shared memory values
//struct shared_values{
//  double angle[7];
//  int finish;
//};


/**NOTE: joint positions are called angles here??? */
/**parses string of angles from WAM and stores them in shared variable
param: angle_str- char * of
retval: true- correct parsing; false- incorrect parsing and/or input string */

//static bool get_angles(char * angle_str, int num_angles, double[] d_array);


//mutex declaration to be used throughout
//pthread_mutex_t mutex1;

static const int DOF = 7;


/*! \brief Draw WAM arm base

The (0,0,0) point where this function is called corresponds to top of the wam arm base,
in the center of the cilinder described in the side where thee firt joint will be connected.
*/
void base(
GLfloat width,  //base width   along x-axis
GLfloat height, //base heigth  along z-axis
GLfloat depth   //base depth   along y-axis
);

/*! \brief Draw the WAM arm shoulder

*/
void shoulder_1DOF(
GLfloat baseDepth,        ///< must be equal to depth in function void base(...)
GLfloat shoulderDepth,    ///< should be higher than baseDepth ------ along Y-axis
GLfloat shoulderWhidth,   ///< along ------ X-axis
GLfloat shoulderHeight,   ///< point where the next join will be ploted ------ along Z-axis
GLfloat cavityDepth       ///< size of cavity where the next joint will be inserted allong ------ Y-axis
);

/*! \brief Draw the 2nd joint cylinder and the cylinder that will support the next link

This function draw the 2nd joint(oriented alng Y-axis) and a cylinder perpendicular(along Z-axis) to the first
The (0,0,0) point must be translated "shoulderHeigh" along Z-axis, then the referencial mut be rotated along -Y axis
*/
void joint_2DOF(
GLfloat cavityDepth,      ///< value equal to the one in previous function along ------ Y-axis
GLfloat diam1,            ///< diameter of the first cylinder representing the 2nd joint
GLfloat diam2,            ///< diameter of the second cylinder where the next link will be inserted
GLfloat height            ///< this is the value desired to the 2nd cylinder heigth ------ along Z-axis
);

/*! \brief Draw the link connected to the 3rd joint and the external part of the elbow

*/
void link_3DOF(
GLfloat diam,             ///< link diameter
GLfloat height,           ///< distance from the 3rd joint to the elbow ------ along Z-axis
GLfloat elbow             ///< elbow displacement relative to link axis ------ along X-axis
);

/*! \brief Draw the link connected to the 4th joint and the inner part of the elbow

*/
void link_4DOF_WRIST(
GLfloat diam,            ///< link diameter
GLfloat height,          ///< distance from the 3rd to the 4th joint along z
GLfloat elbow            ///< elbow displacement relative to link axis ------ along X-axis
);

/*! \brief Draw connection between 5th anh 6th joint

*/
void joint_5DOF_WRIST(
GLfloat diam,            ///< link diameter
GLfloat height           ///< distance from the 5th joint 6th joint axis ----- along Z
);

/*! \brief Connection between 6th joint and end plate

*/
void joint_6DOF_WRIST(
GLfloat diam,           ///< link diameter
GLfloat height          ///< distance from 6th joint to the end plate ----- along Z
);

/*! \brief Draw the end plate of the WAM arm

*/
void endPlate(GLfloat diam);

void referencial();

void box();
#endif

 #ifdef __cplusplus
 }
 #endif
