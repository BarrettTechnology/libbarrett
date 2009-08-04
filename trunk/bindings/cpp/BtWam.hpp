/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... WamJpos.cpp
 *  Author ............. vw
 *  Creation Date ...... 30 July 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Simple test program for integrating libbt and ROS. This WAM node will
 *  publish and subscribe to Joints topic. 
 *
 * ======================================================================== */

#ifndef BT_WAM_HPP
#define BT_WAM_HPP


#include <string>
#include "WamException.hpp"

   
class Wam
{
private:
   char * name;
   char buf[100];

   /** WAM pointers. wam_local needed for synchronous functions like trimesh */
   struct bt_wam * wam;
   struct bt_wam_local * wam_local;
   
public:
   /** Constructor
   * \param name String name of WAM */
   Wam(char * name);

   /** Destructor */
   virtual ~Wam();

   /** Starts up and establishes connection with WAM
   * \return true if successful, false if unsuccessful */
   void init() throw (WamException);

   /** Gravity comp functions */
   void setGravityCompensation(int onoff);
   int isGravityCompensation();
   
   int dof();
   
   /** Controller functions */
   void idle();
   void hold();
   bool isHolding();
   char * conPosition();
   char * getCurrentControllerName();
   char * getCurrentControllerSpace();
   void controllerToggle();
   void controlUseName(char * name);
   void controlUseSpace(char * space);
   
   /** Refgen functions */
   void refgenClear();
   char * refgenActiveName();
   char * refgenLoadedName();
   void refgenSave(char * filename);
   void refgenLoad(char * filename);
   
   /** Teaching functions */
   bool isTeaching();
   void teachStart() throw (WamException);
   void teachEnd();
   
   /** Play a loaded refgen */
   void playback();

   /** For moves */
   void setVelocity(double vel);
   void setAcceleration(double acc);
/*   void moveTo(int n, double * dest); */
   void moveHome();
   bool moveIsDone();
      
   /** String functions */
   char * getName();
   char * getJointPosition();
   char * getJointVelocity();
   char * getJointTorque();
   char * getCartesianPosition(); //??
   char * getCartesianRotationRow1(); //??
   char * getCartesianRotationRow2(); //??
   char * getCartesianRotationRow3(); //??
   
   /** String to array functions */
   double * getJointPositionDbl();
   double * getJointVelocityDbl();
   double * getJointTorqueDbl();
   
   

   
};


#endif //BT_WAM_HPP
