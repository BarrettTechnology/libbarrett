/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... WamJpos.cpp
 *  Author ............. vw
 *  Creation Date ...... 30 July 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  This program demonstrates simple sending of messages over the ROS system 
 *  integrated withlibbt.
 *
 * ======================================================================== */


#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	/* Initializes ROS system. wam_listener is name of node */
	ros::init(argc, argv, "wam_listener");

  /* NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.   */
  ros::NodeHandle n;

  /* Subscribe to wam_jpos topic. Data passed to chatterCallBack function.   */
  ros::Subscriber sub = n.subscribe("wam_jpos", 1000, chatterCallback);

  /* ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.   */
  ros::spin();

  return 0;
}


