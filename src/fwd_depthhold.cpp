//Want: arm vehicle ->move forward while keeping same depth

#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros/RCIn.h>
#include <mavros_msgs/CommandBool.h> //for arming vehicle
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros/State.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fwd_depthhold");
  ros::NodeHandle nh;

  //Subs & Pubs
  Receiver receiver;
  ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::OverrideRCIn>("mavros/rc/in", 10, &Receiver::stateCallback, &receiver);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::OverrideRCIn>("mavros/state", 10, &Receiver::rcCallback, &receiver);

  









}
