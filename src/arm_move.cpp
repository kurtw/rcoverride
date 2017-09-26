#include <cstdlib>
#include <stdlib.h>     //atoi
#include <ros/ros.h>
//mavros msg types
#include <mavros/RCIn.h>
#include <mavros/State.h>
#include <mavros_msgs/OverrideRCIn.h>
//for arming vehicle
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
//fir setting depth hold
#include <mavros/SetMode.h>

//rcinCallback
void rcinCallback(const mavros::RCIn::ConstPtr& msg)
{
  ROS_INFO("rcinCallback start");
  //ADD FORCE STOP FOR SAFETY
  //CHECK RC CHANNEL PWM VALUES
}

//stateCallback
void stateCallback(const mavros::State::ConstPtr& msg)
{
  ROS_INFO("stateCallback start");
  //CHECK STATE, SET ALT_HOLD AND ARM VEHICLE
}

//main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_move");
  ros::NodeHandle nh;

  //Subscribers
  ros::Subscriber rcin_sub = nh.subscribe("mavros/rc/in", 10, rcinCallback);
  ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, stateCallback);

  //Publishers
  ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
  mavros_msgs::OverrideRCIn msg_override;

  //--------VEHICLE SETUP--------//
  //Check value of SYSID_MYGCS
  ros::ServiceClient mygcs_get = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
  mavros_msgs::ParamGet gcsget;
  gcsget.request.param_id = "SYSID_MYGCS";    //Check value of SYSID_MYGCS
  //**If SYSID_MYGCS =/= 1, set to 1. Note: can be done in terminal with \
  'rosrun mavros mavparam set SYSID_MYGCS 1' \
  Will need to set value back to 255 in QGroundControl when used next.**//
  ros::ServiceClient mygcs_set = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
  for(;;)
  {
     if(mygcs_get.call(gcsget))
     {
         ROS_INFO("Send OK %d Value: %ld", gcsget.response.success, gcsget.response.value.integer);
         if (gcsget.response.value.integer == 1)
         {
             break;
         }else{
             mavros_msgs::ParamSet gcsset;
             gcsset.request.param_id = "SYSID_MYGCS";
             gcsset.request.value.integer = 1;
             gcsset.request.value.real = 1;
             if (mygcs_set.call(gcsset)) {
                 ROS_INFO("SYSIS_MYGCS: send ok");
             }else{
                 ROS_INFO("Failed to call service SYSIS_MYGCS");
             }
         }
     }else{
         ROS_ERROR("Failed GET PARAMETER");
       }
     }
  //set ALT_HOLD with service call
  ros::ServiceClient depth_hold_cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode")
  mavros_msgs::SetMode srv_setmode;
  depth_hold_cl.request.base_mode = 0;
  depth_hold_cl.request.custom_mode = "ALT_HOLD";
  if(depth_hold_cl.call(srv_setmode))
  {
    ROS_ERROR("SetMode send ok %d",srv_setmode.response.success);
  }else{
    ROS_ERROR("Failed to set ALT_HOLD")
    return -1;
  }
  //arm vehcile with service call
  ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm;
  srv_arm.request.value = true;
  if(arming_cl.call(srv_arm))
  {
    ROS_ERROR("ARM send ok %d",srv_arm.response.success);
  }else{
    ROS_ERROR("Failed to arm");
  }

  //--------VEHICLE MOVEMENT--------//
  //can us rc_node for user input movement
  //or
  //setup movement when program runs


  return 0;
}
