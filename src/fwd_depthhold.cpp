//Want: arm vehicle ->move forward while keeping same depth
//Standards
#include <cstdlib>
#include <ros/ros.h>
//Msg types
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros/RCIn.h>
#include <mavros/State.h>
// #include <mavros_msgs/CommandBool.h> //for arming vehicle
// #include <mavros_msgs/ParamSet.h>
// #include <mavros_msgs/ParamGet.h>

//RC channels
#define THROTTLE 2
// #define YAW 3
// #define FORWARD 4
// #define LATERAL 5
// #define CAM 7
#define LEFT_TRIGGER //for manual override to end program

//Checks
#define PRELIMINARY_CHECK 1
#define LAND_CHECK 2 //check what this is

//RC values
#define MOVEFWD 1600 //value used during forward motion
#define RELEASE 1500
#define NO_RC

class Receiver
{
  public:
   void stateCallback(const mavros::State::ConstPtr& msg);
   void rc_inCallback(const mavros::RCIn::ConstPtr& msg);
   //PID HERE

   bool state_finished;
   bool rc_in_finished;

   int state_check;
   int rc_check_ch;
   int rc_check_val;
};

void Receiver::stateCallback(const mavros::State::ConstPtr& msg);
{
  //ROS_INFO("State Callback");
  if (terminate) return;
  if (state_finished) return;

  bool check1, check2;

  if (state_check == PRELIMINARY_CHECK)
  {
    if (msg->mode == "ALT_HOLD")
    {
      ROS_INFO("ALT_HOLD Activated");
      check1 = true;
    }
    else
    {
      ROS_INFO("Waiting to Enter ALT_HOLD");
      check1 = false;
      system("rosrun mavros mavsys mode -c ALT_HOLD"); //CHANGE TO SERVICE CALL
    }

    if (msg->armed)
    {
      ROS_INFO("Vehicle Armed");
      check2 = true;
    }
    else
    {
      ROS_INFO("Waiting for Vehcile to be Armed");
      check2 = false;
      system("rosrun mavros mavsafety arm");//CHANGE TO SERVICE CALL
    }

    if (check1 && check2)
    {
      ROS_INFO("Preliminary Check Complete");
      state_finished = true;
    }
  }

}

void Receiver::rc_inCallback(const mavros::RCIn::ConstPtr& msg);
{
  //ROS_INFO("RC Callback");

  //forces program end at left trigger 1500 or higher
  if (msg->channels[LEFT_TRIGGER] > 1500) terminate = true;

  if (rc_in_finished) return;

  if (msg->channels[rc_check_ch] <= (rc_check_val + 7) ||msg->channels[rc_check_ch] >= (rc_check_val - 7))
  {
    rc_in_finished = true;
    ROS_INFO("Complete: RC change")
  }
  else
  {
    rc_in_finished = false;
  }

}

//void Receiver::pidCallback(const (???)::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fwd_depthhold");
  ros::NodeHandle nh;

  //Subscribers
  Receiver receiver;
  ros::Subscriber rc_in_sub = nh.subscribe("mavros/rc/in", 10, &Receiver::rc_inCallback, &receiver);
  ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, &Receiver::stateCallback, &receiver);
  //ADD PID SUB HERE

  receiver.state_finished = true;
  receiver.rc_in_finished = true;
  //PID IN SAME FORMAT HERE
  receiver.termiate = false;

  //Publishers
  ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10, true);
  mavros_msgs::OverrideRCIn msg_override;
  //ADD PID PUB HERE

  //Arm vehicle and set depth hold msg_override
  ROS_INFO("Preliminary setup: arming and setting depth hold");
  //HOLD MODE
  system("rosrun mavros mavsys mode -c ALT_HOLD")//CHANGE TO SERVICE CALL
  //Arm
  system("rosrun mavros mavsafety arm")//CHANGE TO SERVICE CALL

  receiver.state_check = PRELIMINARY_CHECK;
  receiver.state_finished = false;
  while ((!receiver.state_finished) && (ros::ok()) && (!receiver.termiate))
  {
    ros::spinOnce();
  }
  ROS_INFO("Armed and Holding, Vehicle Movement Starting.");
  //Increase forward PWM to move
  for(int i=0; i<8; i++) rc_command.channels[i]=0; //releases all channels
  rc_command.channels[FORWARD] = MOVEFWD; //increase FORWARD PWM to value MOVEFWD

  receiver.rc_check_ch = FORWARD;
  receiver.rc_check_val = MOVEFWD;
  receiver.rc_in_finished = false;
  while ((!receiver.state_finished) && (ros::ok()) && (!receiver.termiate))
  {
    ros::spinOnce();
    rc_message.publish(rc_command);
  }
  ROS_INFO("Completed Movement Command")

  //Release rcoverride on program exit
  for(int i=0; i<8; i++) rc_command.channels[i]=0;

  receiver.rc_check_ch = THROTTLE;
  receiver.rc_check_val = RELEASE;
  receiver.rc_in_finished = false;
  while ((!receiver.rc_in_finished) && (ros::ok()))
  {
    ros:spinOnce();
    rc_message.publish(rc_command);
  }

  return 0;
}
