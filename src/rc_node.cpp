#include <cstdlib>
#include <stdlib.h>     /* atoi */

#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h> //for arming vehicle
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>

//
int main(int argc, char **argv)
{
    if(argc<6){
        printf("Usage: rosrun rcoverride rc THROTTLE YAW FORWARD LATERAL CAM\n"); //Input pwm values. 1100 = full reverse, 1500 = neutral, 1950 = full FORWARD
        return -1;
    }

    ros::init(argc, argv, "mavros_rc_override");
    ros::NodeHandle n;

    //String to integer -- for PWM input mapping
    int THROTTLE  =   atoi(argv[1]);
    int YAW       =   atoi(argv[2]);
    int FORWARD   =   atoi(argv[3]);
    int LATERAL   =   atoi(argv[4]);
    int CAM       =   atoi(argv[5]);

    //Display PWM values
    ROS_INFO_STREAM("THROTTLE: " << THROTTLE);
    ROS_INFO_STREAM("YAW: " << YAW);
    ROS_INFO_STREAM("FORWARD: " << FORWARD);
    ROS_INFO_STREAM("LATERAL: " << LATERAL);
    ROS_INFO_STREAM("CAM: " << CAM);

    int rate = 100;
    ros::Rate r(rate);

    //Publisher for OverrideRCIn
    ros::Publisher rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    mavros_msgs::OverrideRCIn msg_override;

    //Check value of SYSID_MYGCS
    ros::ServiceClient mygcs_get = n.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    mavros_msgs::ParamGet gcsget;
    gcsget.request.param_id = "SYSID_MYGCS";

    //If SYSID_MYGCS =/= 1, set to 1.
    //Note: can be done in terminal with'rosrun mavros mavparam set SYSID_MYGCS 1' Will need to set value back to 255 in QGroundControl when used next.
    ros::ServiceClient mygcs_set = n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    for(;;) {
       if (mygcs_get.call(gcsget)) {
           ROS_INFO("Send OK %d Value: %ld", gcsget.response.success, gcsget.response.value.integer);
           if (gcsget.response.value.integer == 1) {
               break;
           } else {
               mavros_msgs::ParamSet gcsset;
               gcsset.request.param_id = "SYSID_MYGCS";
               gcsset.request.value.integer = 1;
               gcsset.request.value.real = 1;
               if (mygcs_set.call(gcsset)) {
                   ROS_INFO("SYSIS_MYGCS: send ok");
               } else {
                   ROS_INFO("Failed to call service SYSIS_MYGCS");
               }
           }
       } else {
           ROS_ERROR("Failed GET PARAMETER");
         }
       }

    //Set up service call to arm vehicle
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = true;
    arming_cl.call(srv_arm);

    //Loop for OverrideRCIn
    while (ros::ok()){
        msg_override.channels[0] = 1500;
        msg_override.channels[1] = 1500;
        msg_override.channels[2] = THROTTLE; //moves vehcile up and down
        msg_override.channels[3] = YAW;
        msg_override.channels[4] = FORWARD;
        msg_override.channels[5] = LATERAL;
        msg_override.channels[6] = 0;
        msg_override.channels[7] = CAM;

        rc_override_pub.publish(msg_override);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
