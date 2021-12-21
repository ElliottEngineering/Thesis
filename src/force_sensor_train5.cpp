#include "ros/ros.h"
#include <cstdlib>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <limits>
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ContactsState.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"

#include "math.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include "terrain_library.h"

const int N = 5;



class variables{
private:
  ros::NodeHandle nh_;

  ros::Subscriber sub3;
  ros::Subscriber sub4;
  ros::Subscriber sub5;
  ros::Subscriber sub6;
  ros::Subscriber sub7;
  ros::Subscriber sub8;
  ros::Subscriber sub9;
  ros::Subscriber sub10;

  ros::Timer timer;
  ros::Timer timer2;
  ros::Timer timer3;

  float forces[notN-1][2];



public:
  variables(ros::NodeHandle* nodehandle):nh_(*nodehandle)
  {

    sub3 = nh_.subscribe("/sensor_fixed_joint_1_1", 1, &variables::sensor_fixed_joint_1_1_Callback, this);

    sub4 = nh_.subscribe("/sensor_fixed_joint_1_2", 1, &variables::sensor_fixed_joint_1_2_Callback, this);

    sub5 = nh_.subscribe("/sensor_fixed_joint_2_1", 1, &variables::sensor_fixed_joint_2_1_Callback, this);
    sub6 = nh_.subscribe("/sensor_fixed_joint_2_2", 1, &variables::sensor_fixed_joint_2_2_Callback, this);

    sub7 = nh_.subscribe("/sensor_fixed_joint_3_1", 1, &variables::sensor_fixed_joint_3_1_Callback, this);
    sub8 = nh_.subscribe("/sensor_fixed_joint_3_2", 1, &variables::sensor_fixed_joint_3_2_Callback, this);

    sub9 = nh_.subscribe("/sensor_fixed_joint_4_1", 1, &variables::sensor_fixed_joint_4_1_Callback, this);
    sub10 = nh_.subscribe("/sensor_fixed_joint_4_2", 1, &variables::sensor_fixed_joint_4_2_Callback, this);

  }

  void sensor_fixed_joint_1_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    float x = msg->wrench.force.x;
    float y = msg->wrench.force.y;
    float z = msg->wrench.force.z;
    forces[1][1] = sqrt(pos(x,2) + pow(y,2) + pow(z,2));
  }

};









int main(int argc, char **argv){

  ros::init(argc, argv, "force_sensor_train5");
  ros::NodeHandle node_handle;
  // create class
  variables v1(&node_handle);


  while(ros::ok()){

    ros::spinOnce();

  }
}
