#include "ros/ros.h"
#include <cstdlib>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <limits>
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ContactsState.h"
#include "sensor_msgs/JointState.h"

#include "math.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include "terrain_library.h"

const int N = 3;
const double loopRate = 1000;


double goalX = 50;
double goalY = 50;

double currentX;
double currentY;

double startX = 0;
double startY = 0;

double bodyHeading; //each segment
double bodyHeadingRate[N]; //each segment
double axleHeading[2]; //two axles per segment
double axleHeadingRate[N][2]; //two axles per segment


double bodyJointZ[N-1];
double axleJoint;


double wheelYaw[N][2][2];
double wheelYawRate[N][2][2];
double motorTorque[N][2][2];


const double pi = 3.141592;
const double pi2 = 1.570796;

double body_speed = 0;





std::string axle_name[2] =   {"worm::axle_1_1","worm::axle_1_2"};




std::string axleJoint_name = "pivot_joint_1_2";


double get_yaw_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i, double offset){
  double x = msg->pose[i].orientation.x;
  double y = msg->pose[i].orientation.y;
  double z = msg->pose[i].orientation.z;
  double w = msg->pose[i].orientation.w;

  tf::Quaternion q(x,y,z,w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw - offset; //due to model orientation
}

double get_xPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->pose[i].position.x;
}

double get_yPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->pose[i].position.y;
}

double get_yawRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->twist[i].angular.z;
}



double get_lonSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){

  double x = msg->pose[i].orientation.x;
  double y = msg->pose[i].orientation.y;
  double z = msg->pose[i].orientation.z;
  double w = msg->pose[i].orientation.w;

  tf::Quaternion q(x,y,z,w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);


  double xVel = msg->twist[i].linear.x;
  double yVel = msg->twist[i].linear.y;

  double lonVel = xVel*cos(yaw) + yVel*sin(yaw);
  return lonVel;
}

void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){

  const std::vector<std::string> &names = msg->name;
  static double wheelPosition[N][2][2];
  static double yamlRate = 1000; // must be same as in YAML
  //iterate through the joint names
  for(size_t h=0; h<names.size(); ++h){
    //ROS_INFO_STREAM(names[h]);
    // then look through the axle joint names
    if(names[h].compare("pivot_joint_1_2") == 0){
      axleJoint = msg->position[h]; //update position with current
      //ROS_INFO_STREAM("axleJoint is " << axleJoint);
      break;
    }
  }
}

void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){

  const std::vector<std::string> &names = msg->name;
  //iterate through all the link names
  for(size_t h=0; h<names.size(); ++h){

    //first look through the axles
    for(size_t i=0; i<N; ++i){
      if(names[h].compare(axle_name[i]) == 0){
        axleHeading[i] = get_yaw_from_link_states(msg,h,0);
        //axleHeadingRate[i] = get_yawRate_from_link_states(msg,h);
        //ROS_INFO_STREAM(names[h] << " heading is " << axleHeading[i][j]);
      }

    }

    if(names[h].compare("worm::body_1") == 0){
      body_speed = get_lonSpeed_from_link_states(msg,h);
      currentX = get_xPos_from_link_states(msg,h);
      currentY = get_yPos_from_link_states(msg,h);
      bodyHeading = get_yaw_from_link_states(msg,h,0);
      //ROS_INFO_STREAM("body speed is " << body_speed);
      //ROS_INFO_STREAM(names[h] << " x position is " << currentX[i]);
    }
  }
}

int main(int argc, char **argv){



  ros::init(argc, argv, "controller_worm");
  ros::NodeHandle controller_worm_node;

  ros::Publisher pub_motorSetPoint = controller_worm_node.advertise<std_msgs::Float64MultiArray>("/motorSetPoint", 1000);
  ros::Publisher pub_internalJoint = controller_worm_node.advertise<std_msgs::Float64>("/worm/pivot_joint_1_1_controller/command", 1000);

  ros::Subscriber sub1 = controller_worm_node.subscribe("/worm/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub2 = controller_worm_node.subscribe("/gazebo/link_states", 10, link_states_Callback);

  ros::Rate loop_rate(loopRate);

  ros::Rate worm_rate(0.5); //rate for inchworm behavior

  //variables for escape routine
  static ros::Time begin = ros::Time::now();
  static ros::Time last_stuck = ros::Time::now();
  ros::Time sleep_time = ros::Time::now();

  ros::Duration stuck_duration(10.0);
  ros::Duration unstuck_duration(2.0);
  ros::Duration sleep_duration(2.0);

  static bool stuck = false;


  while(ros::ok()){
    //create message to publish the motor setpoints
    std_msgs::Float64MultiArray msg;
    msg.data.resize(N*2*2);
    //create message to publish the internal joint setpoint
    std_msgs::Float64 msg_internal;


    double distanceToGoal;

    double diffX = goalX - currentX;
    double diffY = goalY - currentY;

    //check if goal has been reached
    distanceToGoal = pow(pow(diffX,2) + pow(diffY,2),0.5);
    //ROS_INFO_STREAM("body " << i << " distance to goal " << distanceToGoal);
    if (distanceToGoal < 5) {
      //ROS_INFO_STREAM("Goal location reached");
      //set and reset location variables
      startX = goalX;
      goalX = -goalX;
      startY = goalY;
      goalY = -goalY;
    }


    //calculate target location
    //calculate heading to goal
    double headingToGoal = atan2(goalY - currentY, goalX - currentX);
    //ROS_INFO_STREAM("body " << i << " heading to goal " << headingToGoal);

    double headingDiff = headingDifference(headingToGoal, bodyHeading);

    //ROS_INFO_STREAM("body " << i << " headingDiff " << headingDiff);
    double V = 0.5;
    double W1 = 0.5*headingDiff;
    double W2 = 0.5*headingDiff;

    msg.data[0*2 + 0] = constrain(V - W1,0.1,0.5);
    msg.data[0*2 + 1] = constrain(V + W1,0.1,0.5);
    msg.data[1*2 + 0] = constrain(V - W2,0.1,0.5);
    msg.data[1*2 + 1] = constrain(V + W2,0.1,0.5);
    msg_internal.data = -100;
    pub_motorSetPoint.publish(msg); //publish the motor setpoints
    pub_internalJoint.publish(msg_internal); //publish the motor setpoints
    ros::spinOnce();

    //check if stuck
    if (body_speed < 0.1 && !stuck && ros::Time::now() - last_stuck > unstuck_duration) {
      ROS_INFO_STREAM("Stuck now");
      stuck = true;
      begin = ros::Time::now();
    }

    while (stuck && ros::Time::now() - begin < stuck_duration) {
      sleep_time = ros::Time::now();
      double W;
      double P = 0.5;


      ROS_INFO_STREAM("heave");
      msg_internal.data = 100;
      pub_internalJoint.publish(msg_internal); //publish the motor setpoints
      while (ros::Time::now() - sleep_time < sleep_duration) {
        W = - P*axleJoint;
        msg.data[0*2 + 0] = constrain(0 - W,0,1);
        msg.data[0*2 + 1] = constrain(0 + W,0,1);
        msg.data[1*2 + 0] = 0.1;
        msg.data[1*2 + 1] = 0.1;
        pub_motorSetPoint.publish(msg); //publish the motor setpoints
        ros::spinOnce();
        loop_rate.sleep();
      }


      ROS_INFO_STREAM("ho");
      sleep_time = ros::Time::now();
      msg_internal.data = -100;
      pub_internalJoint.publish(msg_internal); //publish the motor setpoints
      while (ros::Time::now() - sleep_time < sleep_duration) {
        W = - P*axleJoint;
        msg.data[0*2 + 0] = constrain(0.1 - W,0,1);
        msg.data[0*2 + 1] = constrain(0.1 + W,0,1);
        msg.data[1*2 + 0] = 0;
        msg.data[1*2 + 1] = 0;
        pub_motorSetPoint.publish(msg); //publish the motor setpoints
        ros::spinOnce();
        loop_rate.sleep();
      }

    }
    if (stuck) {
      stuck = false;
      last_stuck = ros::Time::now();
      ROS_INFO_STREAM("unstuck");
    }

    //let ROS do its background stuff
    ros::spinOnce();
  }
}
