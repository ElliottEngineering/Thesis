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
const double loopRate = 50;


double currentX[N];
double currentY[N];



double bodyHeading[N]; //each segment
double bodyHeadingRate[N]; //each segment
double axleHeading[N][2]; //two axles per segment
double axleHeadingRate[N][2]; //two axles per segment


double bodyJointZ[2*(N-1)];
double bodyJointZRate[2*(N-1)];
double bodyLonRate[N];
double axleJoint[N][2];


double wheelYaw[N][2][2];
double wheelYawRate[N][2][2];
double motorTorque[N][2][2];
double Xt[N][2][2] = {{{1,1},{1,1}},
                      {{1,1},{1,1}},
                      {{1,1},{1,1}}};
const double pi = 3.141592;
const double pi2 = 1.570796;





std::string body_name[N] = {"train3::body_1","train3::body_2","train3::body_3"};

std::string axle_name[N][2] =   {{"train3::axle_1_1","train3::axle_1_2"},
                                {"train3::axle_2_1","train3::axle_2_2"},
                                {"train3::axle_3_1","train3::axle_3_2"}};

std::string bodyJoint_name[2*(N-1)] = {"body_joint_1_z","body_joint_12_z","body_joint_2_z","body_joint_22_z"};

std::string axleJoint_name[N][2] = {{"pivot_joint_1_1","pivot_joint_1_2"},
                                        {"pivot_joint_2_1","pivot_joint_2_2"},
                                        {"pivot_joint_3_1","pivot_joint_3_2"}};

double get_yaw_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i, double offset){
  double x = msg->pose[i].orientation.x;
  double y = msg->pose[i].orientation.y;
  double z = msg->pose[i].orientation.z;
  double w = msg->pose[i].orientation.w;

  tf::Quaternion q(x,y,z,w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw + offset; //due to model orientation
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
  yaw = yaw;

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
    //first look for the body joints
    for(size_t i=0; i<2*(N-1); ++i){
      if(names[h].compare(bodyJoint_name[i]) == 0){
        bodyJointZ[i] = -msg->position[h]; //opposite direction due to vehicle orientation
        bodyJointZRate[i] = -msg->velocity[h];
        //ROS_INFO_STREAM(names[h] << " position is " << bodyJointZ[i]);
        goto found; //go to next point in message
      }
    }
    // then look for the axle joint names
    for(size_t i=0; i<N; ++i){
      for(size_t j=0; j<2; ++j){
        if(names[h].compare(axleJoint_name[i][j]) == 0){
          axleJoint[i][j] = msg->position[h]; //update position with current
          //ROS_INFO_STREAM(names[h] << " position is " << axleJoint[i][j]);
          goto found; //go to next point in message
        }
      }
    }
    found:;
  }
}

void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  //iterate through all the link names
  for(size_t h=0; h<names.size(); ++h){

    //first look through the axles
    for(size_t i=0; i<N; ++i){
      for(size_t j=0; j<2; ++j){
        if(names[h].compare(axle_name[i][j]) == 0){
          axleHeading[i][j] = get_yaw_from_link_states(msg,h,0);
          axleHeadingRate[i][j] = get_yawRate_from_link_states(msg,h);
          //ROS_INFO_STREAM(names[h] << " heading is " << axleHeading[i][j]);
          goto found; //go to next point in message
        }
      }
    }
    //then look through the bodies
    for(size_t i=0; i<N; ++i){
      if(names[h].compare(body_name[i]) == 0){
        bodyLonRate[i] = get_lonSpeed_from_link_states(msg,h);
        bodyHeading[i] = get_yaw_from_link_states(msg,h,0);
        bodyHeadingRate[i] = get_yawRate_from_link_states(msg,h);
        currentX[i] = get_xPos_from_link_states(msg,h);
        currentY[i] = get_yPos_from_link_states(msg,h);
        //ROS_INFO_STREAM(names[h] << " x position is " << currentX[i]);
        goto found; //go to next point in message
      }
    }
    found:; //found this link so check the next one
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "controller_train3");
  ros::NodeHandle controller_train3_node;

  ros::Publisher pub_motorSetPoint = controller_train3_node.advertise<std_msgs::Float64MultiArray>("/motorSetPoint", 1000);

  ros::Subscriber sub1 = controller_train3_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub2 = controller_train3_node.subscribe("/gazebo/link_states", 10, link_states_Callback);
  ros::Rate loop_rate(loopRate);
  while(ros::ok()){
    //create message to publish the motor setpoints
    std_msgs::Float64MultiArray msg;
    msg.data.resize(N*2*2);



    //calculate target location
    //calculate heading to goal

    //ROS_INFO_STREAM("headingToGoal is " << headingToGoal*180/pi);
    ROS_INFO_STREAM("####################");



    // basic control scheme, works mostly
    int i = 0;
    double navigationRadius[N] = {5,7,4};
    double lowerLimit = -1;
    double upperLimit = -0.3;
    double V = -0.5;
    double W1;
    double W2;
    double P1 = -2;
    double P2 = -1;

    double frontAxleError;
    double latDistance;


    double headingError = headingDifference(axleHeading[0][0], axleHeading[0][1]);
    W1 = P1 * headingError;
    msg.data[0*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
    msg.data[0*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
    ROS_INFO_STREAM(headingError);

    headingError = headingDifference(axleHeading[0][1], axleHeading[1][0]);
    W1 = P1 * headingError;
    msg.data[0*4 + 1*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
    msg.data[0*4 + 1*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
    ROS_INFO_STREAM(headingError);



    headingError = headingDifference(axleHeading[1][0], axleHeading[1][1]);
    W1 = P1 * headingError;
    msg.data[1*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
    msg.data[1*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
    ROS_INFO_STREAM(headingError);

    headingError = headingDifference(axleHeading[1][1], axleHeading[2][0]);
    W1 = P1 * headingError;
    msg.data[1*4 + 1*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
    msg.data[1*4 + 1*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
    ROS_INFO_STREAM(headingError);



    headingError = headingDifference(axleHeading[2][0], axleHeading[2][1]);
    W1 = P1 * headingError;
    msg.data[2*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
    msg.data[2*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
    ROS_INFO_STREAM(headingError);

    headingError = headingDifference(axleHeading[2][1], axleHeading[2][0]);
    W1 = P2 * headingError;
    msg.data[2*4 + 1*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
    msg.data[2*4 + 1*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
    ROS_INFO_STREAM(headingError);






    pub_motorSetPoint.publish(msg); //publish the motor setpoints

    //let ROS do its background stuff
    ros::spinOnce();
    loop_rate.sleep();
  }
}
