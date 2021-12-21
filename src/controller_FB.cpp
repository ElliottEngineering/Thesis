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


double goalX[6] = {0, 200,-30,-30, 30,30};
double goalY[6] = {0, 0, 30,-30,-30,30};

double currentX;
double currentY;



double bodyHeading; //each segment
double bodyHeadingRate; //each segment
double axleHeading; //two axles per segment
double axleHeadingRate; //two axles per segment



double bodyLonRate;
double axleJoint;
double axleJointRate;



const double pi = 3.141592;
const double pi2 = 1.570796;


int waypointNumber = 1;







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

  //iterate through the joint names
  for(size_t h=0; h<names.size(); ++h){
    // then look for the axle joint names
    if(names[h].compare("ddrobot::joint_pivot") == 0){
      axleJoint = msg->position[h]; //update position with current
      axleJointRate = msg->velocity[h];
      //ROS_INFO_STREAM(names[h] << " position is " << axleJoint[i][j]);
    }
  }
}

void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  //iterate through all the link names
  for(size_t h=0; h<names.size(); ++h){
    //first look through the axles
    if(names[h].compare("ddrobot::front_axle") == 0){
      axleHeading = get_yaw_from_link_states(msg,h,0);
      axleHeadingRate = get_yawRate_from_link_states(msg,h);
      //ROS_INFO_STREAM(names[h] << " heading is " << axleHeading[i][j]);
    }
    //then look through the bodies
    if(names[h].compare("ddrobot::base_link") == 0){
        bodyLonRate = get_lonSpeed_from_link_states(msg,h);
        bodyHeading = get_yaw_from_link_states(msg,h,0);
        bodyHeadingRate = get_yawRate_from_link_states(msg,h);
        currentX = get_xPos_from_link_states(msg,h);
        currentY = get_yPos_from_link_states(msg,h);
        //ROS_INFO_STREAM(names[h] << " x position is " << currentX[i]);
      }


  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "controller_FB");
  ros::NodeHandle controller_FB_node;

  ros::Publisher pub_motorSetPoint = controller_FB_node.advertise<std_msgs::Float64MultiArray>("/motorSetPoint", 1000);

  ros::Subscriber sub1 = controller_FB_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub2 = controller_FB_node.subscribe("/gazebo/link_states", 10, link_states_Callback);
  ros::Rate loop_rate(loopRate);
  while(ros::ok()){
    //create message to publish the motor setpoints
    std_msgs::Float64MultiArray msg;
    msg.data.resize(N*2*2);

    //iterate through each body each time step
    double distanceToGoal;

    double diffX = goalX[waypointNumber] - currentX;
    double diffY = goalY[waypointNumber] - currentY;

      //check if goal has been reached
    distanceToGoal = pow(pow(diffX,2) + pow(diffY,2),0.5);
    //ROS_INFO_STREAM("body " << i << " distance to goal " << distanceToGoal);
    if (distanceToGoal < 3) {
      //ROS_INFO_STREAM("Goal location reached");
      //set and reset location variables
      waypointNumber++;
      if (waypointNumber > 5) {
        waypointNumber = 1;
        ROS_INFO_STREAM("End of waypoint list reached, restarting");
      }
    }


    //calculate target location
    //calculate heading to goal

    //ROS_INFO_STREAM("headingToGoal is " << headingToGoal*180/pi);
    ROS_INFO_STREAM("####################");




    //calculate bearing from the goal to last waypoint
    //calculate distance to goal minus navigation radius "goaldist"
    // dead recok location goaldist distance from goal along path to previous waypoint, this is "target"
    // calculate bearing to target point
    //and so on

    double navigationRadius = 4;
    double lowerLimit = 0.2;
    double upperLimit = 1;
    double V = 0.6;
    double W[2*N];
    double P1 = 0.5;
    double P2 = 1;
    double P3 = 1.6;
    double I1 = 0.05;
    double D1 = -1;

    ROS_INFO_STREAM("Goal is " << goalX[waypointNumber] << ", " << goalY[waypointNumber]);

    double bearingFromGoal = atan2(goalY[waypointNumber] - goalY[waypointNumber-1],goalX[waypointNumber] - goalX[waypointNumber-1]);
    double goalDist = distanceToGoal - navigationRadius;
    double targetX = goalX[waypointNumber] - goalDist*cos(bearingFromGoal);
    double targetY = goalY[waypointNumber] - goalDist*sin(bearingFromGoal);
    //the corresponding point on the line
    double shadowX = goalX[waypointNumber] - distanceToGoal*cos(bearingFromGoal);
    double shadowY = goalY[waypointNumber] - distanceToGoal*sin(bearingFromGoal);
    double latDistance = pow(pow(shadowX - currentX,2) + pow(shadowY - currentY,2),0.5);

    ROS_INFO_STREAM("latDistance " << latDistance);
    ROS_INFO_STREAM("Position " << currentX << ", " << currentY);
    ROS_INFO_STREAM("target is " << targetX << "," << targetY);

    double headingToTarget = atan2(targetY - currentY,targetX - currentX);
    double frontAxleError = headingDifference(headingToTarget,axleHeading);
    static double frontAxleIntegral = 0;
    frontAxleIntegral = constrain(frontAxleIntegral + (frontAxleError/loopRate),-1,1);
    ROS_INFO_STREAM("frontAxleIntegral " << frontAxleIntegral);

    ROS_INFO_STREAM("frontAxleError " << frontAxleError);


    W[0] = P1 * frontAxleError + I1*frontAxleIntegral;
    W[1] = 0;
    msg.data[0*4 + 0*2 + 0] = constrain(V - W[0],lowerLimit,upperLimit);
    msg.data[0*4 + 0*2 + 1] = constrain(V + W[0],lowerLimit,upperLimit);
    msg.data[0*4 + 1*2 + 0] = constrain(V - W[0],lowerLimit,upperLimit);
    msg.data[0*4 + 1*2 + 1] = constrain(V + W[0],lowerLimit,upperLimit);

    msg.data[0*4 + 0*2 + 0] = V;
    msg.data[0*4 + 0*2 + 1] = V;
    msg.data[0*4 + 1*2 + 0] = V;
    msg.data[0*4 + 1*2 + 1] = V;


    ROS_INFO_STREAM("bodyLonRate " << bodyLonRate);



    pub_motorSetPoint.publish(msg); //publish the motor setpoints

    //let ROS do its background stuff
    ros::spinOnce();
    loop_rate.sleep();
  }
}
