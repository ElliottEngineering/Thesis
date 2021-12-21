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

const int N = 5;
const double loopRate = 50;
const double wheelRadius = 0.28;


double goalX[6] = {0, 100,-30,-30, 30,30};
double goalY[6] = {0, 100, 30,-30,-30,30};

double currentX[N];
double currentY[N];



double bodyHeading[N]; //each segment
double bodyHeadingRate[N]; //each segment
double axleHeading[N][2]; //two axles per segment
double axleHeadingRate[N][2]; //two axles per segment


double bodyJointZ[N-1][2];
double bodyJointZRate[N-1][2];
double bodyLonRate[N];
double bodyLatRate[N];
double axleJoint[N][2];
double axleJointRate[N][2];
double axleIntegral[N][2];

double wheelYaw[N][2][2];
double wheelYawRate[N][2][2];
double motorTorque[N][2][2];
double Xt[N][2][2] = {{{1,1},{1,1}},
                      {{1,1},{1,1}},
                      {{1,1},{1,1}}};
const double pi = 3.141592;
const double pi2 = 1.570796;


int waypointNumber[N] = {1,1,1};




/*

std::string body_name[N] = {"train5::body_1","train5::body_2","train5::body_3","train5::body_4","train5::body_5"};

std::string axle_name[N][2] =   {{"train5::axle_1_1","train5::axle_1_2"},
                                {"train5::axle_2_1","train5::axle_2_2"},
                                {"train5::axle_3_1","train5::axle_3_2"},
                                {"train5::axle_4_1","train5::axle_4_2"},
                                {"train5::axle_5_1","train5::axle_5_2"}};

std::string bodyJoint_name[N-1][2] = {{"body_joint_1_z","body_joint_12_z"},
                                  {"body_joint_2_z","body_joint_22_z"},
                                  {"body_joint_3_z","body_joint_32_z"},
                                  {"body_joint_4_z","body_joint_42_z"}};

std::string axleJoint_name[N][2] = {{"pivot_joint_1_1","pivot_joint_1_2"},
                                        {"pivot_joint_2_1","pivot_joint_2_2"},
                                        {"pivot_joint_3_1","pivot_joint_3_2"},
                                        {"pivot_joint_4_1","pivot_joint_4_2"},
                                        {"pivot_joint_5_1","pivot_joint_5_2"}};
*/
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

  double xVel = msg->twist[i].linear.x;
  double yVel = msg->twist[i].linear.y;

  double lonVel = xVel*cos(yaw) + yVel*sin(yaw);
  return lonVel;
}

double get_latSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
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

  double latVel = -xVel*sin(yaw) + yVel*cos(yaw);
  return latVel;
}


void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  //iterate through the joint names
  for(size_t h=0; h<names.size(); ++h){
    //first look for the body joints
    for(size_t i=0; i<(N-1); ++i){
      for(size_t j=0; j<2; ++j){
        if(names[h].compare(bodyJoint_name[i][j]) == 0){
          bodyJointZ[i][j] = -msg->position[h]; //opposite direction due to vehicle orientation
          bodyJointZRate[i][j] = -msg->velocity[h];
          //ROS_INFO_STREAM(names[h] << " position is " << bodyJointZ[i]);
          goto found; //go to next point in message
        }
      }
    }
    // then look for the axle joint names
    for(size_t i=0; i<N; ++i){
      for(size_t j=0; j<2; ++j){
        if(names[h].compare(axleJoint_name[i][j]) == 0){
          axleJoint[i][j] = msg->position[h]; //update position with current
          axleJointRate[i][j] = msg->velocity[h];
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
        bodyLatRate[i] = get_latSpeed_from_link_states(msg,h);
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
  ros::init(argc, argv, "controller_train5_crab");
  ros::NodeHandle controller_node;

  ros::Publisher pub_motorSetSpeed = controller_node.advertise<std_msgs::Float64MultiArray>("/motorSetSpeed", 1000);

  ros::Subscriber sub1 = controller_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub2 = controller_node.subscribe("/gazebo/link_states", 10, link_states_Callback);
  ros::Rate loop_rate(loopRate);
  while(ros::ok()){
    //create message to publish the motor setpoints
    std_msgs::Float64MultiArray msg;
    msg.data.resize(N*2*2);

    //iterate through each body each time step
    double distanceToGoal[N];
    for(size_t i=0; i<N; ++i){
      double diffX = goalX[waypointNumber[i]] - currentX[i];
      double diffY = goalY[waypointNumber[i]] - currentY[i];

      //check if goal has been reached
      distanceToGoal[i] = pow(pow(diffX,2) + pow(diffY,2),0.5);
      //ROS_INFO_STREAM("body " << i << " distance to goal " << distanceToGoal);
      if (distanceToGoal[i] < 10) {
        //ROS_INFO_STREAM("Goal location reached");
        //set and reset location variables
        waypointNumber[i]++;

        if (waypointNumber[i] > 5) {
          waypointNumber[i] = 1;
          ROS_INFO_STREAM("End of waypoint list reached, restarting");
        }
      }
    }

    //calculate target location
    //calculate heading to goal

    //ROS_INFO_STREAM("headingToGoal is " << headingToGoal*180/pi);
    ROS_INFO_STREAM("####################");
    ROS_INFO_STREAM("speed is " << bodyLonRate[0]);

    double navigationRadius = 5;


    double P1;
    double P2;
    double P3;

    double D1 = -3;

    double I = 0;

    double frontAxleError;
    double rearAxleError;
    double frontAxleError2;
    double rearAxleError2;

    double W1;
    double W2;
    double V = constrain(1.15*bodyLonRate[0] + 0.2,0.3,2) / wheelRadius; //rad/s
    double lowerLimit = 0.1 / wheelRadius;

    bool crabFlag = true;

    for(size_t i=0; i<N; ++i){
      //V = V*0.95;
      double upperLimit = min(V*1.5,3.0) / wheelRadius;
      P1 = V*4;
      P2 = V*1;
      P3 = V*1.5;

      if(i == 0){
        ROS_INFO_STREAM("Goal is " << goalX[waypointNumber[i]] << ", " << goalY[waypointNumber[i]]);
        //ROS_INFO_STREAM("second axleJointRate " << axleJointRate[i][1]);
        double bearingFromGoal = atan2(goalY[waypointNumber[i]] - goalY[waypointNumber[i]-1],goalX[waypointNumber[i]] - goalX[waypointNumber[i]-1]);
        double goalDist = distanceToGoal[i] - navigationRadius;
        double targetX = goalX[waypointNumber[i]] - goalDist*cos(bearingFromGoal);
        double targetY = goalY[waypointNumber[i]] - goalDist*sin(bearingFromGoal);
        //the corresponding point on the line
        double shadowX = goalX[waypointNumber[i]] - distanceToGoal[i]*cos(bearingFromGoal);
        double shadowY = goalY[waypointNumber[i]] - distanceToGoal[i]*sin(bearingFromGoal);
        double latDistance = pow(pow(shadowX - currentX[i],2) + pow(shadowY - currentY[i],2),0.5);
        double headingToTarget = atan2(targetY - currentY[i],targetX - currentX[i]);

        double frontAxleGoal = constrain(headingDifference(headingToTarget,axleHeading[i][0]),-0.35,0.35);
        frontAxleError = headingDifference(frontAxleGoal,axleJoint[i][0]);

        W1 = P1 * frontAxleError + D1*axleJointRate[i][0];
        //rearAxleError  = headingDifference(-frontAxleGoal,axleJoint[i][1]);
        rearAxleError  = headingDifference(0,axleJoint[i][1]);
        axleIntegral[i][1] = axleIntegral[i][1] + rearAxleError;
        W2 = P2*rearAxleError + D1*axleJointRate[i][1];
        ROS_INFO_STREAM(frontAxleGoal);
        ROS_INFO_STREAM(frontAxleError);


        if (fabs(frontAxleGoal) >= 0.35 || bodyLonRate[0] < 1.0 || frontAxleError > 0.1) {
          crabFlag = false;
          ROS_INFO_STREAM("crabFlag false");
          rearAxleError  = headingDifference(0,axleJoint[i][1]);
          axleIntegral[i][1] = axleIntegral[i][1] + rearAxleError;
          W2 = P2*rearAxleError + D1*axleJointRate[i][1];
          msg.data[i*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
          msg.data[i*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 0] = constrain(V - W2,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 1] = constrain(V + W2,lowerLimit,upperLimit);
        } else if (fabs(frontAxleGoal) > 0.25) {
          crabFlag = true;
          ROS_INFO_STREAM("crabFlag fuzzy true");
          rearAxleError  = headingDifference(0,axleJoint[i][1]);
          axleIntegral[i][1] = axleIntegral[i][1] + rearAxleError;
          W2 = P2*rearAxleError;
          msg.data[i*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
          msg.data[i*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 0] = constrain(V - W2*0.5,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 1] = constrain(V + W2*0.5,lowerLimit,upperLimit);
        } else {
          crabFlag = true;
          ROS_INFO_STREAM("crabFlag true");
          rearAxleError  = headingDifference(0.25,axleJoint[i][1]);
          axleIntegral[i][1] = axleIntegral[i][1] + rearAxleError;
          W2 = P2*rearAxleError;
          msg.data[i*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
          msg.data[i*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 0] = constrain(V - W2,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 1] = constrain(V + W2,lowerLimit,upperLimit);
        }



      }
      else{

        if (crabFlag == true) {
          //ROS_INFO_STREAM("running crabFlag");
          //double headingToTarget = 3.14;
          //double frontAxleGoal = constrain(headingDifference(headingToTarget,axleHeading[i][0]),-0.3,0.25);
          frontAxleError = headingDifference(0.3,axleJoint[i][0]);
          frontAxleError2 = 1.1*bodyJointZ[i-1][1];
          W1 = P1*frontAxleError + P2*frontAxleError2;

          rearAxleError  = headingDifference(0.3,axleJoint[i][1]);

          W2 = P1*rearAxleError;
          msg.data[i*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
          msg.data[i*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 0] = constrain(V - W2,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 1] = constrain(V + W2,lowerLimit,upperLimit);
        } else {
          //ROS_INFO_STREAM("still not crabFlag");
          frontAxleError = headingDifference(bodyJointZ[i-1][1], axleJoint[i][0]);
          W1 = P3*frontAxleError;

          rearAxleError  = headingDifference(0,axleJoint[i][1]);
          W2 = P3*rearAxleError;

          msg.data[i*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,upperLimit);
          msg.data[i*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 0] = constrain(V - W2,lowerLimit,upperLimit);
          msg.data[i*4 + 1*2 + 1] = constrain(V + W2,lowerLimit,upperLimit);
        }
      }



      ROS_INFO_STREAM("");
      ROS_INFO_STREAM(V*wheelRadius);
      ROS_INFO_STREAM(frontAxleError << ", " << rearAxleError);
      //ROS_INFO_STREAM(axleIntegral[i][0] << ", " << axleIntegral[i][1]);

      axleIntegral[i][0] = constrain(axleIntegral[i][0],-20,20);
      axleIntegral[i][1] = constrain(axleIntegral[i][1],-20,20);




      //ROS_INFO_STREAM(W1*wheelRadius << ", " << W2*wheelRadius);
    }

    pub_motorSetSpeed.publish(msg); //publish the motor setpoints

    //let ROS do its background stuff
    ros::spinOnce();
    loop_rate.sleep();
  }
}
