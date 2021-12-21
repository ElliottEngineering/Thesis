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


double goalX[6] = {0, 30,-30,-30, 30,30};
double goalY[6] = {0, 30, 30,-30,-30,30};

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
double axleJointRate[N][2];


double wheelYaw[N][2][2];
double wheelYawRate[N][2][2];
double motorTorque[N][2][2];
double Xt[N][2][2] = {{{1,1},{1,1}},
                      {{1,1},{1,1}},
                      {{1,1},{1,1}}};
const double pi = 3.141592;
const double pi2 = 1.570796;


int waypointNumber[N] = {1,1,1};






std::string body_name[N] = {"train3::body_1","train3::body_2","train3::body_3"};

std::string axle_name[N][2] =   {{"train3::axle_1_1","train3::axle_1_2"},
                                {"train3::axle_2_1","train3::axle_2_2"},
                                {"train3::axle_3_1","train3::axle_3_2"}};

std::string bodyJoint_name[2] = {"body_joint_1_z","body_joint_2_z"};

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

    //iterate through each body each time step
    double distanceToGoal[N];
    for(size_t i=0; i<N; ++i){
      double diffX = goalX[waypointNumber[i]] - currentX[i];
      double diffY = goalY[waypointNumber[i]] - currentY[i];

      //check if goal has been reached
      distanceToGoal[i] = pow(pow(diffX,2) + pow(diffY,2),0.5);
      //ROS_INFO_STREAM("body " << i << " distance to goal " << distanceToGoal);
      if (distanceToGoal[i] < 3) {
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


    if (bodyLonRate[0] > 0.01) {

      //calculate bearing from the goal to last waypoint
      //calculate distance to goal minus navigation radius "goaldist"
      // dead recok location goaldist distance from goal along path to previous waypoint, this is "target"
      // calculate bearing to target point
      //and so on

      double navigationRadius[N] = {10,7,4};
      double lowerLimit = -1;
      double upperLimit = 1;
      double V = 0.4;
      double W[2*N];
      double P1 = 0.5;
      double P2 = 1;
      double P3 = 1.6;
      double I1 = 0.05;
      double D1 = -1;
      for(size_t i=0; i<1; ++i){
        ROS_INFO_STREAM("Goal is " << goalX[waypointNumber[i]] << ", " << goalY[waypointNumber[i]]);
        ROS_INFO_STREAM("second axleJointRate " << axleJointRate[i][1]);
        double bearingFromGoal = atan2(goalY[waypointNumber[i]] - goalY[waypointNumber[i]-1],goalX[waypointNumber[i]] - goalX[waypointNumber[i]-1]);
        double goalDist = distanceToGoal[0] - navigationRadius[i];
        double targetX = goalX[waypointNumber[i]] - goalDist*cos(bearingFromGoal);
        double targetY = goalY[waypointNumber[i]] - goalDist*sin(bearingFromGoal);
        //the corresponding point on the line
        double shadowX = goalX[waypointNumber[i]] - distanceToGoal[i]*cos(bearingFromGoal);
        double shadowY = goalY[waypointNumber[i]] - distanceToGoal[i]*sin(bearingFromGoal);
        double latDistance = pow(pow(shadowX - currentX[i],2) + pow(shadowY - currentY[i],2),0.5);

        ROS_INFO_STREAM("latDistance " << latDistance);
        ROS_INFO_STREAM("Position " << currentX[i] << ", " << currentY[i]);
        ROS_INFO_STREAM("target is " << targetX << "," << targetY);

        double headingToTarget = atan2(targetY - currentY[0],targetX - currentX[0]);


        double frontAxleError = headingDifference(headingToTarget,axleHeading[i][0]);
        double rearAxleError  = headingDifference(0,axleJoint[i][1]);

        static double frontAxleIntegral = 0;
        frontAxleIntegral = constrain(frontAxleIntegral + (frontAxleError/loopRate),-1,1);
        ROS_INFO_STREAM("frontAxleIntegral " << frontAxleIntegral);

        ROS_INFO_STREAM("frontAxleError " << frontAxleError);
        ROS_INFO_STREAM("rearAxleError " << rearAxleError);

        W[0] = P1 * frontAxleError + I1*frontAxleIntegral;
        W[1] = P3 * rearAxleError + D1*axleJointRate[0][1];
        msg.data[i*4 + 0*2 + 0] = constrain(V - W[0],0,upperLimit);
        msg.data[i*4 + 0*2 + 1] = constrain(V + W[0],0,upperLimit);
        msg.data[i*4 + 1*2 + 0] = constrain(V - W[1],lowerLimit,upperLimit);
        msg.data[i*4 + 1*2 + 1] = constrain(V + W[1],lowerLimit,upperLimit);
      }



      double frontAxleError;
      double rearAxleError;

      frontAxleError = headingDifference(bodyJointZ[0], axleJoint[1][0]);
      rearAxleError = headingDifference(0, axleJoint[1][1]);
      W[2] = P3*frontAxleError;
      W[3] = P3*rearAxleError;
      ROS_INFO_STREAM("bodyJointZ[0] " << bodyJointZ[0]);

      msg.data[1*4 + 0*2 + 0] = constrain(V - W[2],lowerLimit,upperLimit);
      msg.data[1*4 + 0*2 + 1] = constrain(V + W[2],lowerLimit,upperLimit);
      msg.data[1*4 + 1*2 + 0] = constrain(V - W[3],lowerLimit,upperLimit);
      msg.data[1*4 + 1*2 + 1] = constrain(V + W[3],lowerLimit,upperLimit);


      frontAxleError = headingDifference(bodyJointZ[1], axleJoint[2][0]);
      rearAxleError = headingDifference(0, axleJoint[2][1]);
      W[4] = P3*frontAxleError;
      W[5] = P3*rearAxleError;

      msg.data[2*4 + 0*2 + 0] = constrain(V - W[4],lowerLimit,upperLimit);
      msg.data[2*4 + 0*2 + 1] = constrain(V + W[4],lowerLimit,upperLimit);
      msg.data[2*4 + 1*2 + 0] = constrain(V - W[5],lowerLimit,upperLimit);
      msg.data[2*4 + 1*2 + 1] = constrain(V + W[5],lowerLimit,upperLimit);

    }
    else{
      ROS_INFO_STREAM("I'm stuck");

      double sum = 0;
      for(size_t i=0; i<N; ++i){
        sum = sum + bodyHeading[i];
      }
      double averageHeading = sum / (N*2);
      ROS_INFO_STREAM("averageHeading " << averageHeading);
      double P1 = 2;
      double P2 = 2;

      for(size_t i=0; i<N; ++i){
        double frontAxleError = headingDifference(averageHeading,axleHeading[i][0]);
        double rearAxleError  = headingDifference(averageHeading,axleHeading[i][1]);
        double W1 = P2*frontAxleError;
        double W2 = P2*rearAxleError;
        double lowerLimit = 0.2;
        double V = 1;
        msg.data[i*4 + 0*2 + 0] = constrain(V - W1,lowerLimit,1);
        msg.data[i*4 + 0*2 + 1] = constrain(V + W1,lowerLimit,1);
        msg.data[i*4 + 1*2 + 0] = constrain(V - W2,lowerLimit,1);
        msg.data[i*4 + 1*2 + 1] = constrain(V + W2,lowerLimit,1);
      }

    }


/*
    // basic control scheme, works mostly
    int i = 0;
    double navigationRadius[N] = {5,7,4};
    double lowerLimit = 0.25;
    double V = 0.5;
    double W[2*N];
    double P1 = 1;
    double P2 = 2;
    double P3 = 3;
    double P4 = 10;
    double frontAxleError;
    double latDistance;

    for(size_t i=0; i<1; ++i){
      ROS_INFO_STREAM("Goal is " << goalX[waypointNumber[i]] << ", " << goalY[waypointNumber[i]]);
      double bearingFromGoal = atan2(goalY[waypointNumber[i]] - goalY[waypointNumber[i]-1],goalX[waypointNumber[i]] - goalX[waypointNumber[i]-1]);
      double goalDist = distanceToGoal[0] - navigationRadius[0];
      double targetX = goalX[waypointNumber[i]] - goalDist*cos(bearingFromGoal);
      double targetY = goalY[waypointNumber[i]] - goalDist*sin(bearingFromGoal);
      //the corresponding point on the line
      double shadowX = goalX[waypointNumber[i]] - distanceToGoal[i]*cos(bearingFromGoal);
      double shadowY = goalY[waypointNumber[i]] - distanceToGoal[i]*sin(bearingFromGoal);
      latDistance = pow(pow(shadowX - currentX[i],2) + pow(shadowY - currentY[i],2),0.5);

      ROS_INFO_STREAM("latDistance " << latDistance);
      ROS_INFO_STREAM("Position " << currentX[i] << ", " << currentY[i]);
      ROS_INFO_STREAM("target is " << targetX << "," << targetY);

      double headingToTarget = atan2(targetY - currentY[0],targetX - currentX[0]);
      double headingError = headingDifference(headingToTarget, axleHeading[i][0]);
      ROS_INFO_STREAM("headingError " << headingError);

      frontAxleError = headingDifference( headingError,axleJoint[i][0]);
    }

    ROS_INFO_STREAM("frontAxleError " << frontAxleError);
    //ROS_INFO_STREAM("frontAxleError " << frontAxleError);
    //frontAxleIntegral = constrain(frontAxleIntegral + frontAxleError,-50,50);
    //ROS_INFO_STREAM("frontAxleIntegral " << frontAxleIntegral);
    W[0] = P1*(frontAxleError);
    W[1] = P1*(frontAxleError);
    //ROS_INFO_STREAM("W[0] " << W[0]);
    //W[1] = - 2*(axleJoint[0][1] - constrain(-headingDiff,-0.18,0.18));



    msg.data[0*4 + 0*2 + 0] = constrain(V - W[0],lowerLimit,1);
    msg.data[0*4 + 0*2 + 1] = constrain(V + W[0],lowerLimit,1);
    msg.data[0*4 + 1*2 + 0] = constrain(V - W[1],lowerLimit,1);
    msg.data[0*4 + 1*2 + 1] = constrain(V + W[1],lowerLimit,1);


    msg.data[1*4 + 0*2 + 0] = constrain(V,lowerLimit,1);
    msg.data[1*4 + 0*2 + 1] = constrain(V,lowerLimit,1);
    msg.data[1*4 + 1*2 + 0] = constrain(V,lowerLimit,1);
    msg.data[1*4 + 1*2 + 1] = constrain(V,lowerLimit,1);


    msg.data[2*4 + 0*2 + 0] = constrain(V,lowerLimit,1);
    msg.data[2*4 + 0*2 + 1] = constrain(V,lowerLimit,1);
    msg.data[2*4 + 1*2 + 0] = constrain(V,lowerLimit,1);
    msg.data[2*4 + 1*2 + 1] = constrain(V,lowerLimit,1);


    */





    pub_motorSetPoint.publish(msg); //publish the motor setpoints

    //let ROS do its background stuff
    ros::spinOnce();
    loop_rate.sleep();
  }
}
