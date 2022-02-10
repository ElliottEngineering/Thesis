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


const double speedControllerLoopRate = 500; //half of actual roboteq controller rate
const double controllerLoopRate = 10;


const double noLoadSpeed = 13.326; //rad/sec

const double pi = 3.141592;
const double pi2 = 1.570796;
/*
const std::string body_name[N] = {"train5::body_1","train5::body_2","train5::body_3","train5::body_4","train5::body_5"};

const std::string axle_name[N][2] =   {{"train5::axle_1_1","train5::axle_1_2"},
                                {"train5::axle_2_1","train5::axle_2_2"},
                                {"train5::axle_3_1","train5::axle_3_2"},
                                {"train5::axle_4_1","train5::axle_4_2"},
                                {"train5::axle_5_1","train5::axle_5_2"}};

const std::string bodyJoint_name[N-1][2] = {{"body_joint_1_z","body_joint_12_z"},
                                  {"body_joint_2_z","body_joint_22_z"},
                                  {"body_joint_3_z","body_joint_32_z"},
                                  {"body_joint_4_z","body_joint_42_z"}};

const std::string axleJoint_name[N][2] = {{"pivot_joint_1_1","pivot_joint_1_2"},
                                        {"pivot_joint_2_1","pivot_joint_2_2"},
                                        {"pivot_joint_3_1","pivot_joint_3_2"},
                                        {"pivot_joint_4_1","pivot_joint_4_2"},
                                        {"pivot_joint_5_1","pivot_joint_5_2"}};

const std::string joint_name[N][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}},
                                  {{"joint_wheel_4_1_1","joint_wheel_4_1_2"},{"joint_wheel_4_2_1","joint_wheel_4_2_2"}},
                                  {{"joint_wheel_5_1_1","joint_wheel_5_1_2"},{"joint_wheel_5_2_1","joint_wheel_5_2_2"}}};
*/

const double goalX[7] = {0, 130,130,0, -130,-130,0};
const double goalY[7] = {0, 0, 90, 90,90,10,0};


class variables{
private:
  ros::NodeHandle nh_;

  double currentX[notN];
  double currentY[notN];
  double bodyHeading[notN]; //each segment
  double bodyHeadingRate[notN]; //each segment
  double axleHeading[notN][2]; //two axles per segment
  double axleHeadingRate[notN][2]; //two axles per segment
  double bodyJointZ[notN-1][2];
  double bodyJointZRate[notN-1][2];
  double bodyLonRate[notN];
  double bodyLatRate[notN];
  double axleJoint[notN][2];
  double axleJointRate[notN][2];


  double motorSetPoint[notN][2][2];

  double wheelYaw[notN][2][2];
  double wheelYawRate[notN][2][2];
  double motorTorque[notN][2][2];
  double Xt[notN][2][2];

  double wheelAngularSpeed[notN][2][2];



  bool runController;
  bool runSpeedControl;

  ros::Subscriber sub1;
  ros::Subscriber sub2;

  ros::Timer timer;
  ros::Timer timer2;

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

public:
  variables(ros::NodeHandle* nodehandle):nh_(*nodehandle)
  {

    runController = false;
    runSpeedControl = false;
    //subscribers
    sub1 = nh_.subscribe("/dd_robot/joint_states", 10, &variables::joint_states_Callback, this);
    sub2 = nh_.subscribe("/gazebo/link_states", 10, &variables::link_states_Callback, this);

    //timer callbacks
    timer = nh_.createTimer(ros::Duration(1 / controllerLoopRate), &variables::controllerCallback, this);
    timer2 = nh_.createTimer(ros::Duration(1 / speedControllerLoopRate), &variables::speedControlCallback, this);


  }

  bool checkRunController(){
    return runController;
  }

  void setRunControllerFalse(){
    runController = false;
  }

  bool checkRunSpeedControl(){
    return runSpeedControl;
  }

  void setRunSpeedControlFalse(){
    runSpeedControl = false;
  }

  float getCurrentX(int i){
    return currentX[i];
  }

  float getCurrentY(int i){
    return currentY[i];
  }

  float getBodyLonRate(int i){
    return bodyLonRate[i];
  }

  float getAxleJoint(int i,int j){
    return axleJoint[i][j];
  }

  float getAxleHeading(int i, int j){
    return axleHeading[i][j];
  }

  float getAxleJointRate(int i, int j){
    return axleJointRate[i][j];
  }

  float getWheelAngularSpeed(int i, int j, int k){
    return wheelAngularSpeed[i][j][k];
  }

  float getBodyJointZ(int i, int j){
    return bodyJointZ[i][j];
  }

  float getBodyJointZRate(int i, int j){
    return bodyJointZRate[i][j];
  }

  void setMotorSetPoint(int i, int j, int k, double value){
    motorSetPoint[i][j][k] = value;
  }

  double getMotorSetPoint(int i, int j, int k){
    return motorSetPoint[i][j][k];
  }


  void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
    static bool first = true;
    static int axleOrder[notN][2];
    static int wheelOrder[notN][2][2];
    static int bodyJointOrder[notN-1][2];
    const std::vector<std::string> &names = msg->name;
    if(!first){
      //ROS_INFO_STREAM("joint states updating");
      //iterate through the body joints
      for(size_t i=0; i<(notN-1); ++i){
        for(size_t j=0; j<2; ++j){
          bodyJointZ[i][j] = -msg->position[bodyJointOrder[i][j]]; //opposite direction due to vehicle orientation
          bodyJointZRate[i][j] = -msg->velocity[bodyJointOrder[i][j]];
        }
      }
      //then go through the axle joints
      for(size_t i=0; i<notN; ++i){
        for(size_t j=0; j<2; ++j){
          axleJoint[i][j] = msg->position[axleOrder[i][j]]; //update position with current
          axleJointRate[i][j] = msg->velocity[axleOrder[i][j]];
          for(size_t k=0; k<2; ++k){
            wheelAngularSpeed[i][j][k] = msg->velocity[wheelOrder[i][j][k]];
          }
        }
      }
    }else{
      ROS_INFO_STREAM("order created");
      first = false;
      for(size_t h=0; h<names.size(); ++h){
        //iterate through the body and axle joints
        for(size_t i=0; i<(notN-1); ++i){
          for(size_t j=0; j<2; ++j){
            //first look for the body joints
            if(names[h].compare(bodyJoint_name[i][j]) == 0){
              bodyJointOrder[i][j] = h;
            }
          }
        }
        //then go through the wheel joints
        for(size_t i=0; i<notN; ++i){
          for(size_t j=0; j<2; ++j){
            // then look for the axle joint names
            if(names[h].compare(axleJoint_name[i][j]) == 0){
              axleOrder[i][j] = h;
              //ROS_INFO_STREAM(names[h] << " position is " << axleJoint[i][j]);
            }
            for(size_t k=0; k<2; ++k){
              if(names[h].compare(joint_name[i][j][k]) == 0){
                wheelOrder[i][j][k] = h;
              }
            }
          }
        }
      }
    }
  }

  void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    static bool first = true;
    static int axleOrder[notN][2];
    static int bodyOrder[notN];
    const std::vector<std::string> &names = msg->name;

    if(!first){
      for(size_t i=0; i<notN; ++i){
        //update each body
        bodyLonRate[i] = get_lonSpeed_from_link_states(msg,bodyOrder[i]);
        bodyLatRate[i] = get_latSpeed_from_link_states(msg,bodyOrder[i]);
        bodyHeading[i] = get_yaw_from_link_states(msg,bodyOrder[i],0);
        bodyHeadingRate[i] = get_yawRate_from_link_states(msg,bodyOrder[i]);
        currentX[i] = get_xPos_from_link_states(msg,bodyOrder[i]);
        currentY[i] = get_yPos_from_link_states(msg,bodyOrder[i]);
        for(size_t j=0; j<2; ++j){
          //update each axles
          axleHeading[i][j] = get_yaw_from_link_states(msg,axleOrder[i][j],0);
          axleHeadingRate[i][j] = get_yawRate_from_link_states(msg,axleOrder[i][j]);
        }
      }

    }else{
      first = false;
      //iterate through all the link names
      for(size_t h=0; h<names.size(); ++h){

        //first look through the axles
        for(size_t i=0; i<notN; ++i){
          for(size_t j=0; j<2; ++j){
            if(names[h].compare(axle_name[i][j]) == 0){
              axleOrder[i][j] = h;

            }
          }
        }
        //then look through the bodies
        for(size_t i=0; i<notN; ++i){
          if(names[h].compare(body_name[i]) == 0){
            bodyOrder[i] = h;
          }
        }
      }
    }
  }

  void controllerCallback(const ros::TimerEvent&)
  {
    runController = true;
  }

  void speedControlCallback(const ros::TimerEvent&)
  {
    runSpeedControl = true;
  }

};









int main(int argc, char **argv){


  ros::init(argc, argv, "controller_train5");
  ros::NodeHandle node_handle;

  // create class
  variables v1(&node_handle);

  //ros::Publisher pub_motorSetSpeed = node_handle.advertise<std_msgs::Float64MultiArray>("/motorSetSpeed", 1000);
  ros::Publisher pub_motorSetPoint = node_handle.advertise<std_msgs::Float64MultiArray>("/motorSetPoint", 1);
  //ros::Publisher pub_frontAxleAngle = node_handle.advertise<std_msgs::Float64>("/frontAxleAngle", 1);




  int waypointNumber = 1;


  std_msgs::Float64MultiArray msg;
  msg.data.resize(notN*2*2);


  std_msgs::Float64 msg_frontAxle;


  ros::spinOnce();
  ros::Duration(0.5).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  ROS_INFO_STREAM("#########################################################");


  while(ros::ok()){
    //let ROS do its background stuff
    ros::spinOnce();

    double motorSetSpeed[notN][2][2];

    if(v1.checkRunController()){ //this is the navigation controller that sets desired wheel speeds
      v1.setRunControllerFalse();


      double distanceToGoal;
      //check if goal has been reached
      distanceToGoal = pow(pow(goalX[waypointNumber] - v1.getCurrentX(0),2) + pow(goalY[waypointNumber] - v1.getCurrentY(0),2),0.5);
      //ROS_INFO_STREAM("body " << i << " distance to goal " << distanceToGoal);
      if (distanceToGoal < 10) {
        waypointNumber++;
        ROS_INFO_STREAM("###### Waypoint Reached");

        if (waypointNumber > 6) {
          waypointNumber = 1;
          ROS_INFO_STREAM("######## End of waypoint list reached");
          break;
        }
      }


      const double navigationRadius = 20;
      const double P1 = 6;

      double V = constrain(1.15*v1.getBodyLonRate(0) + 0.2,0.5,2) / wheelRadius; //rad/s
      V = 1.93 / wheelRadius;
      double lowerLimit = 0.2 / wheelRadius;


      //V = V*0.95;
      int i = 0;
      double upperLimit = min(V*1.5,3.0) / wheelRadius;

      ROS_INFO_STREAM("goal is: " << goalX[waypointNumber] << ", " << goalY[waypointNumber]);
      double bearingFromGoal = atan2(goalY[waypointNumber] - goalY[waypointNumber-1],goalX[waypointNumber] - goalX[waypointNumber-1]);
      double goalDist = distanceToGoal - navigationRadius;
      double targetX = goalX[waypointNumber] - goalDist*cos(bearingFromGoal);
      double targetY = goalY[waypointNumber] - goalDist*sin(bearingFromGoal);
      //the corresponding point on the line
      double shadowX = goalX[waypointNumber] - distanceToGoal*cos(bearingFromGoal);
      double shadowY = goalY[waypointNumber] - distanceToGoal*sin(bearingFromGoal);

      double headingToTarget = atan2(targetY - v1.getCurrentY(i),targetX - v1.getCurrentX(i));

      double frontAxleError = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);


      double W1 = P1 * frontAxleError;

      double W2 = W1;

/*
      motorSetSpeed[0][0][0] = constrain(V - W1,lowerLimit,upperLimit);
      motorSetSpeed[0][0][1] = constrain(V + W1,lowerLimit,upperLimit);
      motorSetSpeed[0][1][0] = constrain(V - W2,lowerLimit,upperLimit);
      motorSetSpeed[0][1][1] = constrain(V + W2,lowerLimit,upperLimit);
    */

      motorSetSpeed[0][0][0] = V;
      motorSetSpeed[0][0][1] = V;
      motorSetSpeed[0][1][0] = V;
      motorSetSpeed[0][1][1] = V;






    }

    if(v1.checkRunSpeedControl()){ //This is the closed loop speed controller
      static double speedErrorIntegral[notN][2][2];

      v1.setRunSpeedControlFalse();

      //run the speed controller for each wheel
      const double P = 0.25;
      const double I = 1.0;



      for(int i=0; i<notN; ++i){
        for(int j=0; j<2; ++j){
          for(int k=0; k<2; ++k){

            double wheelAngularSpeed = v1.getWheelAngularSpeed(i,j,k);
            double speedDiff = motorSetSpeed[i][j][k] - wheelAngularSpeed;
            speedErrorIntegral[i][j][k] = constrain(speedErrorIntegral[i][j][k] + (speedDiff/speedControllerLoopRate),-0.2,0.2);

            double newSetPoint = constrain(motorSetSpeed[i][j][k]/noLoadSpeed + P*speedDiff + I*speedErrorIntegral[i][j][k],-1,1);
            v1.setMotorSetPoint(i,j,k,newSetPoint);
            msg.data[i*4 + j*2 + k] = newSetPoint;
            //msg.data[i*4 + j*2 + k] = 0.25;
            //msg.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k] / noLoadSpeed; //open loop
            if (i ==0, j ==0, k==0) {
              ROS_INFO_STREAM("#########################################################");
              ROS_INFO_STREAM("wheelAngularSpeed: " << wheelAngularSpeed);
              ROS_INFO_STREAM("motorSetSpeed[i][j][k]: " << motorSetSpeed[i][j][k]);
              ROS_INFO_STREAM("speedErrorIntegral[i][j][k]: " << speedErrorIntegral[i][j][k]);
              ROS_INFO_STREAM("speedDiff: " << speedDiff);
              ROS_INFO_STREAM("newSetPoint: " << newSetPoint);
            }

          }
        }
      }
      pub_motorSetPoint.publish(msg); //publish the motor setpoints
    }
  }
}
