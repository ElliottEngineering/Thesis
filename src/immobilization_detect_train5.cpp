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
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <string>

#include "terrain_library.h"

const int N = 5;



const float controllerLoopRate = 20;
const float wheelRadius = 0.28;

const float noLoadSpeed = 13.326; //rad/sec

const float pi = 3.141592;
const float pi2 = 1.570796;



/*
const std::string body_name[6] = {"train5::body_1","train5::body_2","train5::body_3","train5::body_4","train5::body_5","train5::body_6"};


const std::string axle_name[6][2] = {{"train5::axle_1_1","train5::axle_1_2"},{"train5::axle_2_1","train5::axle_2_2"},{"train5::axle_3_1","train5::axle_3_2"},{"train5::axle_4_1","train5::axle_4_2"},{"train5::axle_5_1","train5::axle_5_2"},{"train5::axle_6_1","train5::axle_6_2"}};


const std::string prismatic_name[5] = {"prismatic_joint_1","prismatic_joint_2","prismatic_joint_3","prismatic_joint_4","prismatic_joint_5"};

const std::string bodyJoint_name[5][2] = {{"body_joint_1_z","body_joint_12_z"},
																		{"body_joint_2_z","body_joint_22_z"},
																		{"body_joint_3_z","body_joint_32_z"},
																		{"body_joint_4_z","body_joint_42_z"},
																		{"body_joint_5_z","body_joint_52_z"}};

const std::string axleJoint_name[6][2] = {{"pivot_joint_1_1","pivot_joint_1_2"},
																	{"pivot_joint_2_1","pivot_joint_2_2"},
																	{"pivot_joint_3_1","pivot_joint_3_2"},
																	{"pivot_joint_4_1","pivot_joint_4_2"},
																	{"pivot_joint_5_1","pivot_joint_5_2"},
																	{"pivot_joint_6_1","pivot_joint_6_2"}};

const std::string joint_name[6][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}},
                                  {{"joint_wheel_4_1_1","joint_wheel_4_1_2"},{"joint_wheel_4_2_1","joint_wheel_4_2_2"}},
                                  {{"joint_wheel_5_1_1","joint_wheel_5_1_2"},{"joint_wheel_5_2_1","joint_wheel_5_2_2"}},
								  {{"joint_wheel_6_1_1","joint_wheel_6_1_2"},{"joint_wheel_6_2_1","joint_wheel_6_2_2"}}};

*/

class variables{
private:
  ros::NodeHandle nh_;

  float wheelAngularSpeed[notN][2][2];
  float motorTorque[notN][2][2];
  float bodyLonRate[notN];
  float bodyLatRate[notN];
  float currentX[notN];
  float currentY[notN];

  bool runController;
  std::string driveType;

  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Timer timer;




  float get_lonSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){

    double x = msg->pose[i].orientation.x;
    double y = msg->pose[i].orientation.y;
    double z = msg->pose[i].orientation.z;
    double w = msg->pose[i].orientation.w;

    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    float floatYaw = (float) yaw;

    float xVel = msg->twist[i].linear.x;
    float yVel = msg->twist[i].linear.y;

    float lonVel = xVel*cos(floatYaw) + yVel*sin(floatYaw);
    return lonVel;
  }

  float get_latSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    double x = msg->pose[i].orientation.x;
    double y = msg->pose[i].orientation.y;
    double z = msg->pose[i].orientation.z;
    double w = msg->pose[i].orientation.w;

    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    float floatYaw = (float) yaw;

    float xVel = msg->twist[i].linear.x;
    float yVel = msg->twist[i].linear.y;

    float latVel = -xVel*sin(floatYaw) + yVel*cos(floatYaw);
    return latVel;
  }

public:
  variables(ros::NodeHandle* nodehandle):nh_(*nodehandle)
  {

    runController = false;
    driveType = "normal";

    sub1 = nh_.subscribe("/dd_robot/joint_states", 1, &variables::joint_states_Callback, this, ros::TransportHints().tcpNoDelay());
    sub2 = nh_.subscribe("/gazebo/link_states", 1, &variables::link_states_Callback, this);
    sub3 = nh_.subscribe("/motorTorque", 1, &variables::motorTorque_Callback, this,  ros::TransportHints().tcpNoDelay());
    //timer callbacks
    timer = nh_.createTimer(ros::Duration(1 / controllerLoopRate), &variables::callback, this);
  }

  bool checkRunController(){
    return runController;
  }

  void setRunControllerFalse(){
    runController = false;
  }

  float getMotorTorque(int i, int j, int k){
    return motorTorque[i][j][k];
  }


  float getBodyLonRate(int i){
    return bodyLonRate[i];
  }

  std::string getDriveType(){
    return driveType;
  }

  void setDriveType(std::string newDriveType){
    driveType = newDriveType;
  }

  float get_xPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    return msg->pose[i].position.x;
  }

  float get_yPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    return msg->pose[i].position.y;
  }

  float getCurrentX(int i){
    return currentX[i];
  }

  float getCurrentY(int i){
    return currentY[i];
  }

  void motorTorque_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i=0; i<notN; i++){
      for(int j=0; j<2; j++){
        for(size_t k=0; k<2; ++k){
          motorTorque[i][j][k] = msg->data[i*4 + j*2 + k];
        }
      }
    }
  }

  void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
    //ROS_INFO_STREAM("joint_states_Callback");
    static bool first = true;

    static int wheelOrder[notN][2][2];

    const std::vector<std::string> &names = msg->name;

    if(!first){


      for(size_t i=0; i<notN; ++i){
        for(size_t j=0; j<2; ++j){
          for(size_t k=0; k<2; ++k){
            //wheelAngularSpeed[i][j][k] = (msg->position[wheelOrder[i][j][k]] - wheelAngularPosition[i][j][k]) / wheel_duration;
            //wheelAngularPosition[i][j][k] = msg->position[wheelOrder[i][j][k]];
            wheelAngularSpeed[i][j][k] = msg->velocity[wheelOrder[i][j][k]];
          }
        }
      }
    }else {

      first = false;
      for(size_t h=0; h<names.size(); ++h){
        //iterate through the body and axle joints

        //then go through the wheel joints
        for(size_t i=0; i<notN; ++i){
          for(size_t j=0; j<2; ++j){

            for(size_t k=0; k<2; ++k){
              if(names[h].compare(joint_name[i][j][k]) == 0){
                wheelOrder[i][j][k] = h;
                ROS_INFO_STREAM("wheel " << i << ", " << j << ", " << k << " found");
              }
            }
          }
        }
      }
      ROS_INFO_STREAM("order created");
    }
  }

  void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    //ROS_INFO_STREAM("link_states_Callback");
    static bool first = true;
    static int bodyOrder[notN];
    const std::vector<std::string> &names = msg->name;

    if(!first){
      for(size_t i=0; i<notN; ++i){
        //update each body
        bodyLonRate[i] = get_lonSpeed_from_link_states(msg,bodyOrder[i]);
        currentX[i] = get_xPos_from_link_states(msg,bodyOrder[i]);
        currentY[i] = get_yPos_from_link_states(msg,bodyOrder[i]);

      }

    }else{
      first = false;
      //iterate through all the link names
      for(size_t h=0; h<names.size(); ++h){

        //then look through the bodies
        for(size_t i=0; i<notN; ++i){
          if(names[h].compare(body_name[i]) == 0){
            bodyOrder[i] = h;
            ROS_INFO_STREAM("body " << i << " found");
          }
        }
      }
    }
  }

  void callback(const ros::TimerEvent&)
  {
    //ROS_INFO_STREAM("controllerCallback");
    runController = true;
  }



};









int main(int argc, char **argv){
  ros::init(argc, argv, "immobilization_detect_train5");
  ros::NodeHandle node_handle;

  // create class
  variables v1(&node_handle);
  ros::Publisher pub_driveType = node_handle.advertise<std_msgs::String>("/driveType", 5);
  ros::spinOnce();
  ros::Duration(2.0).sleep();


  while(ros::ok()){
    //let ROS do its background stuff


    //ROS_INFO_STREAM("Inside While Loop");



    if(v1.checkRunController()){ //run periodically to check for being stuck
      v1.setRunControllerFalse();

      static ros::Time start_time = ros::Time::now();
      ros::Duration timeSinceStart = ros::Time::now() - start_time;


      //to determine if vehicle is fully immobilized
      static bool upToSpeed = false;
      static bool stuck = false;
      float bodyLonRate = v1.getBodyLonRate(0);
      if(!upToSpeed &&  bodyLonRate > 0.5 && timeSinceStart.toSec() > 1){
        upToSpeed = true;
      } else if (upToSpeed && bodyLonRate < 0.1 && v1.getDriveType().compare("normal") == 0){
        stuck = true;
        upToSpeed = false;
        ROS_INFO_STREAM("Vehicle is immobilized");
      }

      static float motorTorqueEMA[notN][2];
      static bool axleImmobilized[notN][2];
      int immobilizedAxleCount = 0;
      for(int i=0; i<notN; ++i){
        for(int j=0; j<2; ++j){
          float measurement = (v1.getMotorTorque(i,j,0) + v1.getMotorTorque(i,j,1))/2;
          //0.18 corresponds to P = 10
          motorTorqueEMA[i][j] = constrain((measurement - motorTorqueEMA[i][j])*(0.18) + motorTorqueEMA[i][j],-100,100);
          //ROS_INFO_STREAM(motorTorqueEMA[i][j]);
          if (motorTorqueEMA[i][j] > 50) {
            axleImmobilized[i][j] = true;
            immobilizedAxleCount++;
          } else{
            axleImmobilized[i][j] = false;
          }
        }
      }

      if(upToSpeed && (v1.getDriveType().compare("normal") == 0) && (immobilizedAxleCount >= 6 || bodyLonRate < 0.15)){
        v1.setDriveType("inch1");
        ROS_INFO_STREAM("getting stuck, inch now");
      }

      //ROS_INFO_STREAM(v1.getCurrentX(0) << ", " << v1.getCurrentY(0) << ", " << v1.getCurrentX(1) << ", " << v1.getCurrentY(1) << ", " << v1.getCurrentX(2) << ", " << v1.getCurrentY(2) << ", " << v1.getCurrentX(3) << ", " << v1.getCurrentY(3) << ", " << v1.getCurrentX(4) << ", " << v1.getCurrentY(4));
      //ROS_INFO_STREAM(immobilizedAxleCount << " immobilizedAxleCount");
      //ROS_INFO_STREAM(motorTorqueEMA[0][0] << ", " << motorTorqueEMA[0][1] << ", " << motorTorqueEMA[1][0] << ", " << motorTorqueEMA[1][1] << ", " << motorTorqueEMA[2][0] << ", " << motorTorqueEMA[2][1] << ", " << motorTorqueEMA[3][0] << ", " << motorTorqueEMA[3][1] << ", " << motorTorqueEMA[4][0] << ", " << motorTorqueEMA[4][1] << ", " << motorTorqueEMA[5][0] << ", " << motorTorqueEMA[5][1]);

      std_msgs::String  driveType_msg;
      driveType_msg.data = v1.getDriveType();
      pub_driveType.publish(driveType_msg);




    }


    ros::spinOnce();
  }
  ROS_INFO_STREAM("ROS not ok, node failed");
}
