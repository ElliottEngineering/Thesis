//includes gearbox reduction
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

//11 is front left, 12 front right, 21 back left...

double wheelAngularSpeed[N][2][2];
double motorTorque[N*2*2];
double motorSetPoint[N*2*2];


const double loopRate = 2000;
const double slope = -13.0507;// Nm / (rad/sec)
const double noLoadSpeed = 13.326; //rad/sec
const double wheelRadius = 0.28;

std::string model_name[N][2][2] = {{{"train3::wheel_1_1_1","train3::wheel_1_1_2"},{"train3::wheel_1_2_1","train3::wheel_1_2_2"}},
                                  {{"train3::wheel_2_1_1","train3::wheel_2_1_2"},{"train3::wheel_2_2_1","train3::wheel_2_2_2"}},
                                  {{"train3::wheel_3_1_1","train3::wheel_3_1_2"},{"train3::wheel_3_2_1","train3::wheel_3_2_2"}}};

std::string joint_name[N][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}}};




void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  static double wheelPosition[N][2][2];
  static double yamlRate = 1000; // must be same as in YAML
  for(size_t h=0; h<names.size(); ++h){
    for(size_t i=0; i<N; ++i){
      for(size_t j=0; j<2; ++j){
        for(size_t k=0; k<2; ++k){
          if(names[h].compare(joint_name[i][j][k]) == 0){
            wheelAngularSpeed[i][j][k] = (msg->position[h] - wheelPosition[i][j][k]) * yamlRate; //calculate
            wheelPosition[i][j][k] = msg->position[h]; //update position with current
            goto found; //go to next point in message
          }
        }
      }
    }
    found:;
  }
}

void motorSetPoint_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  for(int i=0; i<N; i++){
    for(int j=0; j<2; j++){
      for(size_t k=0; k<2; ++k){
        motorSetPoint[i*4 + j*2 + k] = msg->data[i*4 + j*2 + k];

      }
    }
  }
}


int main(int argc, char **argv){
  ROS_INFO_STREAM("init motor_model_node");
	ros::init(argc, argv, "motor_model");
	ros::NodeHandle motor_model_node;

  //services called
  ros::ServiceClient jointEffortClient = motor_model_node.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
  gazebo_msgs::ApplyJointEffort::Request apply_joint_effort_req;
  gazebo_msgs::ApplyJointEffort::Response apply_joint_effort_resp;

  //published topics
  ros::Publisher pub_motorTorque = motor_model_node.advertise<std_msgs::Float64MultiArray>("/motorTorque", 1000);


  //topics subscribed to
  ros::Subscriber sub5 = motor_model_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub6 = motor_model_node.subscribe("/motorSetPoint", 10, motorSetPoint_Callback);




	ros::Rate loop_rate(loopRate);


  ros::spinOnce();
	while(ros::ok()){

    std_msgs::Float64MultiArray msg;
    msg.data.resize(N*2*2);


    ros::Duration duration_temp(1.0/loopRate);
    apply_joint_effort_req.duration = duration_temp;
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          double motorSpeed = 1*wheelAngularSpeed[i][j][k]; //constants already consider gearbox
          double motorTorque;
          if (motorSetPoint[i*4 + j*2 + k] == 2){ //if sent a 2, let motors freewheel
            motorTorque = 0;
          }
          else{ //otherwise do normal calculation
            motorTorque = min(slope*(motorSpeed - motorSetPoint[i*4 + j*2 + k]*noLoadSpeed),110);
          }
          msg.data[i*4 + j*2 + k] = motorTorque;
          apply_joint_effort_req.effort = motorTorque;
          //apply_joint_effort_req.effort = constrain(1 + terrainResistanceTorque[i][j][k] + frictionTorque[i][j][k],-1000,1000);
      		apply_joint_effort_req.joint_name = joint_name[i][j][k];
      		// call apply body wrench service
      		jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);
        }
      }
    }

    pub_motorTorque.publish(msg);
		ros::spinOnce();
    //ROS_INFO_STREAM("test5");

		loop_rate.sleep();
	}
	ROS_INFO_STREAM(" oops");
return 0;
}
