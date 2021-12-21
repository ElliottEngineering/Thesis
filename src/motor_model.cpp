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




//11 is front left, 12 front right, 21 back left...
double motorSetPoint[4] = {0, 0, 0, 0};
double wheelAngularSpeed[2][2];
double motorTorque[4];
//double motorSetPoint[4] = {0, 0, 0, 0};
const double pi = 3.141592;
const double pi2 = 1.570796;
const double loopRate = 200;
const double slope = -13.0507;// Nm / (rad/sec)
const double noLoadSpeed = 13.326; //rad/sec

  std::string joint_name[2][2] = {{"ddrobot::joint_front_left","ddrobot::joint_front_right"},{"ddrobot::joint_back_left","ddrobot::joint_back_right"}};



double max(double A, double B){
  if(A > B){
    return A;
  }
  return B;

}

double min(double A, double B){
  if(B < A){
    return B;
  }
  return A;

}

double constrain(double number, double lowerLimit, double upperLimit){
  if(number > upperLimit){
    return upperLimit;
  }
  else if(number < lowerLimit){
    return lowerLimit;
  }
  return number;
}

double signSpeed(double number, double deadBand = 0.01){
  //negative for negative numbers, positive for positive numbers, the number near zero
  if(number < -deadBand){
    return -1;
  }
  else if(number > deadBand){
    return 1;
  }
  else if(number == 0){
    return 0;
  }

  return number / deadBand; //won't divide by zero since check already occured
}

double sign(double number, double deadBand = 0){
  if(number < -deadBand){
    return -1;
  }
  else if(number > deadBand){
    return 1;
  }
  return 0;

}

double signDoubleBand(double number, double deadBand1, double deadBand2){
  if(fabs(number) <= deadBand1){
    return 0;
  }
  else if(number < -deadBand2){
    return -1;
  }
  else if(number > deadBand2){
    return 1;
  }
  return sign(number)*(fabs(number) - deadBand1) / (deadBand2 - deadBand1);
}

int isNotZero(double number, double deadBand = 0.01){
  if(fabs(number) <= deadBand){
    return 0;
  }
  return 1;
}

void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  static double wheelPosition[2][2];
  static double yamlRate = 1000; // must be same as in YAML


  for(size_t i=0; i<names.size(); ++i){
    if(names[i].compare("joint_back_left") == 0){
      //wheelAngularSpeed[1][0] = (msg->position[i] - wheelPosition[1][0]) * yamlRate; //calculate
      wheelAngularSpeed[1][0] = msg->velocity[i]; //calculate
      wheelPosition[1][0] = msg->position[i]; //update position with current

      //ROS_INFO_STREAM("joint_back_left position " << wheelPosition[1][0]);
    }
    else if (names[i].compare("joint_back_right") == 0){
      //wheelAngularSpeed[1][1] = (msg->position[i] - wheelPosition[1][1]) * yamlRate; //calculate
      wheelAngularSpeed[1][1] = msg->velocity[i]; //calculate
      wheelPosition[1][1] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_back_right " << wheelAngularSpeed[1][1]);
    }
    else if (names[i].compare("joint_front_left") == 0){
      //wheelAngularSpeed[0][0] = (msg->position[i] - wheelPosition[0][0]) * yamlRate; //calculate
      wheelAngularSpeed[0][0] = msg->velocity[i]; //calculate
      wheelPosition[0][0] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_front_left " << wheelAngularSpeed[0][0]);
    }
    else if (names[i].compare("joint_front_right") == 0){
      //wheelAngularSpeed[0][1] = (msg->position[i] - wheelPosition[0][1]) * yamlRate; //calculate
      wheelAngularSpeed[0][1] = msg->velocity[i]; //calculate
      wheelPosition[0][1] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_front_right " << wheelAngularSpeed[0][1]);
    }
  }
}

//void motorSetPoint_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
void motorSetPoint_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  for(int i=0; i<2; i++){
    for(int j=0; j<2; j++){
      motorSetPoint[i*2 + j] = motorSetPoint[i*2 + j]*0.95 + msg->data[(2*i)+j]*0.05;
      //ROS_INFO_STREAM(motorSetPoint[i*2 + j]);
    }
  }
}


int main(int argc, char **argv){


  ROS_INFO_STREAM("init motor_model_node");
	ros::init(argc, argv, "motor_model");
	ros::NodeHandle motor_model_node;

  //published topics
  ros::Publisher pub_motorTorque = motor_model_node.advertise<std_msgs::Float64MultiArray>("/motorTorque", 1000);


  //topics subscribed to
  ros::Subscriber sub5 = motor_model_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub6 = motor_model_node.subscribe("/motorSetPoint", 10, motorSetPoint_Callback);

  /*
  ros::ServiceClient jointEffortClient = motor_model_node.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort::Request apply_joint_effort_req;
	gazebo_msgs::ApplyJointEffort::Response apply_joint_effort_resp;


  double service_ready = false;
  while (!service_ready) {
    service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
    ros::Duration(0.5).sleep();
  }
  */

	ros::Rate loop_rate(loopRate);
  ros::spinOnce();
  double motorTorque[2][2] = {{0.0},{0,0}};

	while(ros::ok()){
    static ros::Time start_time = ros::Time::now();
    ros::Duration loopTime = ros::Time::now() - start_time;
    start_time = ros::Time::now();

    std_msgs::Float64MultiArray msg;
    msg.data.resize(4);
    double motorSpeed; //constants already consider gearbox

    for(int i=0; i<2; i++){
      for(int j=0; j<2; j++){
        motorSpeed = 1*wheelAngularSpeed[i][j];

        if(motorSetPoint[i*2 + j] == 2){ //if the value passed is two, let the motor freewheel
          motorTorque[i][j] = 0;
        }
        else { //otherwise do the standard calculation
          motorTorque[i][j] = min(slope*(motorSpeed - motorSetPoint[i*2 + j]*noLoadSpeed),110);
        }
        msg.data[i*2 + j] = motorTorque[i][j];

        //apply_joint_effort_req.effort = motorTorque;
        //apply_joint_effort_req.joint_name = joint_name[i][j];
        //jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);

        //msg.data.insert(msg.data.begin(),i*2 + j + 1,torque);
      }
    }
    int i = 1;
    int j = 0;
    //ROS_INFO_STREAM("motorTorque " << motorTorque);
    //ROS_INFO_STREAM("motorSetPoint " << motorSetPoint[i*2 + j]);
    //ROS_INFO_STREAM("wheelAngularSpeed " << wheelAngularSpeed[i][j]);




    pub_motorTorque.publish(msg);

		ros::spinOnce();
    //ROS_INFO_STREAM("test5");

		loop_rate.sleep();
	}
	ROS_INFO_STREAM(" oops");
return 0;
}
