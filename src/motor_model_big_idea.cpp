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

double wheelAngularSpeed[2][2];
double motorTorque[4];
double motorSetPoint[4] = {0, 0, 0, 0};
const double pi = 3.141592;
const double pi2 = 1.570796;
const double loopRate = 2000;
const double slope = -18;// Nm / (rad/sec)
const double noLoadSpeed = 1.25; //rad/sec



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
    if(names[i].compare("joint_wheel_1_0") == 0){
      wheelAngularSpeed[1][0] = (msg->position[i] - wheelPosition[1][0]) * yamlRate; //calculate
      wheelPosition[1][0] = msg->position[i]; //update position with current

      //ROS_INFO_STREAM("joint_back_left position " << wheelPosition[1][0]);
    }
    else if (names[i].compare("joint_wheel_1_1") == 0){
      wheelAngularSpeed[1][1] = (msg->position[i] - wheelPosition[1][1]) * yamlRate; //calculate
      wheelPosition[1][1] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_back_right " << wheelAngularSpeed[1][1]);
    }
    else if (names[i].compare("joint_wheel_0_0") == 0){
      wheelAngularSpeed[0][0] = (msg->position[i] - wheelPosition[0][0]) * yamlRate; //calculate
      wheelPosition[0][0] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_front_left " << wheelAngularSpeed[0][0]);
    }
    else if (names[i].compare("joint_wheel_0_1") == 0){
      wheelAngularSpeed[0][1] = (msg->position[i] - wheelPosition[0][1]) * yamlRate; //calculate
      wheelPosition[0][1] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_front_right " << wheelAngularSpeed[0][1]);
    }
  }
}

void motorSetPoint_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  for(int i=0; i<2; i++){
    for(int j=0; j<2; j++){
      motorSetPoint[i*2 + j] = msg->data[(2*i)+j];
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
  ros::Subscriber sub5 = motor_model_node.subscribe("/winch/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub6 = motor_model_node.subscribe("/motorSetPoint", 10, motorSetPoint_Callback);




	ros::Rate loop_rate(loopRate);


  ros::spinOnce();
	while(ros::ok()){

    std_msgs::Float64MultiArray msg;
    msg.data.resize(4);

    double torque[4];


    for(int i=0; i<2; i++){
      for(int j=0; j<2; j++){

        double motorSpeed = 1*wheelAngularSpeed[i][j]; //constants already consider gearbox
        if(motorSetPoint[i*2 + j] == 2){ //if the value is two, let the motor freewheel
          msg.data[i*2 + j] = 0;
        }
        else { //otherwise do the standard calculation
          msg.data[i*2 + j] = min(slope*(motorSpeed - motorSetPoint[i*2 + j]*noLoadSpeed),22.5);
        }

        //msg.data.insert(msg.data.begin(),i*2 + j + 1,torque);
      }
    }
    for(int i=0; i<2; i++){
      for(int j=0; j<2; j++){
        ROS_INFO_STREAM("motorTorque " << msg.data[i*2 + j]);
        ROS_INFO_STREAM("motorSetPoint " << motorSetPoint[i*2 + j]);
        ROS_INFO_STREAM("wheelAngularSpeed " << wheelAngularSpeed[i][j]);
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
