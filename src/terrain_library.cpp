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



const double pi = 3.141592;
const double pi2 = 1.570796;

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
  if(isnan(number)){
    return 0;
  }
  else if(number < lowerLimit){
    return lowerLimit;
  }
  else if (number > upperLimit){
	  return upperLimit;
  }
  return number;
}


double signSpeed(double number, double deadBand){
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

double sign(double number, double deadBand){
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



int isNotZero(double number, double deadBand){
  if(fabs(number) <= deadBand){
    return 0;
  }
  return 1;
}

double headingDifference(double goal, double current){
  while(goal < 0) {
    goal = goal + 2*pi;
  }

  while (current < 0) {
    current = current + 2*pi;
  }
  double diff = goal - current;
  //ROS_INFO_STREAM("diff " << diff*180/pi);

  if (diff > pi) {
    //ROS_INFO_STREAM("diff > pi");
    return diff - 2*pi;

  }
  else if (diff <= -pi) {
    //ROS_INFO_STREAM("diff < -pi");
    return diff + 2*pi;
  }
  //ROS_INFO_STREAM("return diff");
  return diff;
}
