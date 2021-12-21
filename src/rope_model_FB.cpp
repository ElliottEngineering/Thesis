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

const double loopRate = 1000;
const double pi = 3.141592;
const double pi2 = 1.570796;

double ropeLength = 2; //this should probably be updated via a Subscriber
double sledDistance;

double sledX;
double sledY;
double anchorX;
double anchorY;


double ropeTension; //the tension in the rope
double ropeHeading;
double sledYawRate;
double sledYaw;

double sledLonRate;
double sledLatRate;
double ropeSpeed;

double sledNormalForce;


double get_xPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->pose[i].position.x;
}

double get_yPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->pose[i].position.y;
}

double get_yawRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->twist[i].angular.z;
}

double get_xRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->twist[i].linear.x;
}

double get_yRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->twist[i].linear.y;
}

double get_yaw_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  double x = msg->pose[i].orientation.x;
  double y = msg->pose[i].orientation.y;
  double z = msg->pose[i].orientation.z;
  double w = msg->pose[i].orientation.w;

  tf::Quaternion q(x,y,z,w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
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

double get_latSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
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

  double latVel = -xVel*sin(yaw) + yVel*cos(yaw);
  return latVel;
}

double normalForce(const gazebo_msgs::ContactsState::ConstPtr& msg){
  if(msg->states.size() > 0){
    double x = msg->states[0].total_wrench.force.x;
    double y = msg->states[0].total_wrench.force.y;
    double z = msg->states[0].total_wrench.force.z;
    return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
  }
  else{
    return 0.0;
  }
}

void bumper_sled_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
  sledNormalForce = normalForce(msg);
  //ROS_INFO_STREAM("sledNormalForce " << sledNormalForce);
}

void winch_length_Callback(const std_msgs::Float64::ConstPtr& msg){
  ropeLength = msg->data;
  //ROS_INFO_STREAM("sledNormalForce " << sledNormalForce);
}

void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    const std::vector<std::string> &names = msg->name;
    for(size_t i=0; i<names.size(); ++i){
      if(names[i].compare("ddrobot::robot_anchor") == 0){
        anchorX = get_xPos_from_link_states(msg,i);
        anchorY = get_yPos_from_link_states(msg,i);
        //ROS_INFO_STREAM("robot anchor found");
      }
      else if (names[i].compare("ddrobot::sled_anchor") == 0){
        sledX = get_xPos_from_link_states(msg,i);
        sledY = get_yPos_from_link_states(msg,i);
        //ROS_INFO_STREAM("sled anchor found");
      }

      else if (names[i].compare("ddrobot::sled") == 0){
        sledYawRate = get_yawRate_from_link_states(msg,i);
        sledLonRate = get_lonSpeed_from_link_states(msg,i);
        sledLatRate = get_latSpeed_from_link_states(msg,i);
        sledYaw = get_yaw_from_link_states(msg,i);
        //ROS_INFO_STREAM("sled anchor found");
      }
    }

    static double sledDistanceOld = sledDistance;
    static ros::Time begin = ros::Time::now();
    ros::Duration duration_temp(ros::Time::now() - begin);
    double ropeSpeedTime = duration_temp.toSec();

    if (ropeSpeedTime > 0.001) {
      begin = ros::Time::now();
      sledDistance = pow(pow(anchorX - sledX,2) + pow(anchorY - sledY,2),0.5);
      ropeSpeed = min((sledDistance - sledDistanceOld) / ropeSpeedTime,5);
      sledDistanceOld = sledDistance;
    }


    if (sledDistance <= ropeLength) {
      ropeTension = 0;
    }
    else if (sledDistance > ropeLength) {
      double percentElongation = 100*(sledDistance / ropeLength - 1);
      ropeTension = 100 * pow(percentElongation,1.5); //keeps values low to start
      //1000 N tension based on https://sterlingrope.com/store/work/ropes/static/workpro/11-mm-workpro
      //look into stiffness parameter
    }
    if (ropeTension > 24000) {
      ropeTension = 100;
      ROS_INFO_STREAM("Rope Just Broke");
    }
    else if (ropeTension < 0) {
      ropeTension = 0;
    }
    ropeTension = 0;
}




int main(int argc, char **argv){
	//ROS_INFO_STREAM("test");
  ros::init(argc, argv, "rope_model_FB");
	ros::NodeHandle rope_model_FB_node;

  ros::ServiceClient wrenchClient = rope_model_FB_node.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
  gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;


  ros::Subscriber sub1 = rope_model_FB_node.subscribe("/gazebo/link_states", 10, link_states_Callback);

  ros::Subscriber sub2 = rope_model_FB_node.subscribe("/bumper_sled", 10, bumper_sled_Callback);

  ros::Subscriber sub3 = rope_model_FB_node.subscribe("/winch_length", 10, winch_length_Callback);


  bool service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
		//ROS_INFO("waiting for apply_body_wrench service");
		ros::Duration(0.5).sleep();
	}


  ros::Rate loop_rate(loopRate);

  ros::Duration duration_temp(1.0/loopRate);
  apply_wrench_req.duration = duration_temp;

  while(ros::ok()){

    ropeHeading = atan2(sledY - anchorY,sledX - anchorX);
    //ROS_INFO_STREAM("ropeHeading " << ropeHeading);
    ROS_INFO_STREAM("#############################");
    ROS_INFO_STREAM("sledDistance " << sledDistance);
    ROS_INFO_STREAM("ropeLength " << ropeLength);
    ROS_INFO_STREAM("ropeTension " << ropeTension);
    ROS_INFO_STREAM("ropeSpeed " << ropeSpeed);
    //ROS_INFO_STREAM("sledYawRate " << sledYawRate);


    //ROS_INFO_STREAM("sledLonRate " << sledLonRate);
    //ROS_INFO_STREAM("sledLatRate " << sledLatRate);
    //ROS_INFO_STREAM("sledYaw " << sledYaw);

    //global reference frame
    apply_wrench_req.wrench.force.x = ropeTension*cos(ropeHeading);
    apply_wrench_req.wrench.force.y = ropeTension*sin(ropeHeading);
    apply_wrench_req.wrench.torque.z = 0.0;
    apply_wrench_req.body_name = "ddrobot::robot_anchor";
    // call apply body wrench service
    wrenchClient.call(apply_wrench_req, apply_wrench_resp);


    //global reference frame
    apply_wrench_req.wrench.force.x = -(ropeTension)*cos(ropeHeading);
    apply_wrench_req.wrench.force.y = -(ropeTension)*sin(ropeHeading);
    apply_wrench_req.wrench.torque.z = 0;
    apply_wrench_req.body_name = "ddrobot::sled_anchor";
    // call apply body wrench service
    wrenchClient.call(apply_wrench_req, apply_wrench_resp);


    double sledRopeAngle = headingDifference(ropeHeading + pi,sledYaw);
    //ROS_INFO_STREAM("sledRopeAngle " << sledRopeAngle);


    double forceLon;

    if (sledLonRate > 0.01) {
      forceLon = -0.08*sledNormalForce*sign(sledLonRate);
    }
    else{
      forceLon = -0.1*sledNormalForce*sign(sledLonRate);
    }

    double forceLat = -0.5*sledNormalForce*sign(sledLatRate);
    //ROS_INFO_STREAM("sled force lon " << forceLon);
    //ROS_INFO_STREAM("sled force lat " << forceLat);

    apply_wrench_req.wrench.force.x  = forceLon*cos(sledYaw) - forceLat*sin(sledYaw);
    //ROS_INFO_STREAM("wrench.force.x " << apply_wrench_req.wrench.force.x);
    apply_wrench_req.wrench.force.y  = forceLon*sin(sledYaw) + forceLat*cos(sledYaw);
    //ROS_INFO_STREAM("wrench.force.y " << apply_wrench_req.wrench.force.y);
    apply_wrench_req.wrench.torque.z = -0.1*sign(sledYawRate)*sledNormalForce;
    apply_wrench_req.body_name = "ddrobot::sled";
    // call apply body wrench service
    wrenchClient.call(apply_wrench_req, apply_wrench_resp);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
