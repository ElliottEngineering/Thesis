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
const double pi = 3.141592;
const double pi2 = 1.570796;
const double loopRate = 2000;

//11 is front left, 12 front right, 21 back left...




double axleJoint[N][2];
std::string axleJoint_name[N][2] = {{"pivot_joint_1_1","pivot_joint_1_2"},
                                        {"pivot_joint_2_1","pivot_joint_2_2"},
                                        {"pivot_joint_3_1","pivot_joint_3_2"}};




void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  //iterate through the joint names
  for(size_t h=0; h<names.size(); ++h){
    // look for the axle joint names
    for(size_t i=0; i<N; ++i){
      for(size_t j=0; j<2; ++j){
        if(names[h].compare(axleJoint_name[i][j]) == 0){
          axleJoint[i][j] = msg->position[h]; //update position with current
          //ROS_INFO_STREAM(names[h] << " position is " << axleJoint[i][j]);
          goto found; //go to next point in message
        }
      }
    }
    found:;
  }
}








int main(int argc, char **argv){
	//ROS_INFO_STREAM("test");


	ros::init(argc, argv, "cable_model_train3");
	ros::NodeHandle cable_model_node;

  //services called
  ros::ServiceClient jointEffortClient = cable_model_node.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort::Request apply_joint_effort_req;
	gazebo_msgs::ApplyJointEffort::Response apply_joint_effort_resp;

  //topics subscribed to


  ros::Subscriber sub5 = cable_model_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);



  double service_ready = false;
  while (!service_ready) {
    service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
    ros::Duration(0.5).sleep();
  }



	ros::Rate loop_rate(loopRate);





  //calcuate slip




  ros::spinOnce();
  ros::Duration(1).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  ROS_INFO_STREAM("#########################################################");


	while(ros::ok()){
    ROS_INFO_STREAM("#########################################################");
    double axleJointTorques[N][2];

    for(int i=0; i<N; ++i){
      double jointDiff = headingDifference(axleJoint[i][0],-axleJoint[i][1]);
      axleJointTorques[i][0] =  500 * jointDiff;
      axleJointTorques[i][1] = -500 * jointDiff;
      ROS_INFO_STREAM("jointDiff " << jointDiff);
      ROS_INFO_STREAM(axleJointTorques[i][0]);
      ROS_INFO_STREAM(axleJointTorques[i][1]);
    }

		ros::Duration duration_temp(1.0/loopRate);
    apply_joint_effort_req.duration = duration_temp;
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        apply_joint_effort_req.effort = axleJointTorques[i][j];
        //apply_joint_effort_req.effort = constrain(1 + terrainResistanceTorque[i][j][k] + frictionTorque[i][j][k],-1000,1000);
    		apply_joint_effort_req.joint_name = axleJoint_name[i][j];
    		// call apply body wrench service
    		jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);
      }
    }

		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM(" oops");
  return 0;
}
