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
const double k1 = 0;
const double k2 = 0;
const double k3 = 500;

//11 is front left, 12 front right, 21 back left...


double joint_position[N-1][2];
double axleJoint[N][2];
double fixedJoint[N-1];


const double pi = 3.141592;
const double pi2 = 1.570796;



std::string joint_name[N-1][2] = {{"body_joint_1_z","body_joint_12_z"},
                                  {"body_joint_2_z","body_joint_22_z"},
                                  {"body_joint_3_z","body_joint_32_z"},
                                  {"body_joint_4_z","body_joint_42_z"}};

std::string axleJoint_name[N][2] = {{"pivot_joint_1_1","pivot_joint_1_2"},
                                        {"pivot_joint_2_1","pivot_joint_2_2"},
                                        {"pivot_joint_3_1","pivot_joint_3_2"},
                                        {"pivot_joint_4_1","pivot_joint_4_2"},
                                        {"pivot_joint_5_1","pivot_joint_5_2"}};

std::string fixed_joint_name[N-1] = {"fixed_joint_0","fixed_joint_2","fixed_joint_4","fixed_joint_6"};



const double loopRate = 200;









void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  for(size_t h=0; h<names.size(); ++h){
    for(size_t i=0; i<N-1; ++i){
      for(size_t j=0; j<2; ++j){
        if(names[h].compare(joint_name[i][j]) == 0){
          //ROS_INFO_STREAM(names[h] << " should equal " << model_name[i][j][k]);
          joint_position[i][j] = msg->position[h]; //calculate
          goto found; //go to next point in message
        }
      }
    }
    // then look for the axle joint names
    for(size_t i=0; i<N; ++i){
      for(size_t j=0; j<2; ++j){
        if(names[h].compare(axleJoint_name[i][j]) == 0){
          axleJoint[i][j] = msg->position[h]; //update position with current
          //ROS_INFO_STREAM(names[h] << " position is " << axleJoint[i][j]);
          goto found; //go to next point in message
        }
      }
    }

    for(size_t i=0; i<N-1; ++i){
      if(names[h].compare(fixed_joint_name[i]) == 0){
        fixedJoint[i] = msg->position[h];
        goto found; //go to next point in message
      }
    }
    found:;
  }
}








int main(int argc, char **argv){
	//ROS_INFO_STREAM("test");


	ros::init(argc, argv, "spring_model_train5");
	ros::NodeHandle spring_model_node;

  //services called
  ros::ServiceClient jointEffortClient = spring_model_node.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort::Request apply_joint_effort_req;
	gazebo_msgs::ApplyJointEffort::Response apply_joint_effort_resp;

  //topics subscribed to
  ros::Subscriber sub1 = spring_model_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);







  double service_ready = false;
  while (!service_ready) {
    service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
    ros::Duration(0.5).sleep();
  }



	ros::Rate loop_rate(loopRate);

  ros::spinOnce();
  ros::Duration(1).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  ROS_INFO_STREAM("#########################################################");


	while(ros::ok()){

    static ros::Time start_time = ros::Time::now();
    ros::Duration loopTime = ros::Time::now() - start_time;
    start_time = ros::Time::now();


    apply_joint_effort_req.duration = loopTime;

    /*
    for(int i=0; i<(N-1); ++i){
      for(int j=0; j<2; ++j){
        //global reference frame
        apply_joint_effort_req.effort = -k1*joint_position[i][j];
        //apply_joint_effort_req.effort = constrain(1 + terrainResistanceTorque[i][j][k] + frictionTorque[i][j][k],-1000,1000);
    		apply_joint_effort_req.joint_name = joint_name[i][j];
    		// call apply body wrench service
    		jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);
      }
    }

    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        //global reference frame
        apply_joint_effort_req.effort = -k2*axleJoint[i][j];
        //apply_joint_effort_req.effort = constrain(1 + terrainResistanceTorque[i][j][k] + frictionTorque[i][j][k],-1000,1000);
    		apply_joint_effort_req.joint_name = axleJoint_name[i][j];
    		// call apply body wrench service
    		jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);
      }
    }
    */
    for(int i=0; i<(N-1); ++i){
        //global reference frame
        apply_joint_effort_req.effort = -k3*fixedJoint[i];
        //apply_joint_effort_req.effort = constrain(1 + terrainResistanceTorque[i][j][k] + frictionTorque[i][j][k],-1000,1000);
    		apply_joint_effort_req.joint_name = fixed_joint_name[i];
    		// call apply body wrench service
    		jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);

    }

		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM(" oops");
  return 0;
}
