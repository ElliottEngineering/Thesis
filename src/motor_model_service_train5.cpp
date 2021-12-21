#include "ros/ros.h"
#include <cstdlib>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <limits>
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ContactsState.h"
#include "sensor_msgs/JointState.h"
#include <std_srvs/Empty.h>

#include <cmath>
#include <tf/transform_datatypes.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include "terrain_library.h"

const int N = 5;



//11 is front left, 12 front right, 21 back left...

const float pi = 3.141592;
const float pi2 = 1.570796;

const float slope = -13.0507;// Nm / (rad/sec)
const float noLoadSpeed = 13.326; //rad/sec
const float loopRate = 500;

const float wheelRadius = 0.28;
const float wheelWidth = 0.45;





const std::string model_name[N][2][2] = {{{"train5::wheel_1_1_1","train5::wheel_1_1_2"},{"train5::wheel_1_2_1","train5::wheel_1_2_2"}},
                                  {{"train5::wheel_2_1_1","train5::wheel_2_1_2"},{"train5::wheel_2_2_1","train5::wheel_2_2_2"}},
                                  {{"train5::wheel_3_1_1","train5::wheel_3_1_2"},{"train5::wheel_3_2_1","train5::wheel_3_2_2"}},
                                  {{"train5::wheel_4_1_1","train5::wheel_4_1_2"},{"train5::wheel_4_2_1","train5::wheel_4_2_2"}},
                                  {{"train5::wheel_5_1_1","train5::wheel_5_1_2"},{"train5::wheel_5_2_1","train5::wheel_5_2_2"}}};

const std::string joint_name[N][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}},
                                  {{"joint_wheel_4_1_1","joint_wheel_4_1_2"},{"joint_wheel_4_2_1","joint_wheel_4_2_2"}},
                                  {{"joint_wheel_5_1_1","joint_wheel_5_1_2"},{"joint_wheel_5_2_1","joint_wheel_5_2_2"}}};

//std::string joint_name[N][2][2] = {{{"train5::joint_wheel_1_1_1","train5::joint_wheel_1_1_2"},{"train5::joint_wheel_1_2_1","train5::joint_wheel_1_2_2"}},
//                                   {{"train5::joint_wheel_2_1_1","train5::joint_wheel_2_1_2"},{"train5::joint_wheel_2_2_1","train5::joint_wheel_2_2_2"}},
//                                   {{"train5::joint_wheel_3_1_1","train5::joint_wheel_3_1_2"},{"train5::joint_wheel_3_2_1","train5::joint_wheel_3_2_2"}}};


const std::string axle_name[N][2] = {{"train5::axle_1_1","train5::axle_1_2"},{"train5::axle_2_1","train5::axle_2_2"},{"train5::axle_3_1","train5::axle_3_2"},{"train5::axle_4_1","train5::axle_4_2"},{"train5::axle_5_1","train5::axle_5_2"}};



class variables{
private:
  ros::NodeHandle nh_;

  bool runTerrainModel;

  float axleHeading[notN][2];
  float contactNormal[notN][2][2];
  float motorTorque[notN][2][2];
  float wheelYawRate[notN][2][2];
  float Xt[notN][2][2];
  float wheelYaw[notN][2][2];
  float normalForce[notN][2][2];
  float wheelAngularSpeed[notN][2][2];
  float wheelLatSpeed[notN][2][2];
  float wheelLonSpeed[notN][2][2];

  float force_from_wheel(const gazebo_msgs::ContactsState::ConstPtr& msg){
    if(msg->states.size() > 0){
      float x = msg->states[0].total_wrench.force.x;
      float y = msg->states[0].total_wrench.force.y;
      float z = msg->states[0].total_wrench.force.z;

      return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
      //return msg->states[0].total_wrench.force.z;

    }
    else{
      return 0.0;
    }
  }

  float contactNormal_from_wheel(const gazebo_msgs::ContactsState::ConstPtr& msg){
    if(msg->states.size() > 0){
      return 1 - msg->states[0].contact_normals[0].z;
    }
    else{
      return 0;
    }
  }

  float get_lonSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i,float yaw){
    float xVel = msg->twist[i].linear.x;
    float yVel = msg->twist[i].linear.y;

    float lonVel = xVel*cos(yaw) + yVel*sin(yaw);
    return lonVel;
  }

  float get_latSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i,float yaw){
    float xVel = msg->twist[i].linear.x;
    float yVel = msg->twist[i].linear.y;

    float latVel = -xVel*sin(yaw) + yVel*cos(yaw);
    return latVel;
  }

  float get_axleHeading_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    double x = msg->pose[i].orientation.x;
    double y = msg->pose[i].orientation.y;
    double z = msg->pose[i].orientation.z;
    double w = msg->pose[i].orientation.w;

    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return (float) yaw;
  }

  float get_wheelYawRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    return msg->twist[i].angular.z;
  }

  ros::Timer timer;

  ros::Subscriber sub4;

public:
  variables(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    runTerrainModel = false;


    //topics subscribed to
    sub4 = nh_.subscribe("/motorTorque", 1, &variables::motorTorque_Callback, this);


    //timer callbacks
    timer = nh_.createTimer(ros::Duration(1 / loopRate), &variables::TerrainTimerCallback, this);

  }

  bool getRunTerrainModel(){
    return runTerrainModel;
  }

  void setRunTerrainModelFalse(){
    runTerrainModel = false;
  }

  float getAxleHeading(int i, int j){
    return axleHeading[i][j];
  }

  float getContactNormal(int i, int j, int k){
    return contactNormal[i][j][k];
  }

  float getMotorTorque(int i, int j, int k){
    return motorTorque[i][j][k];
  }

  float getWheelYaw(int i, int j, int k){
    return wheelYaw[i][j][k];
  }

  float getWheelYawRate(int i, int j, int k){
    return wheelYawRate[i][j][k];
  }

  float getXt(int i, int j, int k){
    return Xt[i][j][k];
  }

  float getNormalForce(int i, int j, int k){
    return normalForce[i][j][k];
  }

  float getWheelAngularSpeed(int i, int j, int k){
    return wheelAngularSpeed[i][j][k];
  }

  float getWheelLatSpeed(int i, int j, int k){
    return wheelLatSpeed[i][j][k];
  }

  float getWheelLonSpeed(int i, int j, int k){
    return wheelLonSpeed[i][j][k];
  }






  void motorTorque_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(int i=0; i<notN; i++){
      for(int j=0; j<2; j++){
        for(size_t k=0; k<2; ++k){
          motorTorque[i][j][k] = msg->data[i*4 + j*2 + k];
        }
      }
    }
  }




  void TerrainTimerCallback(const ros::TimerEvent&)
  {
    runTerrainModel = true;
  }

};







int main(int argc, char **argv){
	//ROS_INFO_STREAM("test");


	ros::init(argc, argv, "motor_model_service");
	ros::NodeHandle node_handle;

  variables v1(&node_handle);

  ros::ServiceClient jointEffortClient = node_handle.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort::Request apply_joint_effort_req;
	gazebo_msgs::ApplyJointEffort::Response apply_joint_effort_resp;



	ros::Rate loop_rate(loopRate);


  ros::spinOnce();
  ros::Duration(1.0).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  ROS_INFO_STREAM("#########################################################");


  int count = 0;
  while(ros::ok()){
    ros::spinOnce();
    //ROS_INFO_STREAM(v1.getRunTerrainModel());
    if(v1.getRunTerrainModel()){
      count++;
      v1.setRunTerrainModelFalse();

      //ROS_INFO_STREAM("running terrain model");


      //calculate the loop time and use it to apply the forces and torques
      static ros::Time start_time = ros::Time::now();
      ros::Duration loopTime = ros::Time::now() - start_time;
      start_time = ros::Time::now();
      //ROS_INFO_STREAM(loopTime);

      apply_joint_effort_req.duration = loopTime;


      for(int i=0; i<notN; ++i){
        for(int j=0; j<2; ++j){
          for(int k=0; k<2; ++k){
            apply_joint_effort_req.effort = v1.getMotorTorque(i,j,k);
            apply_joint_effort_req.joint_name = joint_name[i][j][k];
            // call apply body wrench service
            jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);
          }
        }
      }





  	}
  }
	ROS_INFO_STREAM(" oops");
  return 0;
}
