#include "ros/ros.h"
#include <cstdlib>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <limits>
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ContactsState.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"

#include "math.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "terrain_library.h"

#include <string>
#include <vector> 

//to enable cout instead of ROS_INFO_STREAM
#include <iostream>
using namespace std;

//const int N = 10;




const float controllerLoopRate = 20;
const float wheelRadius = 0.28;

const float noLoadSpeed = 13.326; //rad/sec

const float pi = 3.141592;
const float pi2 = 1.570796;

const float navigationRadius = 10;

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
//const float goalX[7] = {0, 130,130,0, -130,-130,0};
//const float goalY[7] = {0, 0, 90, 90,90,10,0};

//const float goalX[36] = {0, 0, 5, 5, 10, 10, 15, 15, 20, 20, 25, 25, 30, 30, 35, 35, 40, 40, 45, 45, 50, 50, 55, 55, 60, 60, 65, 65, 70, 70, 75, 75, 80, 80, 85, 85};
//const float goalY[36] = {0, 110, 110, 0, 0, 110, 110, 0, 0, 110, 110, 0, 0, 110, 110, 0, 0, 110, 110, 0, 0, 110, 110, 0, 0, 110, 110, 0, 0, 110, 110, 0, 0, 110, 110, 0};

const float goalX[1] = {1000};
const float goalY[1] = {0};

class variables{
private:
  ros::NodeHandle node;

  float currentX[notN];
  float currentY[notN];
  float bodyHeading[notN]; //each segment
  float bodyHeadingRate[notN]; //each segment
  float axleHeading[notN][2]; //two axles per segment
  float axleHeadingRate[notN][2]; //two axles per segment
  float bodyJointZ[notN-1][2];
  float bodyJointZRate[notN-1][2];
  float bodyLonRate[notN];
  float bodyLatRate[notN];
  float axleJoint[notN][2];
  float axleJointRate[notN][2];


  float wheelYaw[notN][2][2];
  float wheelYawRate[notN][2][2];


  float wheelAngularSpeed[notN][2][2];
  float wheelAngularPosition[notN][2][2];
  
  float connectorForce[notN - 1];



  bool runController;
  float heaveHo;
  float leftRight;
  std::string driveType;

  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  
  ros::Subscriber sub6;
  ros::Subscriber sub7;
  ros::Subscriber sub8;
  ros::Subscriber sub9;
  ros::Subscriber sub10;
  ros::Subscriber sub11;
  ros::Subscriber sub12;
  ros::Subscriber sub13;
  ros::Subscriber sub14;


  ros::Timer timer;
  ros::Timer timer_inch;


  float get_yaw_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i, float offset){
    double x = msg->pose[i].orientation.x;
    double y = msg->pose[i].orientation.y;
    double z = msg->pose[i].orientation.z;
    double w = msg->pose[i].orientation.w;

    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    float floatYaw = (float) yaw;

    return floatYaw + offset; //due to model orientation
  }

  float get_xPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    return msg->pose[i].position.x;
  }

  float get_yPos_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    return msg->pose[i].position.y;
  }

  float get_yawRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
    return msg->twist[i].angular.z;
  }

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
  variables(ros::NodeHandle* nodehandle):node(*nodehandle)
  {

    runController = false;
    heaveHo = 0;
    driveType = "two-to-one";
	leftRight = 1;

    //subscribers
    sub1 = node.subscribe("/dd_robot/joint_states", 1, &variables::joint_states_Callback, this, ros::TransportHints().tcpNoDelay());
    sub2 = node.subscribe("/gazebo/link_states", 1, &variables::link_states_Callback, this, ros::TransportHints().tcpNoDelay());
    sub3 = node.subscribe("/driveType", 1, &variables::driveType_Callback, this, ros::TransportHints().tcpNoDelay());

	sub6 = node.subscribe("/sensor_prismatic_joint_1", 1, &variables::sensor_1_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub7 = node.subscribe("/sensor_prismatic_joint_2", 1, &variables::sensor_2_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub8 = node.subscribe("/sensor_prismatic_joint_3", 1, &variables::sensor_3_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub9 = node.subscribe("/sensor_prismatic_joint_4", 1, &variables::sensor_4_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub10 = node.subscribe("/sensor_prismatic_joint_5", 1, &variables::sensor_5_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub11 = node.subscribe("/sensor_prismatic_joint_6", 1, &variables::sensor_6_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub12 = node.subscribe("/sensor_prismatic_joint_7", 1, &variables::sensor_7_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub13 = node.subscribe("/sensor_prismatic_joint_8", 1, &variables::sensor_8_1_Callback, this, ros::TransportHints().tcpNoDelay());
	sub14 = node.subscribe("/sensor_prismatic_joint_9", 1, &variables::sensor_9_1_Callback, this, ros::TransportHints().tcpNoDelay());

    //timer callbacks
    timer = node.createTimer(ros::Duration(1 / controllerLoopRate), &variables::controllerCallback, this);
    timer_inch = node.createTimer(ros::Duration(0.25), &variables::heaveHoCallback, this);



  }
  
  float getLeftRight(){
	  return leftRight;
  }

  bool checkRunController(){
    return runController;
  }

  void setRunControllerFalse(){
    runController = false;
  }

  std::string getDriveType(){
    return driveType;
  }
  
  float get_heaveHo(){
	  return heaveHo;
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
  
  float getConnectorForce(int i){
	  return connectorForce[i];
  }
  
  
	void sensor_1_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[0] = x / 5.0;
			x = 0;
			count = 0;
			//ROS_INFO_STREAM(connectorForce[0] << ", " << connectorForce[1] << ", " << connectorForce[2]  << ", " << connectorForce[3] << ", " << connectorForce[4]);
			//ROS_INFO_STREAM(v1.getConnectorForce(0) << ", " << v1.getConnectorForce(1) << ", " << v1.getConnectorForce(2)  << ", " << v1.getConnectorForce(3) << ", " << v1.getConnectorForce(4));
		}
		//float z = msg->wrench.force.z;
		//float y = msg->wrench.force.y;
		//float x = msg->wrench.force.x;
		
	}

	void sensor_2_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[1] = x / 5.0;
			x = 0;
			count = 0;
		}
	}

	void sensor_3_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[2] = x / 5.0;
			x = 0;
			count = 0;
		}
	}
	void sensor_4_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[3] = x / 5.0;
			x = 0;
			count = 0;
		}
	}

	void sensor_5_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[4] = x / 5.0;
			x = 0;
			count = 0;
		}
	}

	void sensor_6_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[5] = x / 5.0;
			x = 0;
			count = 0;
		}
	}
	
	
	void sensor_7_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[6] = x / 5.0;
			x = 0;
			count = 0;
		}
	}	
	
	
	void sensor_8_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[7] = x / 5.0;
			x = 0;
			count = 0;
		}
	}	
	
	
	void sensor_9_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
		static float count = 0;
		static float x = 0;
		x = x + msg->wrench.force.x;
		
		if(count < 4){
			count++;
		}else{
			connectorForce[8] = x / 5.0;
			x = 0;
			count = 0;
		}
	}	
	
	void driveType_Callback(const std_msgs::String::ConstPtr & msg){
		//reverse = msg->data;
		driveType = msg->data;
	}
	
	void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
		//ROS_INFO_STREAM("joint_states_Callback");
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
						//wheelAngularSpeed[i][j][k] = (msg->position[wheelOrder[i][j][k]] - wheelAngularPosition[i][j][k]) / wheel_duration;
						//wheelAngularPosition[i][j][k] = msg->position[wheelOrder[i][j][k]];
						wheelAngularSpeed[i][j][k] = msg->velocity[wheelOrder[i][j][k]];
					}
				}
			}
		}else { //if this is the first time through this callback
			first = false;
			for(size_t h=0; h<names.size(); ++h){
				//iterate through the body and axle joints
				for(size_t i=0; i<(notN-1); ++i){
					for(size_t j=0; j<2; ++j){
						//first look for the body joints
						if(names[h].compare(bodyJoint_name[i][j]) == 0){
							bodyJointOrder[i][j] = h;
							ROS_INFO_STREAM("bodyJointZ " << i << ", " << j << " found");
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
								ROS_INFO_STREAM("wheel joint " << i << ", " << j << ", " << k << " found");
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
              ROS_INFO_STREAM("axle " << i << ", " << j << " found");

            }
          }
        }
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

  void controllerCallback(const ros::TimerEvent&)
  {
    //ROS_INFO_STREAM("controllerCallback");
    runController = true;
  }

  void heaveHoCallback(const ros::TimerEvent&){
    heaveHo = heaveHo + 0.25;
	if(driveType == "inch1" && heaveHo >= 2.5){
		heaveHo = 0;
		leftRight = -leftRight;
	}else if((driveType == "inch2"|| driveType == "two-to-one") && heaveHo >= 3.75){
		heaveHo = 0;
		leftRight = -leftRight;
	}
	//ROS_INFO_STREAM(heaveHo);

  }

};









int main(int argc, char **argv){
	ros::init(argc, argv, "controller_train5");
	ros::NodeHandle node_handle;

	// create class
	variables v1(&node_handle);


	ros::Publisher pub_motorSetSpeed = node_handle.advertise<std_msgs::Float32MultiArray>("/motorSetSpeed", 5);
	ros::Publisher pub_prismaticSetForce = node_handle.advertise<std_msgs::Float32MultiArray>("/prismaticSetForce", 5);


	int waypointNumber = 1;

	ROS_INFO_STREAM("#########################################################");
	// I am not sure why this works but the following block allows this node to run without hanging
	// It likely has something to do with sufficient time for callbacks to populate
	// a single sleep() of longer duration does not perform the same
	// #######################################
	ros::spinOnce();
	ros::Duration(1.0).sleep(); //allows callbacks to populate variables before calculations
	ros::spinOnce();
	ros::Duration(1.0).sleep(); //allows callbacks to populate variables before calculations
	ros::spinOnce();
	// #######################################



	while(ros::ok()){

		float motorSetSpeed[notN][2][2];
		float prismaticSetForce[notN-1];
		static float axleIntegral[notN][2];
		static float oldAxleError[notN][2];

		if(v1.checkRunController()){ //this is the navigation controller that sets desired wheel speeds
			v1.setRunControllerFalse();

			std_msgs::Float32MultiArray msg_wheelSpeed;
			msg_wheelSpeed.data.resize(notN*2*2);
			
			std_msgs::Float32MultiArray msg_prismaticSetForce;
			msg_prismaticSetForce.data.resize(notN-1);

			static int count = 0;
			count++;
			


			float distanceToGoal;
			//check if goal has been reached
			distanceToGoal = pow(pow(goalX[waypointNumber] - v1.getCurrentX(0),2) + pow(goalY[waypointNumber] - v1.getCurrentY(0),2),0.5);
			//ROS_INFO_STREAM("body " << i << " distance to goal " << distanceToGoal);
			if (distanceToGoal < 5.0) {
				waypointNumber++;
				//ROS_INFO_STREAM("###### Waypoint Reached");

				if (waypointNumber > 35) {
					waypointNumber = 1;
					ROS_INFO_STREAM("######## End of waypoint list reached");
					break;
				}
			}
			
			if(count ==20){
				//ROS_INFO_STREAM(v1.getCurrentX(0) << ", " <<  v1.getCurrentY(0) << ", " << distanceToGoal);
				//ROS_INFO_STREAM(connectorForce[0] << ", " << connectorForce[1] << ", " << connectorForce[2] << ", " << connectorForce[3] << ", " << connectorForce[4]);
				
				count = 0;
			}

			if (v1.getDriveType().compare("normal") == 0) {
				
				float V = 0; //rad/s
				float upperLimit = noLoadSpeed;
				float lowerLimit = -noLoadSpeed;

				//ROS_INFO_STREAM(v1.getCurrentX(0) << ", " <<  v1.getCurrentY(0));
				//plot three segment center positions
				//ROS_INFO_STREAM(v1.getCurrentX(0) << ", " <<  v1.getCurrentY(0) << ", " <<  v1.getCurrentX(1) << ", " <<  v1.getCurrentY(1) << ", " << v1.getCurrentX(2) << ", " <<  v1.getCurrentY(2));
				//plot three segment axle angles
				//ROS_INFO_STREAM(v1.getAxleJoint(0,0) << ", " << v1.getAxleJoint(0,1) << ", " << v1.getAxleJoint(1,0) << ", " << v1.getAxleJoint(1,1) << ", " << v1.getAxleJoint(2,0) << ", " << v1.getAxleJoint(2,1));
				//ROS_INFO_STREAM("######################");
				for(size_t i=0; i<notN; ++i){
					float W1 = 0;
					float W2 = 0;
					float P1 = 4.0;
					float P2 = 4.0;
					float P3 = 4.0;
					float P4 = 4.0;
					float D1 = 0.0;
					V = constrain(v1.getBodyLonRate(i) + 0.2, 0, 2.05) / wheelRadius;
					V = 1.05 / wheelRadius;
					if(i == 0){ //if lead segment
						
						//V = 1.58 / wheelRadius;

						//ROS_INFO_STREAM("goal is: " << goalX[waypointNumber] << ", " << goalY[waypointNumber]);
						float bearingFromGoal = atan2(goalY[waypointNumber] - goalY[waypointNumber-1],goalX[waypointNumber] - goalX[waypointNumber-1]);
						float goalDist = distanceToGoal - navigationRadius;
						float targetX = goalX[waypointNumber] - goalDist*cos(bearingFromGoal);
						float targetY = goalY[waypointNumber] - goalDist*sin(bearingFromGoal);
						//the corresponding point on the line
						float shadowX = goalX[waypointNumber] - distanceToGoal*cos(bearingFromGoal);
						float shadowY = goalY[waypointNumber] - distanceToGoal*sin(bearingFromGoal);
						float latDistance = pow(pow(shadowX - v1.getCurrentX(i),2) + pow(shadowY - v1.getCurrentY(i),2),0.5);
						float headingToTarget = atan2(targetY - v1.getCurrentY(i),targetX - v1.getCurrentX(i));

						/*
						//this is for driving straight then starting a turn
						if (count < 100) {
						  headingToTarget = 0;
						}
						else if (count == 100) {
						  headingToTarget = 0.75;
						  ROS_INFO_STREAM("initiating turn");
						}
						else{
						  headingToTarget = 0.75;
						}
						*/
						float frontAxleGoal = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);
						//ROS_INFO_STREAM("axle angles: " << v1.getAxleJoint(i,0) << ", " << v1.getAxleJoint(i,1));
						float rearAxleGoal = 0;

						/*
						// this is for driving the vehicle is a circle
						//0.03 for a goal angle yields a 15 meter turn radius
						//0.02 for a goal angle yields a 24 meter turn radius
						if (count < 100) {
						  frontAxleGoal = 0;
						  rearAxleGoal  = 0;
						}
						else if (count == 100) {
						  frontAxleGoal = 0.75;
						  rearAxleGoal = -0.75;
						  ROS_INFO_STREAM("initiating turn");
						}
						else{
						  frontAxleGoal = 0.75;
						  rearAxleGoal = -0.75;
						}
						*/
						

						float frontAxleError = headingDifference(frontAxleGoal,v1.getAxleJoint(i,0));
						axleIntegral[i][0] = axleIntegral[i][0] + frontAxleError;
						W1 = P1 * frontAxleError;// + D1*v1.getAxleJointRate(i,0);

						float rearAxleError  = headingDifference(-frontAxleGoal,v1.getAxleJoint(i,1));
						axleIntegral[i][0] = axleIntegral[i][0] + rearAxleError;
						W2 = P2*rearAxleError;// + D1*v1.getAxleJointRate(i,1);// + I*axleIntegral[i][1];
				
						/*
						
						ROS_INFO_STREAM("################");
						ROS_INFO_STREAM(motorSetSpeed[i][0][0] << ", " << motorSetSpeed[i][0][1]);
						ROS_INFO_STREAM(motorSetSpeed[i][1][0] << ", " << motorSetSpeed[i][1][1]);
						ROS_INFO_STREAM(V);
						ROS_INFO_STREAM(frontAxleError << ", " << rearAxleError);
						ROS_INFO_STREAM(axleIntegral[i][0] << ", " << axleIntegral[i][1]);
						*/
						
					}else{ //if follower segment
						//V = constrain(v1.getBodyLonRate(i) + 0.5, 0, 1.03) / wheelRadius;
						//ROS_INFO_STREAM(180*v1.getBodyJointZ(i-1,0)/pi);
						//ROS_INFO_STREAM(180*v1.getBodyJointZ(i-1,1)/pi);

						//normal path following
						float frontAxleError = headingDifference(v1.getBodyJointZ(i-1,1), v1.getAxleJoint(i,0));
						//float frontAxleError = headingDifference(0, v1.getAxleJoint(i,0));
						axleIntegral[i][0] = axleIntegral[i][0] + frontAxleError;
						W1 = P3*frontAxleError + D1*(v1.getAxleJointRate(i,0)-v1.getBodyJointZRate(i-1,1));//  + I*axleIntegral[i][0];
						//make the rear axle seek the opposite angle of the front
						float rearAxleError  = headingDifference(-v1.getBodyJointZ(i-1,1),v1.getAxleJoint(i,1));
						//float rearAxleError  = headingDifference(0,v1.getAxleJoint(i,1));
						//rearAxleError  = headingDifference(0,axleJoint[i][1]);
						axleIntegral[i][1] = axleIntegral[i][1] + rearAxleError;
						W2 = P4*rearAxleError + D1*v1.getAxleJointRate(i,1);// + I*axleIntegral[i][1];

						/*
						//driving connector angles to zero
						float frontAxleError = headingDifference(v1.getBodyJointZ(i-1,1)/2, v1.getAxleJoint(i,0));
						float frontAxleError2 = headingDifference(0,-v1.getBodyJointZ(i-1,1)); //want to keep connectors in line with bodies

						float W1 = P3*frontAxleError + D1*(frontAxleError - oldAxleError[i][0]);
						oldAxleError[i][0] = frontAxleError;

						float rearAxleError  = headingDifference(v1.getBodyJointZ(i,0)/2, v1.getAxleJoint(i,1));
						float rearAxleError2 = headingDifference(0,-v1.getBodyJointZ(i,0));

						float W2 = P3*rearAxleError + D1*(rearAxleError - oldAxleError[i][1]);
						oldAxleError[i][1] = rearAxleError;
						*/
					}
					motorSetSpeed[i][0][0] = constrain(V - W1,lowerLimit,upperLimit);
					motorSetSpeed[i][0][1] = constrain(V + W1,lowerLimit,upperLimit);
					motorSetSpeed[i][1][0] = constrain(V - W2,lowerLimit,upperLimit);
					motorSetSpeed[i][1][1] = constrain(V + W2,lowerLimit,upperLimit);
					
					//ROS_INFO_STREAM(v1.getAxleJoint(i,0) << ", " << v1.getAxleJoint(i,1));
					//ROS_INFO_STREAM(motorSetSpeed[i][0][0] << ", " <<	motorSetSpeed[i][0][1]<< ", " <<	motorSetSpeed[i][1][0] << ", " <<	motorSetSpeed[i][1][1]);

					//axleIntegral[i][0] = constrain(axleIntegral[i][0],-20,20);
					//axleIntegral[i][1] = constrain(axleIntegral[i][1],-20,20);

				}

				//ROS_INFO_STREAM(v1.getCurrentX(0) << ", " << v1.getCurrentY(0) << ", " << v1.getCurrentX(1) << ", " << v1.getCurrentY(1) << ", " << v1.getCurrentX(2) << ", " << v1.getCurrentY(2) << ", " << v1.getCurrentX(3) << ", " << v1.getCurrentY(3) << ", " << v1.getCurrentX(4) << ", " << v1.getCurrentY(4));
				//ROS_INFO_STREAM("#########");
				for(int i=0; i<notN; ++i){
					for(int j=0; j<2; ++j){
						for(int k=0; k<2; ++k){
							msg_wheelSpeed.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k];
						}
						//ROS_INFO_STREAM("(" << motorSetSpeed[i][j][0] << ", " << v1.getWheelAngularSpeed(i,j,0) << "), (" << motorSetSpeed[i][j][1] << ", "<< v1.getWheelAngularSpeed(i,j,1)<< ")");
					}
				}
				pub_motorSetSpeed.publish(msg_wheelSpeed); //publish the motor setpoints
			}
			else if (v1.getDriveType().compare("two_segment_zero_force") == 0) {
				
				float V = 0;
				float upperLimit = noLoadSpeed;
				float lowerLimit = -noLoadSpeed;

				for(size_t i=0; i<notN; ++i){
					float W1 = 0;
					float W2 = 0;
					float P1 = 4.0;
					float P2 = 4.0;
					float P3 = 4.0;
					float P4 = 4.0;
					float D1 = 0.0;
					V = constrain(v1.getBodyLonRate(i) + 0.2, 0, 1.8) / wheelRadius;
					
					if(i == 0){ //if lead segment	
						float forceError = 0 - - v1.getConnectorForce(0);
						static float forceErrorIntegral = 0;
						forceErrorIntegral = constrain(forceErrorIntegral + forceError*(1/controllerLoopRate),-250,250);
						V = constrain(0.001*forceError +  0.01 * forceErrorIntegral,0,2) / wheelRadius;
						ROS_INFO_STREAM(v1.getConnectorForce(0) << ", " << forceErrorIntegral << ", "<< V);
						
						float frontAxleError = headingDifference(0,v1.getAxleJoint(i,0));
						axleIntegral[i][0] = axleIntegral[i][0] + frontAxleError;
						W1 = P1 * frontAxleError;// + D1*v1.getAxleJointRate(i,0);

						float rearAxleError  = headingDifference(0,v1.getAxleJoint(i,1));
						axleIntegral[i][0] = axleIntegral[i][0] + rearAxleError;
						W2 = P2*rearAxleError;// + D1*v1.getAxleJointRate(i,1);// + I*axleIntegral[i][1];						
					}else{ //if follower segment			
						
						float frontAxleError = headingDifference(v1.getBodyJointZ(i-1,1), v1.getAxleJoint(i,0));	
						W1 = P3*frontAxleError + D1*(v1.getAxleJointRate(i,0)-v1.getBodyJointZRate(i-1,1));//  + I*axleIntegral[i][0];
						//make the rear axle seek the opposite angle of the front
						float rearAxleError  = headingDifference(-v1.getBodyJointZ(i-1,1),v1.getAxleJoint(i,1));
						W2 = P4*rearAxleError + D1*v1.getAxleJointRate(i,1);// + I*axleIntegral[i][1];

					}
		
					motorSetSpeed[i][0][0] = constrain(V - W1,lowerLimit,upperLimit);
					motorSetSpeed[i][0][1] = constrain(V + W1,lowerLimit,upperLimit);
					motorSetSpeed[i][1][0] = constrain(V - W2,lowerLimit,upperLimit);
					motorSetSpeed[i][1][1] = constrain(V + W2,lowerLimit,upperLimit);
					
					//ROS_INFO_STREAM(v1.getAxleJoint(i,0) << ", " << v1.getAxleJoint(i,1));
					//ROS_INFO_STREAM(motorSetSpeed[i][0][0] << ", " <<	motorSetSpeed[i][0][1]<< ", " <<	motorSetSpeed[i][1][0] << ", " <<	motorSetSpeed[i][1][1]);

					//axleIntegral[i][0] = constrain(axleIntegral[i][0],-20,20);
					//axleIntegral[i][1] = constrain(axleIntegral[i][1],-20,20);

				}

				//ROS_INFO_STREAM(v1.getCurrentX(0) << ", " << v1.getCurrentY(0) << ", " << v1.getCurrentX(1) << ", " << v1.getCurrentY(1) << ", " << v1.getCurrentX(2) << ", " << v1.getCurrentY(2) << ", " << v1.getCurrentX(3) << ", " << v1.getCurrentY(3) << ", " << v1.getCurrentX(4) << ", " << v1.getCurrentY(4));
				//ROS_INFO_STREAM("#########");
				for(int i=0; i<notN; ++i){
					for(int j=0; j<2; ++j){
						for(int k=0; k<2; ++k){
							msg_wheelSpeed.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k];
						}
						//ROS_INFO_STREAM("(" << motorSetSpeed[i][j][0] << ", " << v1.getWheelAngularSpeed(i,j,0) << "), (" << motorSetSpeed[i][j][1] << ", "<< v1.getWheelAngularSpeed(i,j,1)<< ")");
					}
				}
				pub_motorSetSpeed.publish(msg_wheelSpeed); //publish the motor setpoints
			}
			
			else if (v1.getDriveType().compare("three_segment_zero_net_force") == 0) {
				
				float V = 0;
				float upperLimit = noLoadSpeed;
				float lowerLimit = -noLoadSpeed;
				cout << v1.getConnectorForce(0) << ", " << v1.getConnectorForce(1) << ", ";
				for(size_t i=0; i<notN; ++i){
					float W1 = 0;
					float W2 = 0;
					float P1 = 4.0;
					float P2 = 4.0;
					float P3 = 4.0;
					float P4 = 4.0;
					float D1 = 0.0;
					V = constrain(v1.getBodyLonRate(i) + 0.2, 0, 1.8) / wheelRadius;
					
					if(i == 0){ //if lead segment	
						float forceError = 125 - - v1.getConnectorForce(0);
						static float forceErrorIntegral = 0;
						forceErrorIntegral = constrain(forceErrorIntegral + forceError*(1/controllerLoopRate),-250,250);
						V = constrain(0.0005*forceError +  0.01 * forceErrorIntegral,0,2) / wheelRadius;
						
						
						float frontAxleError = headingDifference(0,v1.getAxleJoint(i,0));
						axleIntegral[i][0] = axleIntegral[i][0] + frontAxleError;
						W1 = P1 * frontAxleError;// + D1*v1.getAxleJointRate(i,0);

						float rearAxleError  = headingDifference(0,v1.getAxleJoint(i,1));
						axleIntegral[i][1] = axleIntegral[i][1] + rearAxleError;
						W2 = P2*rearAxleError;// + D1*v1.getAxleJointRate(i,1);// + I*axleIntegral[i][1];			
						
					}else if(i == 1){
						float forceError = 75 - - v1.getConnectorForce(1);
						static float forceErrorIntegral = 0;
						forceErrorIntegral = constrain(forceErrorIntegral + forceError*(1/controllerLoopRate),-250,250);
						V = constrain(0.0005*forceError +  0.01 * forceErrorIntegral,0,2) / wheelRadius;
						
						
						float frontAxleError = headingDifference(0,v1.getAxleJoint(i,0));
						axleIntegral[i][0] = axleIntegral[i][0] + frontAxleError;
						W1 = P1 * frontAxleError;// + D1*v1.getAxleJointRate(i,0);

						float rearAxleError  = headingDifference(0,v1.getAxleJoint(i,1));
						axleIntegral[i][1] = axleIntegral[i][1] + rearAxleError;
						W2 = P2*rearAxleError;// + D1*v1.getAxleJointRate(i,1);// + I*axleIntegral[i][1];	
						
					}else{ //if follower segment			
						
						float frontAxleError = headingDifference(v1.getBodyJointZ(i-1,1), v1.getAxleJoint(i,0));	
						W1 = P3*frontAxleError + D1*(v1.getAxleJointRate(i,0)-v1.getBodyJointZRate(i-1,1));//  + I*axleIntegral[i][0];
						//make the rear axle seek the opposite angle of the front
						float rearAxleError  = headingDifference(-v1.getBodyJointZ(i-1,1),v1.getAxleJoint(i,1));
						W2 = P4*rearAxleError + D1*v1.getAxleJointRate(i,1);// + I*axleIntegral[i][1];

					}
					cout << V << ", ";
					motorSetSpeed[i][0][0] = constrain(V - W1,lowerLimit,upperLimit);
					motorSetSpeed[i][0][1] = constrain(V + W1,lowerLimit,upperLimit);
					motorSetSpeed[i][1][0] = constrain(V - W2,lowerLimit,upperLimit);
					motorSetSpeed[i][1][1] = constrain(V + W2,lowerLimit,upperLimit);
					
					//ROS_INFO_STREAM(v1.getAxleJoint(i,0) << ", " << v1.getAxleJoint(i,1));
					//ROS_INFO_STREAM(motorSetSpeed[i][0][0] << ", " <<	motorSetSpeed[i][0][1]<< ", " <<	motorSetSpeed[i][1][0] << ", " <<	motorSetSpeed[i][1][1]);

					//axleIntegral[i][0] = constrain(axleIntegral[i][0],-20,20);
					//axleIntegral[i][1] = constrain(axleIntegral[i][1],-20,20);

				}
				ROS_INFO_STREAM(v1.getBodyLonRate(0));

				//ROS_INFO_STREAM(v1.getCurrentX(0) << ", " << v1.getCurrentY(0) << ", " << v1.getCurrentX(1) << ", " << v1.getCurrentY(1) << ", " << v1.getCurrentX(2) << ", " << v1.getCurrentY(2) << ", " << v1.getCurrentX(3) << ", " << v1.getCurrentY(3) << ", " << v1.getCurrentX(4) << ", " << v1.getCurrentY(4));
				//ROS_INFO_STREAM("#########");
				for(int i=0; i<notN; ++i){
					for(int j=0; j<2; ++j){
						for(int k=0; k<2; ++k){
							msg_wheelSpeed.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k];
						}
						//ROS_INFO_STREAM("(" << motorSetSpeed[i][j][0] << ", " << v1.getWheelAngularSpeed(i,j,0) << "), (" << motorSetSpeed[i][j][1] << ", "<< v1.getWheelAngularSpeed(i,j,1)<< ")");
					}
				}
				pub_motorSetSpeed.publish(msg_wheelSpeed); //publish the motor setpoints
			}
			
			else if (v1.getDriveType().compare("reverse") == 0) {
				float V = constrain(v1.getBodyLonRate(notN-1) - 0.5, -1.03, 0) / wheelRadius; //rad/s
				float upperLimit = noLoadSpeed;
				float lowerLimit = -noLoadSpeed;
				float P1 = 5.0;
				float P2 = 5.0;
				float P3 = 5.0;

				for(size_t i=0; i<notN; ++i){
					float W1;
					float W2;
					if(i == notN-1){ //if new lead segment
						//drive straight backwards
						float frontAxleError = headingDifference(0,v1.getAxleJoint(i,0));
						W1 = P1 * frontAxleError;
						float rearAxleError  = headingDifference(0,v1.getAxleJoint(i,1));
						W2 = P2*rearAxleError;
					}else{ //if follower segment
						//follow "lead" straight back
						float frontAxleError = headingDifference(v1.getBodyJointZ(i,0), v1.getAxleJoint(i,0));
						W1 = P3*frontAxleError;
						float rearAxleError  = headingDifference(-v1.getBodyJointZ(i,0),v1.getAxleJoint(i,1));
						W2 = P3*rearAxleError;
					}
					motorSetSpeed[i][0][0] = constrain(V - W1,lowerLimit,upperLimit);
					motorSetSpeed[i][0][1] = constrain(V + W1,lowerLimit,upperLimit);
					motorSetSpeed[i][1][0] = constrain(V - W2,lowerLimit,upperLimit);
					motorSetSpeed[i][1][1] = constrain(V + W2,lowerLimit,upperLimit);
				}

				for(int i=0; i<notN; ++i){
					for(int j=0; j<2; ++j){
						for(int k=0; k<2; ++k){
							msg_wheelSpeed.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k];
						}
					}
				}
				pub_motorSetSpeed.publish(msg_wheelSpeed); //publish the motor setpoints
			}
			else if (v1.getDriveType().compare("inch1") == 0) {

				// this case should work for any numnber of segments
				float V = 0; //rad/s
				float upperLimit = noLoadSpeed;
				float lowerLimit = -noLoadSpeed;
				float P3 = 10;
				float driveSpeed = 0.6 / wheelRadius;
				float idleSpeed = 0.0 / wheelRadius;
				float anchorSpeed = 0.0 / wheelRadius;
				float heaveHo_L = v1.get_heaveHo();	
				float leftRight_L = v1.getLeftRight();				
				for(size_t i=0; i<notN; ++i){
					float frontAxleGoal;
					if(i == 0){ //if the first segment
						
						float bearingFromGoal = atan2(goalY[waypointNumber] - goalY[waypointNumber-1],goalX[waypointNumber] - goalX[waypointNumber-1]);
						float goalDist = distanceToGoal - navigationRadius;
						float targetX = goalX[waypointNumber] - goalDist*cos(bearingFromGoal);
						float targetY = goalY[waypointNumber] - goalDist*sin(bearingFromGoal);
						//the corresponding point on the line
						float shadowX = goalX[waypointNumber] - distanceToGoal*cos(bearingFromGoal);
						float shadowY = goalY[waypointNumber] - distanceToGoal*sin(bearingFromGoal);
						float latDistance = pow(pow(shadowX - v1.getCurrentX(i),2) + pow(shadowY - v1.getCurrentY(i),2),0.5);
						float headingToTarget = atan2(targetY - v1.getCurrentY(i),targetX - v1.getCurrentX(i));
						frontAxleGoal = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);

						if(heaveHo_L < 1){
							V = anchorSpeed;
							frontAxleGoal = leftRight_L*0.3;
						}else if(heaveHo_L < 1.25){
							V = idleSpeed;
							frontAxleGoal = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);
						}else if(heaveHo_L < 2.25){
							V = driveSpeed;
							frontAxleGoal = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);
						}else{
							V = idleSpeed;
							frontAxleGoal = leftRight_L*0.3;
						}
						frontAxleGoal = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);
						
						//ROS_INFO_STREAM(V - W1 << ", " << V + W1 << " left, right");
					}else if(i % 2 == 0){//every other segment (would include first)
						if(heaveHo_L < 1){
							V = anchorSpeed;
							frontAxleGoal = leftRight_L*0.3;
						}else if(heaveHo_L < 1.25){
							V = idleSpeed;
							frontAxleGoal = v1.getBodyJointZ(i-1,1);
						}else if(heaveHo_L < 2.25){
							V = driveSpeed;
							frontAxleGoal = v1.getBodyJointZ(i-1,1);
						}else{
							V = idleSpeed;
							frontAxleGoal = leftRight_L*0.3;
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);
						
						
					}else{
						if(heaveHo_L < 1){
							V = driveSpeed;
							frontAxleGoal = v1.getBodyJointZ(i-1,1);
						}else if(heaveHo_L < 1.25){
							V = idleSpeed;
							frontAxleGoal = -leftRight_L*0.3;
						}else if(heaveHo_L < 2.25){
							V = anchorSpeed;
							frontAxleGoal = -leftRight_L*0.3;
						}else{
							V = idleSpeed;
							frontAxleGoal = v1.getBodyJointZ(i-1,1);
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);

						
					}
					float frontAxleError = headingDifference(frontAxleGoal, v1.getAxleJoint(i,0));						
					float rearAxleError  = headingDifference(-frontAxleGoal,v1.getAxleJoint(i,1));
					float W1 = P3*frontAxleError;
					float W2 = P3*rearAxleError;
					motorSetSpeed[i][0][0] = constrain(V - W1,lowerLimit,upperLimit);
					motorSetSpeed[i][0][1] = constrain(V + W1,lowerLimit,upperLimit);
					motorSetSpeed[i][1][0] = constrain(V - W2,lowerLimit,upperLimit);
					motorSetSpeed[i][1][1] = constrain(V + W2,lowerLimit,upperLimit);
				}
				for(int i=0; i<notN; ++i){
					for(int j=0; j<2; ++j){
						for(int k=0; k<2; ++k){
							msg_wheelSpeed.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k];
						}
					}
				}
				
				float connectorForceP = 5;
				float speedLimit = 0.05;
				static float connectorForce[notN-1];
				//ROS_INFO_STREAM(v1.getBodyLonRate(0) << ", " << connectorForce[0]<< ", " << v1.getBodyLonRate(1));
				for(int i=0; i<notN-1; ++i){
					
					if(i % 2 == 0){	
						if(heaveHo_L < 1){ //ahead segment anchoring
							prismaticSetForce[i] = connectorForce[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),0,1);
							if(fabs(v1.getBodyLonRate(i)) > speedLimit){connectorForce[i] = constrain(connectorForce[i]*0.95,20,5000);}
							else{connectorForce[i] = constrain(connectorForce[i]*1.05,20,5000);}
						}else if(heaveHo_L < 1.25){
							prismaticSetForce[i] = 0;							
						}else if(heaveHo_L < 2.25){ //ahead segment moving forward
							prismaticSetForce[i] = -connectorForce[i]; //constrain(fabs(connectorForceP*v1.getBodyLonRate(i+1))-1,-1,0);
							if(fabs(v1.getBodyLonRate(i+1)) > speedLimit){connectorForce[i] = constrain(connectorForce[i]*0.95,20,5000);}
							else{connectorForce[i] = constrain(connectorForce[i]*1.05,20,5000);}
						}else{
							prismaticSetForce[i] = 0;							
						}
						
						
					}else{
						if(heaveHo_L < 1){ //ahead segment moving forward
							prismaticSetForce[i] = -connectorForce[i];//constrain(fabs(connectorForceP*v1.getBodyLonRate(i+1))-1,0,1);
							if(fabs(v1.getBodyLonRate(i+1)) > speedLimit){connectorForce[i] = constrain(connectorForce[i]*0.95,20,5000);}
							else{connectorForce[i] = constrain(connectorForce[i]*1.05,20,5000);}
						}else if(heaveHo_L < 1.25){
							prismaticSetForce[i] = 0;
							
						}else if(heaveHo_L < 2.25){ //ahead segment anchoring
							prismaticSetForce[i] = connectorForce[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),-1,0);
							if(fabs(v1.getBodyLonRate(i)) > speedLimit){connectorForce[i] = constrain(connectorForce[i]*0.95,20,5000);}
							else{connectorForce[i] = constrain(connectorForce[i]*1.05,20,5000);}
						}else{
							prismaticSetForce[i] = 0;
							
						}
						
					}			
						
					//connectorForce[i] = connectorForce[i] + 1 - fabs(v1.getBodyLonRate(i)) - fabs(v1.getBodyLonRate(i+1));
					msg_prismaticSetForce.data[i] = prismaticSetForce[i];
				}
				
				
				pub_motorSetSpeed.publish(msg_wheelSpeed); //publish the motor setpoints
				pub_prismaticSetForce.publish(msg_prismaticSetForce); //publish the prismatic joint set position
				
			}
			else if (v1.getDriveType().compare("inch2") == 0) {
				
				
				// this case should work for any numnber of segments
				float V = 0; //rad/s
				float upperLimit = noLoadSpeed;
				float lowerLimit = -noLoadSpeed;
				float P3 = 10;
				float driveSpeed = 0.6 / wheelRadius;
				float idleSpeed = 0.0 / wheelRadius;
				float anchorSpeed = 0.01 / wheelRadius;
				float heaveHo_L = v1.get_heaveHo();	
				float leftRight_L = v1.getLeftRight();	
				int phase = 0;

				//determine what phase of the motion vehicle is in
				if(heaveHo_L < 1){
					phase = 0;
				}else if(heaveHo_L < 1.25){
					phase = 1;					
				}else if(heaveHo_L < 2.25){	
					phase = 2;
				}else if(heaveHo_L < 2.5){
					phase = 3;
				}else if(heaveHo_L < 3.5){
					phase = 4;
				}else if(heaveHo_L < 3.75){
					phase = 5;
				}
				
				for(size_t i=0; i<notN; ++i){
					float frontAxleGoal;
					//first go through the segments
					if(i == 0){ //if the first segment						
						float bearingFromGoal = atan2(goalY[waypointNumber] - goalY[waypointNumber-1],goalX[waypointNumber] - goalX[waypointNumber-1]);
						float goalDist = distanceToGoal - navigationRadius;
						float targetX = goalX[waypointNumber] - goalDist*cos(bearingFromGoal);
						float targetY = goalY[waypointNumber] - goalDist*sin(bearingFromGoal);
						//the corresponding point on the line
						float shadowX = goalX[waypointNumber] - distanceToGoal*cos(bearingFromGoal);
						float shadowY = goalY[waypointNumber] - distanceToGoal*sin(bearingFromGoal);
						float latDistance = pow(pow(shadowX - v1.getCurrentX(i),2) + pow(shadowY - v1.getCurrentY(i),2),0.5);
						float headingToTarget = atan2(targetY - v1.getCurrentY(i),targetX - v1.getCurrentX(i));
						frontAxleGoal = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);

						if(phase == 0 || phase == 1){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						//ROS_INFO_STREAM(V - W1 << ", " << V + W1 << " left, right");
					}else if(i % 3 == 0){//every third segment (would include first)
						if(phase == 0){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);
						//frontAxleGoal = leftRight_L*0.3;
					
					}else if(i % 3 == 1){
						if(phase == 2){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);
						//frontAxleGoal = leftRight_L*0.3;
						
						
					}else if(i % 3 == 2){
						if(phase == 4){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);						
					}
					
										
					float frontAxleError = headingDifference(frontAxleGoal, v1.getAxleJoint(i,0));						
					float rearAxleError  = headingDifference(-frontAxleGoal,v1.getAxleJoint(i,1));
					float W1 = P3*frontAxleError;
					float W2 = P3*rearAxleError;
					motorSetSpeed[i][0][0] = constrain(V - W1,lowerLimit,upperLimit);
					motorSetSpeed[i][0][1] = constrain(V + W1,lowerLimit,upperLimit);
					motorSetSpeed[i][1][0] = constrain(V - W2,lowerLimit,upperLimit);
					motorSetSpeed[i][1][1] = constrain(V + W2,lowerLimit,upperLimit);
				}
				for(int i=0; i<notN; ++i){
					for(int j=0; j<2; ++j){
						for(int k=0; k<2; ++k){
							msg_wheelSpeed.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k];
						}
					}
				}
				
				float connectorForceP = 5;
				float speedLimit = -0.05;
				float lowSpeedLimit = -0.01;
				static float connectorForce1[notN-1];
				static float connectorForce2[notN-1];
				//ROS_INFO_STREAM(v1.getBodyLonRate(0) << ", " << prismaticSetForce[0] << ", " << v1.getBodyLonRate(1) << ", " << prismaticSetForce[1] << ", " << v1.getBodyLonRate(2) << ", " << prismaticSetForce[2] << ", " << v1.getBodyLonRate(3) << ", " << prismaticSetForce[3] << ", " << v1.getBodyLonRate(4) << ", " << prismaticSetForce[4]  << ", " << v1.getBodyLonRate(5));
				for(int i=0; i<notN-1; ++i){
					//ROS_INFO_STREAM(prismaticSetForce[i]);
						
					if(i % 3 == 0){//every third connector (would include first)
						if(phase == 0){
							prismaticSetForce[i] = -connectorForce1[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),0,1);
							if(v1.getBodyLonRate(i+1) < speedLimit){connectorForce1[i] = constrain(connectorForce1[i]*0.95,20,5000);}
							else if(v1.getBodyLonRate(i+1) > lowSpeedLimit){connectorForce1[i] = constrain(connectorForce1[i]*1.05,20,5000);}
						}else if(phase == 2){
							prismaticSetForce[i] = connectorForce2[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),0,1);
							if(v1.getBodyLonRate(i) < speedLimit){connectorForce2[i] = constrain(connectorForce2[i]*0.95,20,5000);}
							else if(v1.getBodyLonRate(i) > lowSpeedLimit){connectorForce2[i] = constrain(connectorForce2[i]*1.05,20,5000);}
						} else{
							prismaticSetForce[i] = 0;
						}						
					
					}else if(i % 3 == 1){ //this is the last connector
						if(phase == 2){
							prismaticSetForce[i] = -connectorForce1[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),0,1);
							if(v1.getBodyLonRate(i+1) < speedLimit){connectorForce1[i] = constrain(connectorForce1[i]*0.95,20,5000);}
							else if(v1.getBodyLonRate(i+1) > lowSpeedLimit){connectorForce1[i] = constrain(connectorForce1[i]*1.05,20,5000);}
						}else if(phase == 4){
							prismaticSetForce[i] = connectorForce2[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),0,1);
							if(v1.getBodyLonRate(i) < speedLimit){connectorForce2[i] = constrain(connectorForce2[i]*0.95,20,5000);}
							else if(v1.getBodyLonRate(i) > lowSpeedLimit){connectorForce2[i] = constrain(connectorForce2[i]*1.05,20,5000);}
						} else{
							prismaticSetForce[i] = 0;
						}												
						
					}else if(i % 3 == 2){ 
						if(phase == 0){
							prismaticSetForce[i] = connectorForce1[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),0,1);
							if(v1.getBodyLonRate(i) < speedLimit){connectorForce1[i] = constrain(connectorForce1[i]*0.95,20,5000);}
							else if(v1.getBodyLonRate(i) > lowSpeedLimit){connectorForce1[i] = constrain(connectorForce1[i]*1.05,20,5000);}
						}else if(phase == 4){
							prismaticSetForce[i] = -connectorForce2[i];//constrain(1-fabs(connectorForceP*v1.getBodyLonRate(i)),0,1);
							if(v1.getBodyLonRate(i+1) < speedLimit){connectorForce2[i] = constrain(connectorForce2[i]*0.95,20,5000);}
							else if(v1.getBodyLonRate(i+1) > lowSpeedLimit){connectorForce2[i] = constrain(connectorForce2[i]*1.05,20,5000);}
						} else{
							prismaticSetForce[i] = 0;
						}	
										
					}					
						
					//connectorForce[i] = connectorForce[i] + 1 - fabs(v1.getBodyLonRate(i)) - fabs(v1.getBodyLonRate(i+1));
					msg_prismaticSetForce.data[i] = prismaticSetForce[i];
				}
				
				
				pub_motorSetSpeed.publish(msg_wheelSpeed); //publish the motor setpoints
				pub_prismaticSetForce.publish(msg_prismaticSetForce); //publish the prismatic joint set position
				
			}
			else if (v1.getDriveType().compare("two-to-one") == 0) {
							
				// this case should work for any number of segments
				float V = 0; //rad/s
				float upperLimit = noLoadSpeed;
				float lowerLimit = -noLoadSpeed;
				float P3 = 10;
				float linearDriveSpeed = 0.6;
				float driveSpeed = linearDriveSpeed / wheelRadius;
				float idleSpeed = 0.0 / wheelRadius;
				float anchorSpeed = 0.05 / wheelRadius;
				float heaveHo_L = v1.get_heaveHo();	
				int phase = 0;

				//determine what phase of the motion vehicle is in
				if(heaveHo_L < 1){
					phase = 0;
				}else if(heaveHo_L < 1.25){
					phase = 1;					
				}else if(heaveHo_L < 2.25){	
					phase = 2;
				}else if(heaveHo_L < 2.5){
					phase = 3;
				}else if(heaveHo_L < 3.5){
					phase = 4;
				}else if(heaveHo_L < 3.75){
					phase = 5;
				}
				
				for(size_t i=0; i<notN; ++i){
					float frontAxleGoal;
					//first go through the segments
					if(i == 0){ //if the first segment						
						float bearingFromGoal = atan2(goalY[waypointNumber] - goalY[waypointNumber-1],goalX[waypointNumber] - goalX[waypointNumber-1]);
						float goalDist = distanceToGoal - navigationRadius;
						float targetX = goalX[waypointNumber] - goalDist*cos(bearingFromGoal);
						float targetY = goalY[waypointNumber] - goalDist*sin(bearingFromGoal);
						//the corresponding point on the line
						float shadowX = goalX[waypointNumber] - distanceToGoal*cos(bearingFromGoal);
						float shadowY = goalY[waypointNumber] - distanceToGoal*sin(bearingFromGoal);
						float latDistance = pow(pow(shadowX - v1.getCurrentX(i),2) + pow(shadowY - v1.getCurrentY(i),2),0.5);
						float headingToTarget = atan2(targetY - v1.getCurrentY(i),targetX - v1.getCurrentX(i));
						frontAxleGoal = constrain(headingDifference(headingToTarget,v1.getAxleHeading(i,0)),-0.3,0.3);

						if(phase == 0 || phase == 1){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						//ROS_INFO_STREAM(V - W1 << ", " << V + W1 << " left, right");
					}else if(i % 3 == 0){//every third segment (would include first)
						if(phase == 0){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);
						
					
					}else if(i % 3 == 1){
						if(phase == 2){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);
						
						
						
					}else if(i % 3 == 2){
						if(phase == 4){
							V = driveSpeed;
						} else{
							V = anchorSpeed;
						}
						frontAxleGoal = v1.getBodyJointZ(i-1,1);						
					}
					
										
					float frontAxleError = headingDifference(frontAxleGoal, v1.getAxleJoint(i,0));						
					float rearAxleError  = headingDifference(-frontAxleGoal,v1.getAxleJoint(i,1));
					float W1 = P3*frontAxleError;
					float W2 = P3*rearAxleError;
					motorSetSpeed[i][0][0] = constrain(V - W1,lowerLimit,upperLimit);
					motorSetSpeed[i][0][1] = constrain(V + W1,lowerLimit,upperLimit);
					motorSetSpeed[i][1][0] = constrain(V - W2,lowerLimit,upperLimit);
					motorSetSpeed[i][1][1] = constrain(V + W2,lowerLimit,upperLimit);
				}
				for(int i=0; i<notN; ++i){
					for(int j=0; j<2; ++j){
						for(int k=0; k<2; ++k){
							msg_wheelSpeed.data[i*4 + j*2 + k] = motorSetSpeed[i][j][k];
						}
					}
				}
				
				float connectorForceP1 = 100;
				float connectorForceP2 = 300;

				//ROS_INFO_STREAM(v1.getBodyLonRate(0) << ", " << prismaticSetForce[0] << ", " << v1.getBodyLonRate(1) << ", " << prismaticSetForce[1] << ", " << v1.getBodyLonRate(2) << ", " << prismaticSetForce[2] << ", " << v1.getBodyLonRate(3) << ", " << prismaticSetForce[3] << ", " << v1.getBodyLonRate(4) << ", " << prismaticSetForce[4]  << ", " << v1.getBodyLonRate(5));
				ROS_INFO_STREAM(v1.getBodyLonRate(0) << ", " << prismaticSetForce[0] << ", " << v1.getBodyLonRate(1) << ", " << prismaticSetForce[1] << ", " << v1.getBodyLonRate(2));
	
				for(int i=0; i<notN-1; ++i){
					//ROS_INFO_STREAM(prismaticSetForce[i]);
					
					float M = 1.0;
					if((i == 0 || i == notN-2) && notN >= 4){
						M = 1.5;
					}else{
						M = 1;
					}
					
					if(i % 3 == 0){//every third connector (would include first)
						if(phase == 0){ 
							float error1 = 0 - v1.getBodyLonRate(i+1);	//stationary segment
							float error2 = linearDriveSpeed - v1.getBodyLonRate(i); //mobile segment
							prismaticSetForce[i] = -connectorForceP1*error1 - connectorForceP2*error2*M;
							
						}else if(phase == 2){							
							float error1 = 0 - v1.getBodyLonRate(i);	//stationary segment
							float error2 = linearDriveSpeed - v1.getBodyLonRate(i+1); //mobile segment
							prismaticSetForce[i] = connectorForceP1*error1 + connectorForceP2*error2;
							
						}else{
							prismaticSetForce[i] = 0;
						}						
					
					}else if(i % 3 == 1){
						if(phase == 2){			
							float error1 = 0 - v1.getBodyLonRate(i+1);	//stationary segment
							float error2 = linearDriveSpeed - v1.getBodyLonRate(i); //mobile segment						
							prismaticSetForce[i] = - connectorForceP1*error1 - connectorForceP2*error2;
						
						}else if(phase == 4){			
							float error1 = 0 - v1.getBodyLonRate(i);	//stationary segment
							float error2 = linearDriveSpeed - v1.getBodyLonRate(i+1); //mobile segment						
							prismaticSetForce[i] = connectorForceP1*error1 + connectorForceP2*error2*M;
						
						} else{
							prismaticSetForce[i] = 0;
						}												
						
					}else if(i % 3 == 2){ 
						if(phase == 4){		
							float error1 = 0 - v1.getBodyLonRate(i+1);	//stationary segment
							float error2 = linearDriveSpeed - v1.getBodyLonRate(i); //mobile segment
							prismaticSetForce[i] = - connectorForceP1*error1 - connectorForceP2*error2;
						
						}else if(phase == 0){							
							float error1 = 0 - v1.getBodyLonRate(i);	//stationary segment
							float error2 = linearDriveSpeed - v1.getBodyLonRate(i+1); //mobile segment
							prismaticSetForce[i] = connectorForceP1*error1 + connectorForceP2*error2*M;
						
						} else{
							prismaticSetForce[i] = 0;
						}	
										
					}					
						
					//connectorForce[i] = connectorForce[i] + 1 - fabs(v1.getBodyLonRate(i)) - fabs(v1.getBodyLonRate(i+1));
					msg_prismaticSetForce.data[i] = constrain(prismaticSetForce[i],-1000,1000);
				}
				
				
				pub_motorSetSpeed.publish(msg_wheelSpeed); //publish the motor setpoints
				pub_prismaticSetForce.publish(msg_prismaticSetForce); //publish the prismatic joint set position
				
			}
		}		
	
		ros::spinOnce();
	}
		
  
  ROS_INFO_STREAM("ROS not ok, node failed");
}
