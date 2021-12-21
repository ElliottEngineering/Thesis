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


#include "terrain_library.h"

//const int N = 10;


const float speedControllerLoopRate = 1000; //actual roboteq controller is 1000hz
const float wheelRadius = 0.28;

const float noLoadSpeed = 13.326; //rad/sec

const float pi = 3.141592;
const float pi2 = 1.570796;




class variables{
private:
	ros::NodeHandle nh_;




	float motorSetSpeed[notN][2][2];
	float motorSetPoint[notN][2][2];

	float wheelAngularSpeed[notN][2][2];
	float wheelAngularPosition[notN][2][2];

	float prismaticJointPosition[notN - 1];
	float prismaticSetPosition[notN - 1];
	float prismaticJointSpeed[notN - 1];
	float prismaticSetSpeed[notN - 1];


	bool runSpeedControl;

	ros::Subscriber sub1;
	ros::Subscriber sub2;
	ros::Subscriber sub3;



	ros::Timer timer2;






public:
	variables(ros::NodeHandle* nodehandle):nh_(*nodehandle)
	{


		runSpeedControl = false;
		//subscribers
		sub1 = nh_.subscribe("/dd_robot/joint_states", 1, &variables::joint_states_Callback, this, ros::TransportHints().tcpNoDelay());
		sub2 = nh_.subscribe("/motorSetSpeed", 1, &variables::motorSetSpeed_Callback, this, ros::TransportHints().tcpNoDelay());
		sub3 = nh_.subscribe("/prismaticSetPosition", 1, &variables::prismaticSetPosition_Callback, this, ros::TransportHints().tcpNoDelay());

		//timer callbacks

		timer2 = nh_.createTimer(ros::Duration(1 / speedControllerLoopRate), &variables::speedControlCallback, this);
	}
  
	float getPrismaticSetPosition(int i){
		return prismaticSetPosition[i];
	}
  
  float getPrismaticSetSpeed(int i){
	  return prismaticSetSpeed[i];
  }
  
  float getPrismaticJointSpeed(int i){
	  return prismaticJointSpeed[i];
  }
  
  float getPrismaticJointPosition(int i){
	  return prismaticJointPosition[i];
  }

  float getMotorSetSpeed(int i, int j, int k){
	return motorSetSpeed[i][j][k];
  }


  bool checkRunSpeedControl(){
    return runSpeedControl;
  }

  void setRunSpeedControlFalse(){
    runSpeedControl = false;
  }


  float getWheelAngularSpeed(int i, int j, int k){
    return wheelAngularSpeed[i][j][k];
  }


  float getMotorSetPoint(int i,int j,int k){
    return motorSetPoint[i][j][k];
  }

  void setMotorSetPoint(int i,int j,int k, float setpoint){
    motorSetPoint[i][j][k] = setpoint;
  }
  
	void prismaticSetPosition_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
		for(int i=0; i<notN; i++){
			prismaticSetPosition[i] = msg->data[i];
			prismaticSetSpeed[i] = msg->data[i]; //same value to both, change later
		}
	}

  void motorSetSpeed_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    //ROS_INFO_STREAM("#########################");
    for(int i=0; i<notN; i++){
      for(int j=0; j<2; j++){
        for(size_t k=0; k<2; ++k){
          motorSetSpeed[i][j][k] = msg->data[i*4 + j*2 + k];
		  if(i == 4 && j == 1 && (k == 0 || k == 1)){
            //ROS_INFO_STREAM("motor set speed updated " << motorSetSpeed[i][j][k]);
			//ROS_INFO_STREAM(motorSetSpeed[i][j][k]);
          }
        }
      }
    }

  }

  void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
    //ROS_INFO_STREAM("joint_states_Callback");
    static bool first = true;
    static int axleOrder[notN][2];
    static int wheelOrder[notN][2][2];
	static int prismaticJointOrder[notN - 1];

    const std::vector<std::string> &names = msg->name;


    if(!first){
		for(size_t i=0; i<notN - 1 ; ++i){
			prismaticJointPosition[i] = msg->position[prismaticJointOrder[i]];
			prismaticJointSpeed[i] = msg->velocity[prismaticJointOrder[i]];
		}
		

		for(size_t i=0; i<notN; ++i){
			for(size_t j=0; j<2; ++j){
				for(size_t k=0; k<2; ++k){
					wheelAngularSpeed[i][j][k] = msg->velocity[wheelOrder[i][j][k]];
				}
			}
		}
    }else{
		 first = false;
		 for(size_t h=0; h<names.size(); ++h){
			//iterate through the body  joints
			for(size_t i=0; i<notN - 1 ; ++i){
				if(names[h].compare(prismatic_name[i]) == 0){
					prismaticJointOrder[i] = h;
					ROS_INFO_STREAM("prismatic joint " << i  << " found");
				}
			}

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

  void speedControlCallback(const ros::TimerEvent&)
  {
    //ROS_INFO_STREAM("speedControlCallback");
    runSpeedControl = true;
  }

};



int main(int argc, char **argv){
  ros::init(argc, argv, "controller_train5");
  ros::NodeHandle node_handle;

  // create class
  variables v1(&node_handle);

  ros::Publisher pub_motorSetPoint = node_handle.advertise<std_msgs::Float32MultiArray>("/motorSetPoint", 5);
  ros::Publisher pub_prismaticSetPoint = node_handle.advertise<std_msgs::Float32MultiArray>("/prismaticSetPoint", 5);


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
    ros::Duration(1.0).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  // #######################################


  while(ros::ok()){
	  
	  
    //let ROS do its background stuff


    //ROS_INFO_STREAM("Inside While Loop");



    if(v1.checkRunSpeedControl()){ //This is the closed loop speed controller
		//ROS_INFO_STREAM("runSpeedControl");
		v1.setRunSpeedControlFalse();

		std_msgs::Float32MultiArray msg;
		msg.data.resize(notN*2*2);

		std_msgs::Float32MultiArray msg_prismaticSetPoint;
		msg_prismaticSetPoint.data.resize(notN - 1);


		static float speedErrorIntegral[notN][2][2];
		static int count2;
		count2++;



		//run the speed controller for each wheel
		const float P = 0.0;
		const float I = 0.6;


		// This is in order to have segments start incrementally, not all at once
		int segmentNumber = notN;
		/*
		
		static int segmentNumber = 0;
		static ros::Time startTime = ros::Time::now();
		if(segmentNumber < notN){
			if(ros::Time::now() - startTime > ros::Duration(1,0)){
				segmentNumber++;
				//ROS_INFO_STREAM("increment segment number");
				startTime = ros::Time::now();
			}
		}
		*/
		
		
		
		for(int i=0; i<segmentNumber; ++i){
			for(int j=0; j<2; ++j){
				for(int k=0; k<2; ++k){
					float motorSetSpeed_l = v1.getMotorSetSpeed(i,j,k);

					float wheelAngularSpeed = v1.getWheelAngularSpeed(i,j,k);
					float speedDiff = motorSetSpeed_l - wheelAngularSpeed;
					speedErrorIntegral[i][j][k] = constrain(speedErrorIntegral[i][j][k] + (speedDiff/speedControllerLoopRate),-0.5,0.5);

					float newSetPoint = constrain(motorSetSpeed_l/noLoadSpeed + P*speedDiff + I*speedErrorIntegral[i][j][k],-1,1);

					// comment out the following line to return to closed loop mode
					//newSetPoint = 1.5*motorSetSpeed_l/noLoadSpeed; //this line makes speed controller run open loop
					if (isnan(newSetPoint)) {
						newSetPoint = 0;
					}
					v1.setMotorSetPoint(i,j,k,newSetPoint);

					msg.data[i*4 + j*2 + k] = newSetPoint;
					//msg.data[i*4 + j*2 + k] = 0.25;
					//msg.data[i*4 + j*2 + k] = motorSetSpeed_l / noLoadSpeed; //open loop
					
					
					if (false && i ==0 && j ==1 && k==1 && count2 > 50) {
						count2 = 0;
						ROS_INFO_STREAM("#########################################################");
						ROS_INFO_STREAM("wheelAngularSpeed: " << wheelAngularSpeed);
						ROS_INFO_STREAM("motorSetSpeed_l: " << motorSetSpeed_l);
						ROS_INFO_STREAM("speedErrorIntegral[i][j][k]: " << speedErrorIntegral[i][j][k]);
						ROS_INFO_STREAM("speedDiff: " << speedDiff);
						ROS_INFO_STREAM("newSetPoint: " << newSetPoint);
					}

					if (i ==0 && j ==0 && k==0) {
						//ROS_INFO_STREAM(msg.data[i*4 + j*2 + k]);
					}

				}
			}
		}
	  
	  //work through the prismatic joints
	  for(int i=0; i<notN-1; ++i){
		  msg_prismaticSetPoint.data[i] = v1.getPrismaticSetPosition(i);
		  //msg_prismaticSetPoint.data[i] = constrain(2000*(v1.getPrismaticSetSpeed(i)  - v1.getPrismaticJointSpeed(i)),-1000,1000);
		  
	  }
	  
		/*
		if(count2 == 50){
			count2 = 0;
			ROS_INFO_STREAM(1000*v1.getPrismaticJointPosition(0) << ", " << v1.getPrismaticJointSpeed(0));
		}*/
	  
      pub_motorSetPoint.publish(msg); //publish the motor setpoints
	  pub_prismaticSetPoint.publish(msg_prismaticSetPoint);
    }
    ros::spinOnce();
  }
  ROS_INFO_STREAM("ROS not ok, node failed");
}
