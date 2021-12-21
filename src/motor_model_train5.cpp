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


const int N = 5;
//notN is defined in terrain_library.h


//11 is front left, 12 front right, 21 back left...




const float loopRate = 2000;


const float slope = -13.0507;// Nm / (rad/sec)
const float noLoadSpeed = 13.326; //rad/sec
const float wheelRadius = 0.28;


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

class variables{
private:
    float wheelAngularSpeed[notN][2][2];
    float motorSetPoint[notN][2][2];
    float motorTorque[notN][2][2];
    bool runMotorModel;
    ros::NodeHandle nh_;

    ros::Subscriber sub5;
    ros::Subscriber sub6;
    ros::Timer timer;

public:
  variables(ros::NodeHandle* nodehandle):nh_(*nodehandle)
  {
    //subscribers
     sub5 = nh_.subscribe("/dd_robot/joint_states", 5, &variables::joint_states_Callback, this, ros::TransportHints().tcpNoDelay());
     sub6 = nh_.subscribe("/motorSetPoint", 5, &variables::motorSetPoint_Callback, this,  ros::TransportHints().tcpNoDelay());

    //timer callbacks
    timer = nh_.createTimer(ros::Duration(1 / loopRate), &variables::motorTorqueTimerCallback, this);

    for(size_t i=0; i<notN; ++i){
      for(size_t j=0; j<2; ++j){
        for(size_t k=0; k<2; ++k){
          motorSetPoint[i][j][k] = 0;
          wheelAngularSpeed[i][j][k] = 0;
          motorTorque[i][j][k] = 0;


        }
      }
    }

  }

    float getMotorSetPoint(int i,int j,int k){
      return motorSetPoint[i][j][k];
    }

    float getWheelAngularSpeed(int i,int j,int k){
      return wheelAngularSpeed[i][j][k];
    }

    bool getRunMotorModel(){
      return runMotorModel;
    }

    void setRunMotorModelTrue(){
      runMotorModel = true;
    }

    void setRunMotorModelFalse(){
      runMotorModel = false;
    }

    void setMotorTorque(int i, int j, int k, float torque){
      motorTorque[i][j][k] = torque;
    }

    float getMotorTorque(int i, int j, int k){
      return motorTorque[i][j][k];
    }


    void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
      const std::vector<std::string> &names = msg->name;
      static bool first = true;
      static int order[notN][2][2];
      if(!first){
        for(size_t i=0; i<notN; ++i){
          for(size_t j=0; j<2; ++j){
            for(size_t k=0; k<2; ++k){
              float data = msg->velocity[order[i][j][k]];
              if (isnan(data)) {
                data = 0;
              }
              wheelAngularSpeed[i][j][k] = data;
              if(i == 0 && j == 0 && k == 0){
                ROS_INFO_STREAM("wheelAngularSpeed updated " << wheelAngularSpeed[i][j][k]);
              }

            }
          }
        }
        //ROS_INFO_STREAM("joint states updated");
      }else{
        for(size_t h=0; h<names.size(); ++h){
          for(size_t i=0; i<notN; ++i){
            for(size_t j=0; j<2; ++j){
              for(size_t k=0; k<2; ++k){
                if(names[h].compare(joint_name[i][j][k]) == 0){
                  order[i][j][k] = h; //record which position the joint is
                }
              }
            }
          }
        }
        ROS_INFO_STREAM("order of joints found");
        first = false;
      }
    }

    void motorSetPoint_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
      //ROS_INFO_STREAM("#########################");
      for(int i=0; i<notN; i++){
        for(int j=0; j<2; j++){
          for(size_t k=0; k<2; ++k){
            motorSetPoint[i][j][k] = msg->data[i*4 + j*2 + k];

            if(i == 0 && j == 0 && k == 0){
              //ROS_INFO_STREAM("motor set point updated " << motorSetPoint[i][j][k]);
            }
          }
        }
      }

    }

    void motorTorqueTimerCallback(const ros::TimerEvent&)
    {
      //ROS_INFO("Callback 1 triggered");
      setRunMotorModelTrue();
    }

};


int main(int argc, char **argv){
  ROS_INFO_STREAM("init node_handle");
	ros::init(argc, argv, "motor_model_train5");
	ros::NodeHandle node_handle;

  // create class
  variables v1(&node_handle);


  //published topics
  ros::Publisher pub_motorTorque = node_handle.advertise<std_msgs::Float64MultiArray>("/motorTorque", 5);


  //topics subscribed to



  ros::Duration(0.5).sleep(); //allows callbacks to populate variables before calculations

	ros::Rate loop_rate(loopRate);





  while(ros::ok()){
    ros::spinOnce();
    if(v1.getRunMotorModel()){
      v1.setRunMotorModelFalse();
      //ROS_INFO_STREAM("########################");

      std_msgs::Float64MultiArray msg;
      msg.data.resize(notN*2*2);

      for(int i=0; i<notN; ++i){
        for(int j=0; j<2; ++j){
          for(int k=0; k<2; ++k){
            float motorTorque = constrain(0.0*v1.getMotorTorque(i,j,k) + 1.0*slope*(v1.getWheelAngularSpeed(i,j,k) - v1.getMotorSetPoint(i,j,k)*noLoadSpeed),-88,88);
            if (isnan(motorTorque)) {motorTorque = 0;}
            v1.setMotorTorque(i,j,k,motorTorque);
            msg.data[i*4 + j*2 + k] = motorTorque;
            if(i == 0 && j == 0 && k == 0){
              //ROS_INFO_STREAM("motorTorque: " << motorTorque);
            }
          }
        }
      }
      //ROS_INFO_STREAM("motor torque " << msg.data[0]);
      pub_motorTorque.publish(msg);
  	}
  }

	ROS_INFO_STREAM(" oops");
  return 0;
}
