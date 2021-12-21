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
const double loopRate = 10;





const double pi = 3.141592;
const double pi2 = 1.570796;


/*

const std::string joint_name[N][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}},
                                  {{"joint_wheel_4_1_1","joint_wheel_4_1_2"},{"joint_wheel_4_2_1","joint_wheel_4_2_2"}},
                                  {{"joint_wheel_5_1_1","joint_wheel_5_1_2"},{"joint_wheel_5_2_1","joint_wheel_5_2_2"}}};




const std::string axle_name[N][2] = {{"train5::axle_1_1","train5::axle_1_2"},{"train5::axle_2_1","train5::axle_2_2"},{"train5::axle_3_1","train5::axle_3_2"},{"train5::axle_4_1","train5::axle_4_2"},{"train5::axle_5_1","train5::axle_5_2"}};



const std::string body_name[N] = {"train5::body_1","train5::body_2","train5::body_3","train5::body_4","train5::body_5"};
*/

const std::string model_name[N][2][2] = {{{"train5::wheel_1_1_1","train5::wheel_1_1_2"},{"train5::wheel_1_2_1","train5::wheel_1_2_2"}},
                                  {{"train5::wheel_2_1_1","train5::wheel_2_1_2"},{"train5::wheel_2_2_1","train5::wheel_2_2_2"}},
                                  {{"train5::wheel_3_1_1","train5::wheel_3_1_2"},{"train5::wheel_3_2_1","train5::wheel_3_2_2"}},
                                  {{"train5::wheel_4_1_1","train5::wheel_4_1_2"},{"train5::wheel_4_2_1","train5::wheel_4_2_2"}},
                                  {{"train5::wheel_5_1_1","train5::wheel_5_1_2"},{"train5::wheel_5_2_1","train5::wheel_5_2_2"}}};

class variables{
private:
  ros::NodeHandle nh_;

  double motorTorque[notN][2][2];
  double motorSpeed[notN][2][2];
  double wheelLonSpeed[notN][2][2];

  double get_axleHeading_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
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

  double get_lonSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i,double yaw){
    double xVel = msg->twist[i].linear.x;
    double yVel = msg->twist[i].linear.y;

    double lonVel = xVel*cos(yaw) + yVel*sin(yaw);
    return lonVel;
  }



  ros::Subscriber sub4;
  ros::Subscriber sub5;
  ros::Subscriber sub6;
public:
  variables(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    sub4 = nh_.subscribe("/motorTorque", 1, &variables::motorTorque_Callback, this);
    sub5 = nh_.subscribe("/dd_robot/joint_states", 10, &variables::joint_states_Callback, this);
    sub6 = nh_.subscribe("/gazebo/link_states", 10, &variables::link_states_Callback, this);

  }

  double getMotorTorque(int i, int j, int k){
    return motorTorque[i][j][k];
  }

  float getWheelLonSpeed(int i, int j, int k){
    return wheelLonSpeed[i][j][k];
  }

  float getMotorSpeed(int i, int j, int k){
    return motorSpeed[i][j][k];
  }

  void motorTorque_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(size_t i=0; i<notN; ++i){
      for(size_t j=0; j<2; ++j){
        for(size_t k=0; k<2; ++k){
          motorTorque[i][j][k] = msg->data[i*4 + j*2 + k];
        }
      }
    }
  }

  void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
      const std::vector<std::string> &names = msg->name;

      static bool first = true;
      static int axleOrder[notN][2];
      static int wheelOrder[notN][2][2];
      static double axleHeading[notN][2];
      if (!first) {
        //first get axle yaws
        for(size_t i=0; i<notN; ++i){
          for(size_t j=0; j<2; ++j){
            axleHeading[i][j] = get_axleHeading_from_link_states(msg,axleOrder[i][j]);
          }
        }
        //then go through for wheels
        for(size_t i=0; i<notN; ++i){
          for(size_t j=0; j<2; ++j){
            for(size_t k=0; k<2; ++k){
              wheelLonSpeed[i][j][k] = get_lonSpeed_from_link_states(msg,wheelOrder[i][j][k],axleHeading[i][j]);
              //ROS_INFO_STREAM(wheelLonSpeed[i][j][k]);
            }
          }
        }
        //ROS_INFO_STREAM("wheel speed callback: " << wheelLonSpeed[0][0][0]);

      }else{
        first = false;
        //first get axle yaws
        for(size_t h=0; h<names.size(); ++h){
          for(size_t i=0; i<notN; ++i){
            for(size_t j=0; j<2; ++j){
              if(names[h].compare(axle_name[i][j]) == 0){
                axleOrder[i][j] = h;
              }
            }
          }
        }
        //then go through for wheels
        for(size_t h=0; h<names.size(); ++h){
          for(size_t i=0; i<notN; ++i){
            for(size_t j=0; j<2; ++j){
              for(size_t k=0; k<2; ++k){
                if(names[h].compare(model_name[i][j][k]) == 0){
                  wheelOrder[i][j][k] = h;
                }
              }
            }
          }
        }
      }
  }


  void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
    const std::vector<std::string> &names = msg->name; //this might explode things
    //static double wheelPosition[notN][2][2];
    static bool first = true;
    static int order[notN][2][2];
    if(!first){
      for(size_t i=0; i<notN; ++i){
        for(size_t j=0; j<2; ++j){
          for(size_t k=0; k<2; ++k){
            motorSpeed[i][j][k] = msg->velocity[order[i][j][k]];
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
};

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_train5");
  ros::NodeHandle node_handle;


  variables v1(&node_handle);

  ros::Rate loop_rate(loopRate);
  double totalEnergy = 0;
  double totalDistance = 0;

  ros::spinOnce();
  ros::Duration(2.0).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  ROS_INFO_STREAM("#########################################################");

  while(ros::ok()){
    static ros::Time start_time = ros::Time::now();
    ros::Duration loopTime = ros::Time::now() - start_time;
    start_time = ros::Time::now();
    double loopTimeDouble = loopTime.toSec();

    double energy = 0;
    double speed = v1.getWheelLonSpeed(0,1,0);
    for(size_t i=0; i<notN; ++i){
      for(size_t j=0; j<2; ++j){
        for(size_t k=0; k<2; ++k){
          double motorEnergy = (v1.getMotorSpeed(i,j,k)*v1.getMotorTorque(i,j,k) * loopTimeDouble);

          if (motorEnergy < 0) {
            motorEnergy = motorEnergy * 0.50;
          }
          energy = energy + motorEnergy;
        }
      }
    }

    totalEnergy = totalEnergy + energy;
    totalDistance = totalDistance + (speed * loopTimeDouble);

    //ROS_INFO_STREAM("Joules " << energy);
    //ROS_INFO_STREAM("total kWh " << totalEnergy / 3.6e6);
    ROS_INFO_STREAM("wheel lon speed: " << v1.getWheelLonSpeed(0,1,0));
    ROS_INFO_STREAM("meters: " << totalDistance);
    ROS_INFO_STREAM("Joules: " << totalEnergy);
    ROS_INFO_STREAM("km / kWh: " << (speed * loopTimeDouble / energy) * 3600);

    //let ROS do its background stuff
    ros::spinOnce();
    loop_rate.sleep();
  }
}
