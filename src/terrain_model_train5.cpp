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
const float loopRate = 2000;

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
    float x = msg->pose[i].orientation.x;
    float y = msg->pose[i].orientation.y;
    float z = msg->pose[i].orientation.z;
    float w = msg->pose[i].orientation.w;

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
  ros::Subscriber sub5;
  ros::Subscriber sub6;


  ros::Subscriber sub8;

  ros::Subscriber sub10;
  ros::Subscriber sub11;
  ros::Subscriber sub12;
  ros::Subscriber sub13;

  ros::Subscriber sub14;
  ros::Subscriber sub15;
  ros::Subscriber sub16;
  ros::Subscriber sub17;

  ros::Subscriber sub18;
  ros::Subscriber sub19;
  ros::Subscriber sub20;
  ros::Subscriber sub21;

  ros::Subscriber sub22;
  ros::Subscriber sub23;
  ros::Subscriber sub24;
  ros::Subscriber sub25;

  ros::Subscriber sub26;
  ros::Subscriber sub27;
  ros::Subscriber sub28;
  ros::Subscriber sub29;
public:
  variables(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    runTerrainModel = false;


    //topics subscribed to
    sub4 = nh_.subscribe("/motorTorque", 1, &variables::motorTorque_Callback, this,  ros::TransportHints().tcpNoDelay());
    sub5 = nh_.subscribe("/dd_robot/joint_states", 1, &variables::joint_states_Callback, this,  ros::TransportHints().tcpNoDelay());
    sub6 = nh_.subscribe("/gazebo/link_states", 1, &variables::link_states_Callback, this,  ros::TransportHints().tcpNoDelay());


    sub8 = nh_.subscribe("/terrain_coefficient", 1, &variables::terrain_coefficient_Callback, this);

    sub10 = nh_.subscribe("/bumper_wheel_1_1_1", 1, &variables::bumper_wheel_1_1_1_Callback, this);
    sub11 = nh_.subscribe("/bumper_wheel_1_1_2", 1, &variables::bumper_wheel_1_1_2_Callback, this);
    sub12 = nh_.subscribe("/bumper_wheel_1_2_1", 1, &variables::bumper_wheel_1_2_1_Callback, this);
    sub13 = nh_.subscribe("/bumper_wheel_1_2_2", 1, &variables::bumper_wheel_1_2_2_Callback, this);

    sub14 = nh_.subscribe("/bumper_wheel_2_1_1", 1, &variables::bumper_wheel_2_1_1_Callback, this);
    sub15 = nh_.subscribe("/bumper_wheel_2_1_2", 1, &variables::bumper_wheel_2_1_2_Callback, this);
    sub16 = nh_.subscribe("/bumper_wheel_2_2_1", 1, &variables::bumper_wheel_2_2_1_Callback, this);
    sub17 = nh_.subscribe("/bumper_wheel_2_2_2", 1, &variables::bumper_wheel_2_2_2_Callback, this);

    sub18 = nh_.subscribe("/bumper_wheel_3_1_1", 1, &variables::bumper_wheel_3_1_1_Callback, this);
    sub19 = nh_.subscribe("/bumper_wheel_3_1_2", 1, &variables::bumper_wheel_3_1_2_Callback, this);
    sub20 = nh_.subscribe("/bumper_wheel_3_2_1", 1, &variables::bumper_wheel_3_2_1_Callback, this);
    sub21 = nh_.subscribe("/bumper_wheel_3_2_2", 1, &variables::bumper_wheel_3_2_2_Callback, this);


    sub22 = nh_.subscribe("/bumper_wheel_4_1_1", 1, &variables::bumper_wheel_4_1_1_Callback, this);
    sub23 = nh_.subscribe("/bumper_wheel_4_1_2", 1, &variables::bumper_wheel_4_1_2_Callback, this);
    sub24 = nh_.subscribe("/bumper_wheel_4_2_1", 1, &variables::bumper_wheel_4_2_1_Callback, this);
    sub25 = nh_.subscribe("/bumper_wheel_4_2_2", 1, &variables::bumper_wheel_4_2_2_Callback, this);

    sub26 = nh_.subscribe("/bumper_wheel_5_1_1", 1, &variables::bumper_wheel_5_1_1_Callback, this);
    sub27 = nh_.subscribe("/bumper_wheel_5_1_2", 1, &variables::bumper_wheel_5_1_2_Callback, this);
    sub28 = nh_.subscribe("/bumper_wheel_5_2_1", 1, &variables::bumper_wheel_5_2_1_Callback, this);
    sub29 = nh_.subscribe("/bumper_wheel_5_2_2", 1, &variables::bumper_wheel_5_2_2_Callback, this);



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

  void bumper_wheel_1_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[0][0][0] = force_from_wheel(msg);
    contactNormal[0][0][0] = contactNormal_from_wheel(msg);
  }
  void bumper_wheel_1_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[0][0][1] = force_from_wheel(msg);
  }
  void bumper_wheel_1_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[0][1][0] = force_from_wheel(msg);
  }
  void bumper_wheel_1_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[0][1][1] = force_from_wheel(msg);
  }

  void bumper_wheel_2_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[1][0][0] = force_from_wheel(msg);
  }
  void bumper_wheel_2_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[1][0][1] = force_from_wheel(msg);
  }
  void bumper_wheel_2_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[1][1][0] = force_from_wheel(msg);
  }
  void bumper_wheel_2_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[1][1][1] = force_from_wheel(msg);
  }

  void bumper_wheel_3_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[2][0][0] = force_from_wheel(msg);
  }
  void bumper_wheel_3_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[2][0][1] = force_from_wheel(msg);
  }
  void bumper_wheel_3_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[2][1][0] = force_from_wheel(msg);
  }
  void bumper_wheel_3_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[2][1][1] = force_from_wheel(msg);
  }

  void bumper_wheel_4_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[3][0][0] = force_from_wheel(msg);
  }
  void bumper_wheel_4_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[3][0][1] = force_from_wheel(msg);
  }
  void bumper_wheel_4_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[3][1][0] = force_from_wheel(msg);
  }
  void bumper_wheel_4_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[3][1][1] = force_from_wheel(msg);
  }

  void bumper_wheel_5_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[4][0][0] = force_from_wheel(msg);
  }
  void bumper_wheel_5_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[4][0][1] = force_from_wheel(msg);
  }
  void bumper_wheel_5_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[4][1][0] = force_from_wheel(msg);
  }
  void bumper_wheel_5_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
    normalForce[4][1][1] = force_from_wheel(msg);
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
            wheelAngularSpeed[i][j][k] = msg->velocity[order[i][j][k]];
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


  void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
      const std::vector<std::string> &names = msg->name;

      static bool first = true;
      static int axleOrder[notN][2];
      static int wheelOrder[notN][2][2];
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
              //ROS_INFO_STREAM("wheel " << wheelOrder[i][j][k]  << ": " << wheelLonSpeed[i][j][k]);
              //wheelLatSpeed[i][j][k] = get_latSpeed_from_link_states(msg,wheelOrder[i][j][k],axleHeading[i][j]);
              wheelYaw[i][j][k] = axleHeading[i][j];
              wheelYawRate[i][j][k] = get_wheelYawRate_from_link_states(msg,wheelOrder[i][j][k]);
            }
          }
        }

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

  void motorTorque_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(int i=0; i<notN; i++){
      for(int j=0; j<2; j++){
        for(size_t k=0; k<2; ++k){
          motorTorque[i][j][k] = msg->data[i*4 + j*2 + k];
        }
      }
    }
  }


  void terrain_coefficient_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    for(int i=0; i<notN; i++){
      for(int j=0; j<2; j++){
        for(int k=0; k<2; k++){
          Xt[i][j][k] = msg->data[(4*i)+(2*j)+k];
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


	ros::init(argc, argv, "terrain_model");
	ros::NodeHandle node_handle;

  variables v1(&node_handle);


  ros::Publisher pub_wheelForcesX = node_handle.advertise<std_msgs::Float64MultiArray>("/wheelForcesX", 1);
  ros::Publisher pub_wheelForcesY = node_handle.advertise<std_msgs::Float64MultiArray>("/wheelForcesY", 1);
  ros::Publisher pub_wheelTorques = node_handle.advertise<std_msgs::Float64MultiArray>("/wheelTorques", 1);

  //services called
  /*
  ros::ServiceClient wrenchClient = node_handle.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
	gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
	gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

  ros::ServiceClient jointEffortClient = node_handle.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort::Request apply_joint_effort_req;
	gazebo_msgs::ApplyJointEffort::Response apply_joint_effort_resp;

  apply_wrench_req.wrench.force.z = 0.0;
  apply_wrench_req.wrench.torque.x = 0.0;
  apply_wrench_req.wrench.torque.y = 0.0;


  ros::ServiceClient pauseSim = node_handle.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  std_srvs::Empty pauseSrv;

  ros::ServiceClient unpauseSim = node_handle.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  std_srvs::Empty unpauseSrv;
  */


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

      /*
      //calculate the loop time and use it to apply the forces and torques
      static ros::Time start_time = ros::Time::now();
      ros::Duration loopTime = ros::Time::now() - start_time;
      start_time = ros::Time::now();
      //ROS_INFO_STREAM(loopTime);

      apply_wrench_req.duration = loopTime;
      apply_joint_effort_req.duration = loopTime;
      pauseSim.call(pauseSrv);
      */



      std_msgs::Float64MultiArray msg_forcesX;
      msg_forcesX.data.resize(notN*2*2);

      std_msgs::Float64MultiArray msg_forcesY;
      msg_forcesY.data.resize(notN*2*2);

      std_msgs::Float64MultiArray msg_torques;
      msg_torques.data.resize(notN*2*2);


      //ROS_INFO_STREAM("msgs created");

      for(int i=0; i<notN; ++i){
        for(int j=0; j<2; ++j){
          for(int k=0; k<2; ++k){

            float COEFF;
            float wheelSlip;
            float frictionTorque;
            float terrainResistanceForce;
            float latTerrainResistanceForce;
            float compactionResistanceTorque;
            float terrainResistanceTorque;
            float yawResistanceTorque;
            float forceSlip;

            float Xt = v1.getXt(i,j,k);
            float normalForce = v1.getNormalForce(i,j,k);
            float wheelAngularSpeed = v1.getWheelAngularSpeed(i,j,k);
            float wheelLonSpeed = v1.getWheelLonSpeed(i,j,k);
            float contactNormal = v1.getContactNormal(i,j,k);

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //calculate slip
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            const float deadBand = 0.025;
            //if speed and wheel speed very small
            if(fabs(wheelRadius*wheelAngularSpeed) < deadBand && fabs(wheelLonSpeed) < deadBand){
              wheelSlip = 0;
            }
            //if moving backwards slower than wheels
            else if(wheelLonSpeed < 0 && wheelRadius*wheelAngularSpeed < wheelLonSpeed){
              wheelSlip = -fabs(wheelLonSpeed - wheelRadius*wheelAngularSpeed) / fabs(wheelRadius*wheelAngularSpeed);
            }
            //if moving backwards faster than wheels
            else if(wheelLonSpeed < 0 && wheelRadius*wheelAngularSpeed > wheelLonSpeed){
              wheelSlip = fabs(wheelRadius*wheelAngularSpeed - wheelLonSpeed) / fabs(wheelLonSpeed);
            }
            //if moving forward faster than wheels
            else if(wheelLonSpeed > 0 && wheelRadius*wheelAngularSpeed < wheelLonSpeed){
              wheelSlip = -fabs(wheelLonSpeed - wheelRadius*wheelAngularSpeed) / fabs(wheelLonSpeed);
            }
            //if moving forward slower than wheels
            else if(wheelLonSpeed > 0 && wheelRadius*wheelAngularSpeed > wheelLonSpeed){
              wheelSlip = fabs(wheelRadius*wheelAngularSpeed - wheelLonSpeed) / fabs(wheelRadius*wheelAngularSpeed);
            }
            wheelSlip = constrain(wheelSlip,-1,1);

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //calculate terrain resistance torque
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            compactionResistanceTorque = -coeffCompactionResistanceTorque*normalForce*sign(wheelLonSpeed)*fabs(Xt - 1);

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //calculate terrain resistance forces
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            COEFF = coeffTerrainResistanceForce*(1 + 5*fabs(Xt - 1)) ;//0.2;
            //see equation 2.18 in thesis proposal
            //signSpeed prevents rapid switching between full forward and full backward values
            //terrainResistanceForce = -COEFF*signDoubleBand(wheelLonSpeed,0,0.0001)*normalForce;
            terrainResistanceForce = -COEFF*sign(wheelLonSpeed)*normalForce - (28.8/4);
            //ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce);

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //frictionTorque
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            frictionTorque = -coeffFrictionTorque*wheelAngularSpeed;

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //calculate lateralTerrainResistanceForce
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //latTerrainResistanceForce = -coeffLatTerrainResistanceForce*sign(v1.getWheelLatSpeed(i,j,k))*normalForce;
            latTerrainResistanceForce = 0;

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //calculate yaw resistance torque
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            COEFF = coeffYawResistanceTorque*(1 + 10*fabs(Xt - 1)) ;
            yawResistanceTorque = -COEFF*signSpeed(v1.getWheelYawRate(i,j,k))*normalForce;

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            // calculate force slip
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            //double B = 2.3*Xt;
            //double C = 2.6/pow(Xt,0.5);
            //Equation from https://en.wikipedia.org/wiki/Hans_B._Pacejka
            //Pacejka, H. B. (2012). Tire and vehicle dynamics. Besselink, Igo (3rd ed.). Oxford: Butterworth-Heinemann. p. 165. ISBN 978-0-08-097016-5. OCLC 785829133
            float D = coeffForceSlip*Xt*normalForce;
            //const double E = 1.0;
            //forceSlip[i][j][k] = D*sin(C*atan(B*wheelSlip - E*(B*wheelSlip - atan(B*wheelSlip))));

            //curve most recently used to calculate coefficients
            //forceSlip = D*tanh(wheelSlip*19*Xt)* fabs(tanh(20*wheelSlip*Xt));

            //curve most recently used to create figures
            forceSlip = D*tanh(wheelSlip*19*pow(Xt-0.4,2))* fabs(tanh(20*wheelSlip*Xt));


            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            // calculate torque from force slip
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            terrainResistanceTorque = -forceSlip*wheelRadius;


            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            // Apply forces and torques to wheel and joint
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            float wheelYaw = v1.getWheelYaw(i,j,k);
            /*
            apply_wrench_req.wrench.force.x = (forceSlip + terrainResistanceForce)*cos(wheelYaw) + -latTerrainResistanceForce*sin(wheelYaw); //this is good;
            apply_wrench_req.wrench.force.y = (forceSlip + terrainResistanceForce)*sin(wheelYaw) + latTerrainResistanceForce*cos(wheelYaw);//this is good
            apply_wrench_req.wrench.torque.z = yawResistanceTorque;
            apply_wrench_req.body_name = model_name[i][j][k];
            // call apply body wrench service
            wrenchClient.call(apply_wrench_req, apply_wrench_resp);


            apply_joint_effort_req.effort = v1.getMotorTorque(i,j,k) + compactionResistanceTorque + terrainResistanceTorque + frictionTorque;
            apply_joint_effort_req.joint_name = joint_name[i][j][k];
            // call apply body wrench service
            jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);
            */


            msg_forcesX.data[i*4 + j*2 + k] = constrain((forceSlip + terrainResistanceForce)*cos(wheelYaw) + -latTerrainResistanceForce*sin(wheelYaw),-1000,1000);
            msg_forcesY.data[i*4 + j*2 + k] = constrain((forceSlip + terrainResistanceForce)*sin(wheelYaw) + latTerrainResistanceForce*cos(wheelYaw),-1000,1000);
            msg_torques.data[i*4 + j*2 + k] = constrain(v1.getMotorTorque(i,j,k) + compactionResistanceTorque + terrainResistanceTorque + frictionTorque,-100,100);

            //ROS_INFO_STREAM(msg_forcesX.data[i*4 + j*2 + k] << ", " << msg_forcesY.data[i*4 + j*2 + k] << ", " << msg_torques.data[i*4 + j*2 + k]);

            /*
            if (isnan(msg_forcesX.data[i*4 + j*2 + k])) {
              ROS_INFO_STREAM("msg_forcesX was nan");
            }
            msg_forcesX.data[i*4 + j*2 + k] = 0;

            if (isnan(msg_forcesY.data[i*4 + j*2 + k])) {
              ROS_INFO_STREAM("msg_forcesY was nan");
            }
            msg_forcesY.data[i*4 + j*2 + k] = 0;

            if (isnan(msg_torques.data[i*4 + j*2 + k])) {
              ROS_INFO_STREAM("msg_torques was nan");
            }
            msg_torques.data[i*4 + j*2 + k] = 0;
            */

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            // print values for front left wheel of lead segment
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if(i == 0 && j == 0 && k ==0 && count >= 50){
              count = 0;
              /*
              ROS_INFO_STREAM(j);
              ROS_INFO_STREAM("Xt is " << Xt);
              ROS_INFO_STREAM("wheelYaw is " << v1.getWheelYaw(i,j,k));
              ROS_INFO_STREAM("normalForce is " << normalForce);
              ROS_INFO_STREAM("wheelAngularSpeed*R is " << wheelAngularSpeed*wheelRadius);
              ROS_INFO_STREAM("frictionTorque is " << frictionTorque);
              ROS_INFO_STREAM("wheelLonSpeed is " << wheelLonSpeed);
              ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce);
              ROS_INFO_STREAM("compactionResistanceTorque is " << compactionResistanceTorque);
              ROS_INFO_STREAM("terrainResistanceTorque is " << terrainResistanceTorque);
              //ROS_INFO_STREAM("wheelLatSpeed is " << v1.getWheelLatSpeed(i,j,k));
              ROS_INFO_STREAM("latTerrainResistanceForce is " << latTerrainResistanceForce);
              ROS_INFO_STREAM("wheelYawRate is " << v1.getWheelYawRate(i,j,k));
              ROS_INFO_STREAM("yawResistanceTorque is " << yawResistanceTorque);
              ROS_INFO_STREAM("wheelSlip is " << wheelSlip);
              ROS_INFO_STREAM("forceSlip is " << forceSlip);
              ROS_INFO_STREAM("motorTorque is " << v1.getMotorTorque(i,j,k));

              ROS_INFO_STREAM(" ");
              */
              //plot lon speed, wheel speed, wheel torque, slip
              ROS_INFO_STREAM(wheelLonSpeed << ", " << wheelAngularSpeed*wheelRadius << ", " << v1.getMotorTorque(i,j,k) << ", " << wheelSlip);
            }
          }
        }
      }

      //ROS_INFO_STREAM("calculations performed");
      pub_wheelForcesX.publish(msg_forcesX);
      pub_wheelForcesY.publish(msg_forcesY);
      pub_wheelTorques.publish(msg_torques);
      //ROS_INFO_STREAM("Values published");
      //unpauseSim.call(unpauseSrv);
  	}
  }
	ROS_INFO_STREAM(" oops");
  return 0;
}
