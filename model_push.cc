#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <cmath>

#include <tf/transform_datatypes.h>
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/WrenchStamped.h"




const int notN = 6;

const float slope = -13.0507;// Nm / (rad/sec)
const float noLoadSpeed = 13.326; //rad/sec
const float wheelRadius = 0.28;

//const float coeffFrictionTorque = 0.1;//0.1;
const float coeffOtherResistanceTorque = 0.32;
const float coeffTerrainResistanceForce = 0.08;
const float coeffForceSlip = 0.245;
const float coeffYawResistanceTorque = 0.001;
const float coeffLatTerrainResistanceForce = 0.1;


const std::string model_name[10][2][2] = {{{"train5::wheel_1_1_1","train5::wheel_1_1_2"},{"train5::wheel_1_2_1","train5::wheel_1_2_2"}},
																	{{"train5::wheel_2_1_1","train5::wheel_2_1_2"},{"train5::wheel_2_2_1","train5::wheel_2_2_2"}},
																	{{"train5::wheel_3_1_1","train5::wheel_3_1_2"},{"train5::wheel_3_2_1","train5::wheel_3_2_2"}},
																	{{"train5::wheel_4_1_1","train5::wheel_4_1_2"},{"train5::wheel_4_2_1","train5::wheel_4_2_2"}},
																	{{"train5::wheel_5_1_1","train5::wheel_5_1_2"},{"train5::wheel_5_2_1","train5::wheel_5_2_2"}},
																	{{"train5::wheel_6_1_1","train5::wheel_6_1_2"},{"train5::wheel_6_2_1","train5::wheel_6_2_2"}},
																	{{"train5::wheel_7_1_1","train5::wheel_7_1_2"},{"train5::wheel_7_2_1","train5::wheel_7_2_2"}},
																	{{"train5::wheel_8_1_1","train5::wheel_8_1_2"},{"train5::wheel_8_2_1","train5::wheel_8_2_2"}},
																	{{"train5::wheel_9_1_1","train5::wheel_9_1_2"},{"train5::wheel_9_2_1","train5::wheel_9_2_2"}},
																	{{"train5::wheel_10_1_1","train5::wheel_10_1_2"},{"train5::wheel_10_2_1","train5::wheel_10_2_2"}}};


/*
const std::string body_name[6] = {"train5::body_1","train5::body_2","train5::body_3","train5::body_4","train5::body_5","train5::body_6"};




const std::string joint_name[6][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}},
                                  {{"joint_wheel_4_1_1","joint_wheel_4_1_2"},{"joint_wheel_4_2_1","joint_wheel_4_2_2"}},
                                  {{"joint_wheel_5_1_1","joint_wheel_5_1_2"},{"joint_wheel_5_2_1","joint_wheel_5_2_2"}},
								  {{"joint_wheel_6_1_1","joint_wheel_6_1_2"},{"joint_wheel_6_2_1","joint_wheel_6_2_2"}}};

const std::string axle_name[6][2] = {{"train5::axle_1_1","train5::axle_1_2"},
									{"train5::axle_2_1","train5::axle_2_2"},
									{"train5::axle_3_1","train5::axle_3_2"},
									{"train5::axle_4_1","train5::axle_4_2"},
									{"train5::axle_5_1","train5::axle_5_2"},
									{"train5::axle_6_1","train5::axle_6_2"}};

const std::string prismatic_name[5] = {"prismatic_joint_1","prismatic_joint_2","prismatic_joint_3","prismatic_joint_4","prismatic_joint_5"};
*/

const std::string body_name[10] = {"train5::body_1","train5::body_2","train5::body_3","train5::body_4","train5::body_5","train5::body_6","train5::body_7","train5::body_8","train5::body_9","train5::body_10"};

const std::string prismatic_name[9] = {"prismatic_joint_1","prismatic_joint_2","prismatic_joint_3","prismatic_joint_4","prismatic_joint_5","prismatic_joint_6","prismatic_joint_7","prismatic_joint_8","prismatic_joint_9"};

const std::string axle_name[10][2] = {{"train5::axle_1_1","train5::axle_1_2"},
											{"train5::axle_2_1","train5::axle_2_2"},
											{"train5::axle_3_1","train5::axle_3_2"},
											{"train5::axle_4_1","train5::axle_4_2"},
											{"train5::axle_5_1","train5::axle_5_2"},
											{"train5::axle_6_1","train5::axle_6_2"},
											{"train5::axle_7_1","train5::axle_7_2"},
											{"train5::axle_8_1","train5::axle_8_2"},
											{"train5::axle_9_1","train5::axle_9_2"},
											{"train5::axle_10_1","train5::axle_10_2"}};

const std::string bodyJoint_name[9][2] =	 {{"body_joint_1_z","body_joint_12_z"},
											  {"body_joint_2_z","body_joint_22_z"},
											  {"body_joint_3_z","body_joint_32_z"},
											  {"body_joint_4_z","body_joint_42_z"},
											  {"body_joint_5_z","body_joint_52_z"},
											  {"body_joint_6_z","body_joint_62_z"},
											  {"body_joint_7_z","body_joint_72_z"},
											  {"body_joint_8_z","body_joint_82_z"},
											  {"body_joint_9_z","body_joint_92_z"}};

const std::string axleJoint_name[10][2] = {{"pivot_joint_1_1","pivot_joint_1_2"},
																{"pivot_joint_2_1","pivot_joint_2_2"},
																{"pivot_joint_3_1","pivot_joint_3_2"},
																{"pivot_joint_4_1","pivot_joint_4_2"},
																{"pivot_joint_5_1","pivot_joint_5_2"},
																{"pivot_joint_6_1","pivot_joint_6_2"},
																{"pivot_joint_7_1","pivot_joint_7_2"},
																{"pivot_joint_8_1","pivot_joint_8_2"},
																{"pivot_joint_9_1","pivot_joint_9_2"},
																{"pivot_joint_10_1","pivot_joint_10_2"}};

const std::string joint_name[10][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}},
                                  {{"joint_wheel_4_1_1","joint_wheel_4_1_2"},{"joint_wheel_4_2_1","joint_wheel_4_2_2"}},
                                  {{"joint_wheel_5_1_1","joint_wheel_5_1_2"},{"joint_wheel_5_2_1","joint_wheel_5_2_2"}},
								  {{"joint_wheel_6_1_1","joint_wheel_6_1_2"},{"joint_wheel_6_2_1","joint_wheel_6_2_2"}},
								  {{"joint_wheel_7_1_1","joint_wheel_7_1_2"},{"joint_wheel_7_2_1","joint_wheel_7_2_2"}},
								  {{"joint_wheel_8_1_1","joint_wheel_8_1_2"},{"joint_wheel_8_2_1","joint_wheel_8_2_2"}},
								  {{"joint_wheel_9_1_1","joint_wheel_9_1_2"},{"joint_wheel_9_2_1","joint_wheel_9_2_2"}},
								  {{"joint_wheel_10_1_1","joint_wheel_10_1_2"},{"joint_wheel_10_2_1","joint_wheel_10_2_2"}}};

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

    private:
      float forcesX[notN][2][2];
      float forcesY[notN][2][2];
      float torques[notN][2][2];
      ros::Subscriber sub1;
      ros::Subscriber sub2;
      ros::Subscriber sub3;
      ros::Subscriber sub4;
      ros::Subscriber sub5;

      //force sensors in connector links
      ros::Subscriber sub6;
      ros::Subscriber sub7;
      ros::Subscriber sub8;
      ros::Subscriber sub9;
	  ros::Subscriber sub9a;
	  ros::Subscriber sub9b;
	  ros::Subscriber sub9c;
	  ros::Subscriber sub9d;
	  ros::Subscriber sub9e;

      //contact sensors for normal force
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
	  
	  ros::Subscriber sub30;
      ros::Subscriber sub31;
      ros::Subscriber sub32;
      ros::Subscriber sub33;
	  
	  ros::Subscriber sub34;
      ros::Subscriber sub35;
      ros::Subscriber sub36;
      ros::Subscriber sub37;
	  
	  ros::Subscriber sub38;
      ros::Subscriber sub39;
      ros::Subscriber sub40;
      ros::Subscriber sub41; 
	  
	  
	  ros::Subscriber sub42;
      ros::Subscriber sub43;
      ros::Subscriber sub44;
      ros::Subscriber sub45;
	  
	  ros::Subscriber sub46;
      ros::Subscriber sub47;
      ros::Subscriber sub48;
      ros::Subscriber sub49;

      ros::Publisher pub_motorTorque;

      ros::NodeHandle node;

      float wheelAngularSpeed[notN][2][2];
      float motorSetPoint[notN][2][2];
      float motorTorque[notN][2][2];
      float prismaticSetPoint[notN-1];


      float axleHeading[notN][2];
      float contactNormal[notN][2][2];
      float wheelYawRate[notN][2][2];
      float Xt[notN][2][2];
      float wheelYaw[notN][2][2];
      float normalForce[notN][2][2];
      float wheelLatSpeed[notN][2][2];
      float wheelLonSpeed[notN][2][2];

      float connectorForce[notN]; // should be notN-1, but adding one element to prevent zero length when notN = 1

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      printf("Hello World!\n");



      //sub1 = node.subscribe("/wheelForcesX", 1, &ModelPush::wheelForcesX_callback,this);
      //sub2 = node.subscribe("/wheelForcesY", 1, &ModelPush::wheelForcesY_callback,this);
      //sub3 = node.subscribe("/wheelTorques", 1, &ModelPush::wheelTorques_callback,this,  ros::TransportHints().tcpNoDelay());
      sub3 = node.subscribe("/prismaticSetPoint", 5, &ModelPush::prismaticSetPoint_Callback, this,  ros::TransportHints().tcpNoDelay());
      sub4 = node.subscribe("/motorSetPoint", 5, &ModelPush::motorSetPoint_Callback, this,  ros::TransportHints().tcpNoDelay());
      sub5 = node.subscribe("/terrain_coefficient", 1, &ModelPush::terrain_coefficient_Callback, this);

      sub6 = node.subscribe("/sensor_prismatic_joint_1", 1, &ModelPush::sensor_1_1_Callback, this);
      sub7 = node.subscribe("/sensor_prismatic_joint_2", 1, &ModelPush::sensor_2_1_Callback, this);
      sub8 = node.subscribe("/sensor_prismatic_joint_3", 1, &ModelPush::sensor_3_1_Callback, this);
      sub9 = node.subscribe("/sensor_prismatic_joint_4", 1, &ModelPush::sensor_4_1_Callback, this);
	  sub9a = node.subscribe("/sensor_prismatic_joint_5", 1, &ModelPush::sensor_5_1_Callback, this);
	  sub9b = node.subscribe("/sensor_prismatic_joint_6", 1, &ModelPush::sensor_6_1_Callback, this);
	  sub9c = node.subscribe("/sensor_prismatic_joint_7", 1, &ModelPush::sensor_7_1_Callback, this);
	  sub9d = node.subscribe("/sensor_prismatic_joint_8", 1, &ModelPush::sensor_8_1_Callback, this);
	  sub9e = node.subscribe("/sensor_prismatic_joint_9", 1, &ModelPush::sensor_9_1_Callback, this);




      sub10 = node.subscribe("/bumper_wheel_1_1_1", 1, &ModelPush::bumper_wheel_1_1_1_Callback, this);
      sub11 = node.subscribe("/bumper_wheel_1_1_2", 1, &ModelPush::bumper_wheel_1_1_2_Callback, this);
      sub12 = node.subscribe("/bumper_wheel_1_2_1", 1, &ModelPush::bumper_wheel_1_2_1_Callback, this);
      sub13 = node.subscribe("/bumper_wheel_1_2_2", 1, &ModelPush::bumper_wheel_1_2_2_Callback, this);

      sub14 = node.subscribe("/bumper_wheel_2_1_1", 1, &ModelPush::bumper_wheel_2_1_1_Callback, this);
      sub15 = node.subscribe("/bumper_wheel_2_1_2", 1, &ModelPush::bumper_wheel_2_1_2_Callback, this);
      sub16 = node.subscribe("/bumper_wheel_2_2_1", 1, &ModelPush::bumper_wheel_2_2_1_Callback, this);
      sub17 = node.subscribe("/bumper_wheel_2_2_2", 1, &ModelPush::bumper_wheel_2_2_2_Callback, this);

      sub18 = node.subscribe("/bumper_wheel_3_1_1", 1, &ModelPush::bumper_wheel_3_1_1_Callback, this);
      sub19 = node.subscribe("/bumper_wheel_3_1_2", 1, &ModelPush::bumper_wheel_3_1_2_Callback, this);
      sub20 = node.subscribe("/bumper_wheel_3_2_1", 1, &ModelPush::bumper_wheel_3_2_1_Callback, this);
      sub21 = node.subscribe("/bumper_wheel_3_2_2", 1, &ModelPush::bumper_wheel_3_2_2_Callback, this);


      sub22 = node.subscribe("/bumper_wheel_4_1_1", 1, &ModelPush::bumper_wheel_4_1_1_Callback, this);
      sub23 = node.subscribe("/bumper_wheel_4_1_2", 1, &ModelPush::bumper_wheel_4_1_2_Callback, this);
      sub24 = node.subscribe("/bumper_wheel_4_2_1", 1, &ModelPush::bumper_wheel_4_2_1_Callback, this);
      sub25 = node.subscribe("/bumper_wheel_4_2_2", 1, &ModelPush::bumper_wheel_4_2_2_Callback, this);

      sub26 = node.subscribe("/bumper_wheel_5_1_1", 1, &ModelPush::bumper_wheel_5_1_1_Callback, this);
      sub27 = node.subscribe("/bumper_wheel_5_1_2", 1, &ModelPush::bumper_wheel_5_1_2_Callback, this);
      sub28 = node.subscribe("/bumper_wheel_5_2_1", 1, &ModelPush::bumper_wheel_5_2_1_Callback, this);
      sub29 = node.subscribe("/bumper_wheel_5_2_2", 1, &ModelPush::bumper_wheel_5_2_2_Callback, this);
	  
	  sub30 = node.subscribe("/bumper_wheel_6_1_1", 1, &ModelPush::bumper_wheel_6_1_1_Callback, this);
      sub31 = node.subscribe("/bumper_wheel_6_1_2", 1, &ModelPush::bumper_wheel_6_1_2_Callback, this);
      sub32 = node.subscribe("/bumper_wheel_6_2_1", 1, &ModelPush::bumper_wheel_6_2_1_Callback, this);
      sub33 = node.subscribe("/bumper_wheel_6_2_2", 1, &ModelPush::bumper_wheel_6_2_2_Callback, this);
	  
	  sub34 = node.subscribe("/bumper_wheel_7_1_1", 1, &ModelPush::bumper_wheel_7_1_1_Callback, this);
      sub35 = node.subscribe("/bumper_wheel_7_1_2", 1, &ModelPush::bumper_wheel_7_1_2_Callback, this);
      sub36 = node.subscribe("/bumper_wheel_7_2_1", 1, &ModelPush::bumper_wheel_7_2_1_Callback, this);
      sub37 = node.subscribe("/bumper_wheel_7_2_2", 1, &ModelPush::bumper_wheel_7_2_2_Callback, this);
	  
	  sub38 = node.subscribe("/bumper_wheel_8_1_1", 1, &ModelPush::bumper_wheel_8_1_1_Callback, this);
      sub39 = node.subscribe("/bumper_wheel_8_1_2", 1, &ModelPush::bumper_wheel_8_1_2_Callback, this);
      sub40 = node.subscribe("/bumper_wheel_8_2_1", 1, &ModelPush::bumper_wheel_8_2_1_Callback, this);
      sub41 = node.subscribe("/bumper_wheel_8_2_2", 1, &ModelPush::bumper_wheel_8_2_2_Callback, this);
	  
	  sub42 = node.subscribe("/bumper_wheel_9_1_1", 1, &ModelPush::bumper_wheel_9_1_1_Callback, this);
      sub43 = node.subscribe("/bumper_wheel_9_1_2", 1, &ModelPush::bumper_wheel_9_1_2_Callback, this);
      sub44 = node.subscribe("/bumper_wheel_9_2_1", 1, &ModelPush::bumper_wheel_9_2_1_Callback, this);
      sub45 = node.subscribe("/bumper_wheel_9_2_2", 1, &ModelPush::bumper_wheel_9_2_2_Callback, this);
	  
	  sub46 = node.subscribe("/bumper_wheel_10_1_1", 1, &ModelPush::bumper_wheel_10_1_1_Callback, this);
      sub47 = node.subscribe("/bumper_wheel_10_1_2", 1, &ModelPush::bumper_wheel_10_1_2_Callback, this);
      sub48 = node.subscribe("/bumper_wheel_10_2_1", 1, &ModelPush::bumper_wheel_10_2_1_Callback, this);
      sub49 = node.subscribe("/bumper_wheel_10_2_2", 1, &ModelPush::bumper_wheel_10_2_2_Callback, this);


      pub_motorTorque = node.advertise<std_msgs::Float32MultiArray>("/motorTorque", 5);

      //initialize Xt to non zero values
      for(int i=0; i<notN; ++i){
        for(int j=0; j<2; ++j){
          for(int k=0; k<2; ++k){
            Xt[i][j][k] = 1;
          }
        }
      }
    }



    // Called by the world update start event
    public: void OnUpdate()
    {
		static int count;
		static int count2;
		count++;
		count2++;
		static float output_torque[notN*2*2];
		static float output_speed[notN*2*2];
		static float output_wheelSpeed[notN*2*2];
		static float output_slip[notN*2*2];
		
		// This code adds a rearward force to the last segment in the vehicle
		//ROS_INFO_STREAM("######");
		//float rearAxleYaw = getWheelYaw(notN-1,1,0);
		//this->model->GetLink(axle_name[notN-1][1])->SetForce(ignition::math::Vector3d(-200*cos(rearAxleYaw), -200*sin(rearAxleYaw), 0.0));
	
		
		for(int i=0; i<notN; ++i){
			for(int j=0; j<2; ++j){
				//ROS_INFO_STREAM(normalForce[i][j][0] << ", " << normalForce[i][j][1]);
				for(int k=0; k<2; ++k){

					float COEFF;
					float wheelSlip;
					float frictionTorque;
					float terrainResistanceForce;
					float latTerrainResistanceForce;
					float otherResistanceTorque;
					float resistanceTorque;
					float yawResistanceTorque;
					float forceSlip;


					float wheelAngularSpeed_l = this->model->GetJoint(joint_name[i][j][k])->GetVelocity(0);
					float wheelLonSpeed = getWheelLonSpeed(i,j,k);

					//float contactNormal = v1.getContactNormal(i,j,k);

					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//calculate slip
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					const float deadBand = 0.025;
					//if speed and wheel speed very small
					if(fabs(wheelRadius*wheelAngularSpeed_l) < deadBand && fabs(wheelLonSpeed) < deadBand){
						wheelSlip = 0;
					}
					//if moving backwards slower than wheels
					else if(wheelLonSpeed < 0 && wheelRadius*wheelAngularSpeed_l < wheelLonSpeed){
						wheelSlip = -fabs(wheelLonSpeed - wheelRadius*wheelAngularSpeed_l) / fabs(wheelRadius*wheelAngularSpeed_l);
					}
					//if moving backwards faster than wheels
					else if(wheelLonSpeed < 0 && wheelRadius*wheelAngularSpeed_l > wheelLonSpeed){
						wheelSlip = fabs(wheelRadius*wheelAngularSpeed_l - wheelLonSpeed) / fabs(wheelLonSpeed);
					}
					//if moving forward faster than wheels
					else if(wheelLonSpeed > 0 && wheelRadius*wheelAngularSpeed_l < wheelLonSpeed){
						wheelSlip = -fabs(wheelLonSpeed - wheelRadius*wheelAngularSpeed_l) / fabs(wheelLonSpeed);
					}
					//if moving forward slower than wheels
					else if(wheelLonSpeed > 0 && wheelRadius*wheelAngularSpeed_l > wheelLonSpeed){
						wheelSlip = fabs(wheelRadius*wheelAngularSpeed_l - wheelLonSpeed) / fabs(wheelRadius*wheelAngularSpeed_l);
					}
					wheelSlip = constrain(wheelSlip,-1,1);

					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//calculate terrain resistance torque
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					otherResistanceTorque = -coeffOtherResistanceTorque*normalForce[i][j][k]*sign(wheelSlip)*(1 - Xt[i][j][k]);

					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//calculate terrain resistance forces
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//see equation 2.18 in thesis proposal
					//signSpeed prevents rapid switching between full forward and full backward values
					//terrainResistanceForce = -COEFF*signDoubleBand(wheelLonSpeed,0,0.0001)*normalForce;
					terrainResistanceForce = -coeffTerrainResistanceForce*sign(wheelLonSpeed)*normalForce[i][j][k] / pow(Xt[i][j][k],1) ;
					//ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce);

					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//frictionTorque
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//frictionTorque = -coeffFrictionTorque*wheelAngularSpeed_l;
					frictionTorque = 0;

					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//calculate lateralTerrainResistanceForce
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//latTerrainResistanceForce = -coeffLatTerrainResistanceForce*sign(v1.getWheelLatSpeed(i,j,k))*normalForce;
					latTerrainResistanceForce = 0;

					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					//calculate yaw resistance torque
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					yawResistanceTorque = -coeffYawResistanceTorque*sign(getWheelYawRate(i,j))*normalForce[i][j][k] / Xt[i][j][k];


					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					// calculate force slip
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					float B = 6*pow(Xt[i][j][k],1);
					float C = 2.8*Xt[i][j][k];
					float D = coeffForceSlip*pow(Xt[i][j][k],1)*normalForce[i][j][k];
					float E = 1;
					forceSlip = D*sin(C*atan(B*wheelSlip - E*(B*wheelSlip - atan(B*wheelSlip))));
					//ROS_INFO_STREAM(forceSlip);
					//double B = 2.3*Xt;
					//double C = 2.6/pow(Xt,0.5);
					//Equation from https://en.wikipedia.org/wiki/Hans_B._Pacejka
					//Pacejka, H. B. (2012). Tire and vehicle dynamics. Besselink, Igo (3rd ed.). Oxford: Butterworth-Heinemann. p. 165. ISBN 978-0-08-097016-5. OCLC 785829133
					//float D = coeffForceSlip*Xt*normalForce;
					//const double E = 1.0;
					//forceSlip[i][j][k] = D*sin(C*atan(B*wheelSlip - E*(B*wheelSlip - atan(B*wheelSlip))));



					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					// calculate torque from force slip
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					resistanceTorque = (-forceSlip + terrainResistanceForce)*wheelRadius;


					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					// Apply forces and torques to wheel and joint
					//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					float wheelYaw = getWheelYaw(i,j,k);


					forcesX[i][j][k] = constrain((forceSlip + terrainResistanceForce)*cos(wheelYaw) + -latTerrainResistanceForce*sin(wheelYaw),-1000,1000);
					forcesY[i][j][k] = constrain((forceSlip + terrainResistanceForce)*sin(wheelYaw) + latTerrainResistanceForce*cos(wheelYaw),-1000,1000);
					torques[i][j][k] = constrain(motorTorque[i][j][k] + resistanceTorque + otherResistanceTorque,-200,200);

					this->model->GetLink(model_name[i][j][k])->SetForce(ignition::math::Vector3d(forcesX[i][j][k], forcesY[i][j][k], 0.0));
					this->model->GetLink(model_name[i][j][k])->SetTorque(ignition::math::Vector3d(0.0, 0.0, yawResistanceTorque));
					this->model->GetJoint(joint_name[i][j][k])->SetForce(0, torques[i][j][k]);

					//ROS_INFO_STREAM(Xt[i][j][k] << ", " << wheelLonSpeed << ", " << wheelAngularSpeed_l*wheelRadius << ", " << motorTorque[i][j][k] << ", " << wheelSlip);
					if( i == 0 && j ==0 && k == 0 && count > 50){		
						count = 0;
						//ROS_INFO_STREAM(Xt[i][j][k]);
						//ROS_INFO_STREAM(roll << ", " << pitch << ", " << yaw);
						//ROS_INFO_STREAM(Xt[i][j][k] << ", " << wheelLonSpeed << ", " << wheelAngularSpeed_l*wheelRadius << ", " << motorTorque[i][j][k] << ", " << wheelSlip  << ", " << connectorForce[0] << ", " << connectorForce[1] << ", " << connectorForce[2] << ", " << connectorForce[3]);
						//ROS_INFO_STREAM(forceSlip << ", " << terrainResistanceForce);
						//ROS_INFO_STREAM(Xt[i][j][k] << ", " << wheelLonSpeed << ", " << wheelAngularSpeed_l*wheelRadius << ", " << motorTorque[i][j][k] << ", " << 100*wheelSlip);
						//ROS_INFO_STREAM(motorTorque[i][j][k] << ", " << compactionResistanceTorque << ", " << resistanceTorque << ", " << frictionTorque);
					}
					output_speed[i*4 + j*2 + k] = wheelLonSpeed;
					output_wheelSpeed[i*4 + j*2 + k] = wheelAngularSpeed_l*wheelRadius;
					output_slip[i*4 + j*2 + k] = wheelSlip;


				}
			}
		}
		//how forces are applied to the prismatic joints
		for(int i=0; i<notN-1; ++i){
			this->model->GetJoint(prismatic_name[i])->SetForce(0, prismaticSetPoint[i]);
			if(prismaticSetPoint[i] == 0){
				this->model->GetJoint(prismatic_name[i])->SetParam("friction",0,5000.0);
			} else{
				this->model->GetJoint(prismatic_name[i])->SetParam("friction",0,10.0);
			}
		}

		if(count2 == 50){
			count2 = 0;
			
			//xt, speed, wheelSpeed, torque, slip for four wheels on single segment
			//ROS_INFO_STREAM(Xt[0][0][0] << ", " << output_speed[0] << ", " << output_wheelSpeed[0] << ", " << motorTorque[0][0][0] << ", " << output_slip[0] << ", " << Xt[0][0][1] << ", " << output_speed[1] << ", " << output_wheelSpeed[1] << ", " << motorTorque[0][0][1] << ", " << output_slip[1] << ", " << Xt[0][1][0] << ", " << output_speed[2] << ", " << output_wheelSpeed[2] << ", " << motorTorque[0][1][0] << ", " << output_slip[2] << ", " << Xt[0][1][1] << ", " << output_speed[3] << ", " << output_wheelSpeed[3] << ", " << motorTorque[0][1][1] << ", " << output_slip[3]);
			//
			//ROS_INFO_STREAM(output_speed[0] << ", " << output_wheelSpeed[0] << ", " << motorTorque[0][0][0] << ", " << connectorForce[0] << ", " << output_speed[4] << ", " << output_wheelSpeed[4] << ", " << motorTorque[1][0][0] << ", " << connectorForce[1] << ", " << output_speed[8] << ", " << output_wheelSpeed[8] << ", " << motorTorque[2][0][0] << ", " << connectorForce[2] << ", " << output_speed[12] << ", " << output_wheelSpeed[12] << ", " << motorTorque[3][0][0]);
			//ROS_INFO_STREAM(output_slip[0] << ", " << output_slip[4] << ", " << output_slip[8] << ", " << output_slip[12]);
		}

    } //end onupdate

    public:

      float getWheelLonSpeed(int i, int j, int k){
        gazebo::math::Vector3 speeds;
        speeds = this->model->GetLink(model_name[i][j][k])->GetWorldLinearVel();
        float x = speeds.x;
        float y = speeds.y;
        float z = speeds.z;

        float yaw = getWheelYaw(i,j,k);

        if(i == 0 && j == 0 && k ==0){
          //ROS_INFO_STREAM(x << ", " << y << ", " << z << ", " << yaw);
        }



        float lonVel = x*cos(yaw) + y*sin(yaw);
        return lonVel;

      }

      float getWheelYawRate(int i, int j){
        gazebo::math::Vector3 angularVel;
        angularVel = this->model->GetLink(axle_name[i][j])->GetWorldAngularVel();
        return angularVel.z;


      }

      float getWheelYaw(int i, int j, int k){
        gazebo::math::Pose pose;
        pose = this->model->GetLink(axle_name[i][j])->GetWorldPose();
        float x = pose.rot.x;
        float y = pose.rot.y;
        float z = pose.rot.z;
        float w = pose.rot.w;

        tf::Quaternion q(x,y,z,w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);


        /*
        if(i == 0 && j == 0 && k ==0){
          ROS_INFO_STREAM(roll << ", " << pitch << ", " << yaw);
        }
        */

        return yaw;

      }

      void prismaticSetPoint_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        for(int i=0; i<notN-1; i++){
          prismaticSetPoint[i] = msg->data[i];
        }
      }

      void terrain_coefficient_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        for(int i=0; i<notN; i++){
          for(int j=0; j<2; j++){
            for(int k=0; k<2; k++){
              Xt[i][j][k] = msg->data[(4*i)+(2*j)+k];
            }
          }
        }
      }

      void motorSetPoint_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
      {
        //publishes the calculated motor torque
        std_msgs::Float32MultiArray outgoing_msg;
        outgoing_msg.data.resize(notN*2*2);

        for(size_t i=0; i<notN; i++){
          for(size_t j=0; j<2; j++){
            for(size_t k=0; k<2; ++k){

              // get the new setpoint value
              float data = msg->data[i*4 + j*2 + k];
              if (std::isnan(data) || std::isinf(data)) {
                motorSetPoint[i][j][k] = 0;
              } else {
                motorSetPoint[i][j][k] = data;
              }

              //calculate the new motor torque
              float speed = this->model->GetJoint(joint_name[i][j][k])->GetVelocity(0);
              float motorTorque1 = constrain(0.8*motorTorque[i][j][k] + 0.2*slope*(speed - motorSetPoint[i][j][k]*noLoadSpeed),-88,88);
              if (std::isnan(motorTorque1)) {motorTorque1 = 0;}
			  //motorTorque1 = round(motorTorque1); rounding seems bad when this can be very small
              motorTorque[i][j][k] = motorTorque1;
              outgoing_msg.data[i*4 + j*2 + k] = motorTorque1;


            }
          }
        }
        pub_motorTorque.publish(outgoing_msg);
        //ROS_INFO_STREAM("wheelForcesX Callback " << forcesX[0][0][0]);
      }


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

      void sensor_1_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        //float z = msg->wrench.force.z;
        //float y = msg->wrench.force.y;
        float x = msg->wrench.force.x;
        connectorForce[0] = x;
        //ROS_INFO_STREAM(connectorForce[0] << ", " << connectorForce[1] << ", " << connectorForce[2] << ", " << connectorForce[3] << ", " << connectorForce[4]);
      }

      void sensor_2_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[1] = x;
      }

      void sensor_3_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[2] = x;
      }
      void sensor_4_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[3] = x;
      }
	  
	  void sensor_5_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[4] = x;
      }
	  
	  void sensor_6_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[5] = x;
      }
	  
	 void sensor_7_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[6] = x;
      }
	  
	 void sensor_8_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[7] = x;
      }	  
	  
	 void sensor_9_1_Callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
        float x = msg->wrench.force.x;
        connectorForce[8] = x;
      }	  
	  

      void bumper_wheel_1_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[0][0][0] = force_from_wheel(msg);

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
	  
	  void bumper_wheel_6_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[5][0][0] = force_from_wheel(msg);
      }
      void bumper_wheel_6_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[5][0][1] = force_from_wheel(msg);
      }
      void bumper_wheel_6_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[5][1][0] = force_from_wheel(msg);
      }
      void bumper_wheel_6_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[5][1][1] = force_from_wheel(msg);
      }
	  
	  void bumper_wheel_7_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[6][0][0] = force_from_wheel(msg);
      }
      void bumper_wheel_7_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[6][0][1] = force_from_wheel(msg);
      }
      void bumper_wheel_7_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[6][1][0] = force_from_wheel(msg);
      }
      void bumper_wheel_7_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[6][1][1] = force_from_wheel(msg);
      }	  
	  
	  void bumper_wheel_8_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[7][0][0] = force_from_wheel(msg);
      }
      void bumper_wheel_8_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[7][0][1] = force_from_wheel(msg);
      }
      void bumper_wheel_8_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[7][1][0] = force_from_wheel(msg);
      }
      void bumper_wheel_8_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[7][1][1] = force_from_wheel(msg);
      }
	  
	  void bumper_wheel_9_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[8][0][0] = force_from_wheel(msg);
      }
      void bumper_wheel_9_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[8][0][1] = force_from_wheel(msg);
      }
      void bumper_wheel_9_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[8][1][0] = force_from_wheel(msg);
      }
      void bumper_wheel_9_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[8][1][1] = force_from_wheel(msg);
      }	  

	  void bumper_wheel_10_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[9][0][0] = force_from_wheel(msg);
      }
      void bumper_wheel_10_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[9][0][1] = force_from_wheel(msg);
      }
      void bumper_wheel_10_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[9][1][0] = force_from_wheel(msg);
      }
      void bumper_wheel_10_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg){
        normalForce[9][1][1] = force_from_wheel(msg);
      }	  
	  

      double constrain(double number, double lowerLimit, double upperLimit){
        if(number > upperLimit){
          return upperLimit;
        }
        else if(number < lowerLimit){
          return lowerLimit;
        }
        return number;
      }

      float sign(float number){
        if(number < 0){
          return -1;
        }
        else if (number > 0){
          return 1;
        }
        return 0;

      }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
