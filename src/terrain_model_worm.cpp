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




//11 is front left, 12 front right, 21 back left...
double normalForce[2][2]= {{0,0},{0,0}};
double wheelAngularSpeed[2][2]= {{0,0},{0,0}};
double wheelLonSpeed[2][2]= {{0,0},{0,0}};
double wheelLatSpeed[2][2]= {{0,0},{0,0}};
double wheelYaw[2][2]= {{0,0},{0,0}};
double wheelYawRate[2][2]= {{0,0},{0,0}};
double motorTorque[2][2] = {{0,0},{0,0}};
double Xt[2][2]= {{1.5,1.5},{1.5,1.5}};
const double pi = 3.141592;
const double pi2 = 1.570796;




const double loopRate = 2000;




double force_from_wheel(const gazebo_msgs::ContactsState::ConstPtr& msg){
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

void bumper_wheel_1_1_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  normalForce[0][1] = force_from_wheel(msg);
}

void bumper_wheel_1_1_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  normalForce[0][0] = force_from_wheel(msg);
}

void bumper_wheel_1_2_1_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  normalForce[1][0] = force_from_wheel(msg);
}

void bumper_wheel_1_2_2_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  normalForce[1][1] = force_from_wheel(msg);
}

void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  static double wheelPosition[2][2];
  static double yamlRate = 1000; // must be same as in YAML


  for(size_t i=0; i<names.size(); ++i){
    if(names[i].compare("joint_wheel_1_2_1") == 0){
      wheelAngularSpeed[1][0] = (msg->position[i] - wheelPosition[1][0]) * yamlRate; //calculate
      wheelPosition[1][0] = msg->position[i]; //update position with current

      //ROS_INFO_STREAM("joint_back_left position " << wheelPosition[1][0]);
    }
    else if (names[i].compare("joint_wheel_1_2_2") == 0){
      wheelAngularSpeed[1][1] = (msg->position[i] - wheelPosition[1][1]) * yamlRate; //calculate
      wheelPosition[1][1] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_back_right " << wheelAngularSpeed[1][1]);
    }
    else if (names[i].compare("joint_wheel_1_1_1") == 0){
      wheelAngularSpeed[0][0] = (msg->position[i] - wheelPosition[0][0]) * yamlRate; //calculate
      wheelPosition[0][0] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_front_left " << wheelAngularSpeed[0][0]);
    }
    else if (names[i].compare("joint_wheel_1_1_2") == 0){
      wheelAngularSpeed[0][1] = (msg->position[i] - wheelPosition[0][1]) * yamlRate; //calculate
      wheelPosition[0][1] = msg->position[i]; //update position with current
      //ROS_INFO_STREAM("joint_front_right " << wheelAngularSpeed[0][1]);
    }

  }

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
  yaw = yaw - pi2;

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
  yaw = yaw - pi2;

  double xVel = msg->twist[i].linear.x;
  double yVel = msg->twist[i].linear.y;

  double latVel = -xVel*sin(yaw) + yVel*cos(yaw);
  return latVel;
}

double get_wheelYaw_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  double x = msg->pose[i].orientation.x;
  double y = msg->pose[i].orientation.y;
  double z = msg->pose[i].orientation.z;
  double w = msg->pose[i].orientation.w;

  tf::Quaternion q(x,y,z,w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw = yaw - pi2;

  return yaw; //due to model orientation
}

double get_wheelYawRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
return msg->twist[i].angular.z;
}

void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    const std::vector<std::string> &names = msg->name;
    for(size_t i=0; i<names.size(); ++i){
      if(names[i].compare("worm::wheel_1_2_1") == 0){
        wheelLonSpeed[1][0] = get_lonSpeed_from_link_states(msg,i);
        wheelLatSpeed[1][0] = get_latSpeed_from_link_states(msg,i);
        wheelYaw[1][0] = get_wheelYaw_from_link_states(msg,i);
        wheelYawRate[1][0] = get_wheelYawRate_from_link_states(msg,i);
        //ROS_INFO_STREAM("back_left wheelLonSpeed is " << wheelLonSpeed[1][0]);
      }
      else if (names[i].compare("worm::wheel_1_2_2") == 0){
        wheelLonSpeed[1][1] = get_lonSpeed_from_link_states(msg,i);
        wheelLatSpeed[1][1] = get_latSpeed_from_link_states(msg,i);
        wheelYaw[1][1] = get_wheelYaw_from_link_states(msg,i);
        wheelYawRate[1][1] = get_wheelYawRate_from_link_states(msg,i);
        //ROS_INFO_STREAM("back_right wheelLonSpeed is " << wheelLonSpeed[1][1]);
      }
      else if (names[i].compare("worm::wheel_1_1_1") == 0){
        wheelLonSpeed[0][0] = get_lonSpeed_from_link_states(msg,i);
        wheelLatSpeed[0][0] = get_latSpeed_from_link_states(msg,i);
        wheelYaw[0][0] = get_wheelYaw_from_link_states(msg,i);
        wheelYawRate[0][0] = get_wheelYawRate_from_link_states(msg,i);
        //ROS_INFO_STREAM("front_left wheelLonSpeed is " << wheelLonSpeed[0][0]);
      }
      else if (names[i].compare("worm::wheel_1_1_2") == 0){
        wheelLonSpeed[0][1] = get_lonSpeed_from_link_states(msg,i);
        wheelLatSpeed[0][1] = get_latSpeed_from_link_states(msg,i);
        wheelYaw[0][1] = get_wheelYaw_from_link_states(msg,i);
        wheelYawRate[0][1] = get_wheelYawRate_from_link_states(msg,i);
        //ROS_INFO_STREAM("front_right wheelLonSpeed is " << wheelLonSpeed[0][1]);
      }
    }

}

void motorTorque_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  //callback for the motorTorque topic from the control node
  for(int i=0; i<2; i++){
    for(int j=0; j<2; j++){
      //ROS_INFO_STREAM(msg->data[(2*i)+j]);
      motorTorque[i][j] = msg->data[(2*i)+j];
      //ROS_INFO_STREAM(" motorTorque " << motorTorque[i][j]);
    }
  }
}

void terrain_coefficient_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  for(int i=0; i<2; i++){
    for(int j=0; j<2; j++){
      Xt[i][j] = msg->data[2*i+j];
    }
  }
  //ROS_INFO_STREAM("Xt " << Xt);
}



int main(int argc, char **argv){
	//ROS_INFO_STREAM("test");



	std::string model_name[2][2] = {{"worm::wheel_1_1_1","worm::wheel_1_1_2"},{"worm::wheel_1_2_1","worm::wheel_1_2_2"}};
  std::string joint_name[2][2] = {{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}};


	ros::init(argc, argv, "terrain_model");
	ros::NodeHandle terrain_model_node;

  //services called
  ros::ServiceClient wrenchClient = terrain_model_node.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
	gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
	gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

  ros::ServiceClient jointEffortClient = terrain_model_node.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
	gazebo_msgs::ApplyJointEffort::Request apply_joint_effort_req;
	gazebo_msgs::ApplyJointEffort::Response apply_joint_effort_resp;

  //topics subscribed to
  ros::Subscriber sub1 = terrain_model_node.subscribe("/bumper_wheel_1_1_1", 10, bumper_wheel_1_1_1_Callback);
  ros::Subscriber sub2 = terrain_model_node.subscribe("/bumper_wheel_1_1_2", 10, bumper_wheel_1_1_2_Callback);
  ros::Subscriber sub3 = terrain_model_node.subscribe("/bumper_wheel_1_2_1", 10, bumper_wheel_1_2_1_Callback);
  ros::Subscriber sub4 = terrain_model_node.subscribe("/bumper_wheel_1_2_2", 10, bumper_wheel_1_2_2_Callback);

  ros::Subscriber sub5 = terrain_model_node.subscribe("/worm/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub6 = terrain_model_node.subscribe("/gazebo/link_states", 10, link_states_Callback);


  ros::Subscriber sub7 = terrain_model_node.subscribe("/motorTorque", 10, motorTorque_Callback);

  ros::Subscriber sub8 = terrain_model_node.subscribe("/terrain_coefficient", 10, terrain_coefficient_Callback);


	bool service_ready = false;
	while (!service_ready) {
		service_ready = ros::service::exists("/gazebo/apply_body_wrench", true);
		//ROS_INFO("waiting for apply_body_wrench service");
		ros::Duration(0.5).sleep();
	}

  service_ready = false;
  while (!service_ready) {
    service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
    ros::Duration(0.5).sleep();
  }



	ros::Rate loop_rate(loopRate);


  const double wheelRadius = 0.28;
  const double wheelWidth = 0.45;



  //calcuate slip

  double wheelSlip[2][2]; //left right then front to back
  double frictionTorque[2][2];
  double terrainResistanceForce[2][2];
  double latTerrainResistanceForce[2][2];
  double terrainResistanceTorque[2][2];
  double yawResistanceTorque[2][2];
  double forceSlip[2][2];


  ros::spinOnce();
  ros::Duration(1).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  ROS_INFO_STREAM("#########################################################");


	while(ros::ok()){
    //
    //calculate relevant vehicle variables
    //


    //calculate slip
    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){

        const double deadBand = 0.025;
        //if speed and wheel speed very small
        if(fabs(wheelRadius*wheelAngularSpeed[i][j]) < deadBand && fabs(wheelLonSpeed[i][j]) < deadBand){
          wheelSlip[i][j] = 0;
        }
        //if moving backwards slower than wheels
        else if(wheelLonSpeed[i][j] < 0 && wheelRadius*wheelAngularSpeed[i][j] < wheelLonSpeed[i][j]){
          wheelSlip[i][j] = -fabs(wheelLonSpeed[i][j] - wheelRadius*wheelAngularSpeed[i][j]) / fabs(wheelRadius*wheelAngularSpeed[i][j]);
        }
        //if moving backwards faster than wheels
        else if(wheelLonSpeed[i][j] < 0 && wheelRadius*wheelAngularSpeed[i][j] > wheelLonSpeed[i][j]){
          wheelSlip[i][j] = fabs(wheelRadius*wheelAngularSpeed[i][j] - wheelLonSpeed[i][j]) / fabs(wheelLonSpeed[i][j]);
        }
        //if moving forward faster than wheels
        else if(wheelLonSpeed[i][j] > 0 && wheelRadius*wheelAngularSpeed[i][j] < wheelLonSpeed[i][j]){
          wheelSlip[i][j] = -fabs(wheelLonSpeed[i][j] - wheelRadius*wheelAngularSpeed[i][j]) / fabs(wheelLonSpeed[i][j]);
        }
        //if moving forward slower than wheels
        else if(wheelLonSpeed[i][j] > 0 && wheelRadius*wheelAngularSpeed[i][j] > wheelLonSpeed[i][j]){
          wheelSlip[i][j] = fabs(wheelRadius*wheelAngularSpeed[i][j] - wheelLonSpeed[i][j]) / fabs(wheelRadius*wheelAngularSpeed[i][j]);
        }
        wheelSlip[i][j] = constrain(wheelSlip[i][j],-1,1);

      }
    }


    //calculate frictionTorque
    //double coeffFrictionTorque = 2.4;//0.1;
    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){
        //sign speed is used to add a non linear drop off when close to zero
        frictionTorque[i][j] = -(coeffFrictionTorque*fabs(wheelAngularSpeed[i][j])*signSpeed(wheelAngularSpeed[i][j]) + 0.1*signSpeed(wheelAngularSpeed[i][j],0.001));
        //ROS_INFO_STREAM("frictionTorque is " << frictionTorque[i][j]);
      }
    }



    //calculate terrainResistanceTorque

    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){
        double COEFF = coeffCompactionResistanceTorque*Xt[i][j];
        if(wheelLonSpeed[i][j] > 0){
          if(wheelRadius*wheelAngularSpeed[i][j] > wheelLonSpeed[i][j]){ //spinning faster than moving
            terrainResistanceTorque[i][j] = -pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j] - wheelRadius*wheelAngularSpeed[i][j])*COEFF;
            //terrainResistanceTorque[i][j] = -pow(wheelRadius,2)*wheelWidth*fabs(wheelSlip[i][j])*coeffTerrainResistanceTorque;
          }
          else if(wheelRadius*wheelAngularSpeed[i][j] < wheelLonSpeed[i][j]){  //spinning slower than moving
            terrainResistanceTorque[i][j] = pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j] - wheelRadius*wheelAngularSpeed[i][j])*COEFF;
            //terrainResistanceTorque[i][j] = pow(wheelRadius,2)*wheelWidth*fabs(wheelSlip[i][j])*coeffTerrainResistanceTorque;
          }
        }
        else if(wheelLonSpeed[i][j] < 0) { //wheel going backwards
          if(wheelRadius*wheelAngularSpeed[i][j] > wheelLonSpeed[i][j]){ //spinning slower than moving
            terrainResistanceTorque[i][j] = -pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j] - wheelRadius*wheelAngularSpeed[i][j])*COEFF;
            //terrainResistanceTorque[i][j] = -pow(wheelRadius,2)*wheelWidth*fabs(wheelSlip[i][j])*coeffTerrainResistanceTorque;
          }
          else if(wheelRadius*wheelAngularSpeed[i][j] < wheelLonSpeed[i][j]){  //spinning faster than moving
            terrainResistanceTorque[i][j] = pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j] - wheelRadius*wheelAngularSpeed[i][j])*COEFF;
            //terrainResistanceTorque[i][j] = pow(wheelRadius,2)*wheelWidth*fabs(wheelSlip[i][j])*coeffTerrainResistanceTorque;
          }

        }
        //constrain, add Xt, and make sure wheel is on ground
        terrainResistanceTorque[i][j] = constrain(terrainResistanceTorque[i][j],-100,100)*isNotZero(normalForce[i][j]);
      }
    }

    //calculate terrain resistance forces

    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){
        double COEFF = coeffTerrainResistanceForce/Xt[i][j];//0.2;
        //double coeffTerrainResistanceForce = 0.3/Xt[i][j];//0.2;
        //see equation 2.18 in thesis proposal
        //signSpeed prevents rapid switching between full forward and full backward values
        terrainResistanceForce[i][j] = -COEFF*signDoubleBand(wheelLonSpeed[i][j],0,0.0001)*normalForce[i][j];
        //ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce[i][j]);
      }
    }

    //calculate lateralTerrainResistanceForce

    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){
        //double coeffLatTerrainResistanceForce = 1.5/Xt[i][j];
        double COEFF = coeffLatTerrainResistanceForce/Xt[i][j];
        //terrainResistanceForce[i][j] = coeffTerrainResistanceForce*signSpeed(wheelLatSpeed[i][j])*isNotZero(normalForce[i][j]);
        //ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce[i][j]);
        latTerrainResistanceForce[i][j] = COEFF*signDoubleBand(wheelLatSpeed[i][j],0.0001,0.05)*normalForce[i][j];
      }
    }



    // calculate force slip


    double D;
    double E = 1.0;
    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){
        double B = 2.3*Xt[i][j];
        double C = 2.6/pow(Xt[j][j],0.5);
        //Equation from https://en.wikipedia.org/wiki/Hans_B._Pacejka
        //Pacejka, H. B. (2012). Tire and vehicle dynamics. Besselink, Igo (3rd ed.). Oxford: Butterworth-Heinemann. p. 165. ISBN 978-0-08-097016-5. OCLC 785829133
        D = pow(Xt[i][j],0.5)*normalForce[i][j];
        forceSlip[i][j] = D*sin(C*atan(B*wheelSlip[i][j] - E*(B*wheelSlip[i][j] - atan(B*wheelSlip[i][j]))))*isNotZero(normalForce[i][j]);

        /*
        if(wheelSlip[i][j] > 0 && forceSlip[i][j] < 0){
          forceSlip[i][j] = 0;
        }
        else if(wheelSlip[i][j] < 0 && forceSlip[i][j] > 0){
          forceSlip[i][j] = 0;
        }
        */


      }
    }


    //calculate yaw resistance torque
    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){
        //double coeffYawResistanceTorque = 5/Xt[i][j];
        double COEFF = coeffYawResistanceTorque*Xt[i][j];
        yawResistanceTorque[i][j] = -COEFF*signSpeed(wheelYawRate[i][j])*isNotZero(normalForce[i][j]);
      }
    }



    //ROS_INFO_STREAM("##################################");
    /*
    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){

        //int i = 1;
        //int j = 0;
        ROS_INFO_STREAM("Xt is " << Xt[i][j]);
        ROS_INFO_STREAM("wheelYaw is " << wheelYaw[i][j]);
        ROS_INFO_STREAM("normalForce is " << normalForce[i][j]);
        ROS_INFO_STREAM("wheelAngularSpeed*R is " << wheelAngularSpeed[i][j]*wheelRadius);
        ROS_INFO_STREAM("frictionTorque is " << frictionTorque[i][j]);


        ROS_INFO_STREAM("wheelLonSpeed is " << wheelLonSpeed[i][j]);
        ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce[i][j]);
        ROS_INFO_STREAM("terrainResistanceTorque is " << terrainResistanceTorque[i][j]);


        ROS_INFO_STREAM("wheelLatSpeed is " << wheelLatSpeed[i][j]);
        ROS_INFO_STREAM("latTerrainResistanceForce is " << latTerrainResistanceForce[i][j]);


        ROS_INFO_STREAM("wheelYawRate is " << wheelYawRate[i][j]);
        ROS_INFO_STREAM("yawResistanceTorque is " << yawResistanceTorque[i][j]);

        ROS_INFO_STREAM("wheelSlip is " << wheelSlip[i][j]);
        ROS_INFO_STREAM("forceSlip is " << forceSlip[i][j]);

        ROS_INFO_STREAM("motorTorque is " << motorTorque[i][j]);



        ROS_INFO_STREAM(" ");

      }
    }
    */





    //ROS_INFO_STREAM("wheelYaw is " << 180*wheelYaw[1][0]/3.14);



    /*
    ROS_INFO_STREAM("wheelLonSpeed is " << wheelLatSpeed[0][0]);
    ROS_INFO_STREAM("wheelLonSpeed is " << wheelLatSpeed[0][1]);
    ROS_INFO_STREAM("wheelLonSpeed is " << wheelLatSpeed[1][0]);
    ROS_INFO_STREAM("wheelLonSpeed is " << wheelLatSpeed[1][1]);
    */

    /*
    ROS_INFO_STREAM("terrainResistanceTorque is " << terrainResistanceTorque[0][0]);
    ROS_INFO_STREAM("terrainResistanceTorque is " << terrainResistanceTorque[0][1]);
    ROS_INFO_STREAM("terrainResistanceTorque is " << terrainResistanceTorque[1][0]);
    ROS_INFO_STREAM("terrainResistanceTorque is " << terrainResistanceTorque[1][1]);
    */

    //assign motor torque quick and dirty
    /*
    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){
        motorTorque[i][j] = (1.0)*min(max(5 - fabs(wheelAngularSpeed[i][j]),0),5);
      }
    }
    */



		ros::Duration duration_temp(1.0/loopRate);
    apply_wrench_req.duration = duration_temp;
    apply_joint_effort_req.duration = duration_temp;
    for(int i=0; i<2; ++i){
      for(int j=0; j<2; ++j){



        //global reference frame
    		apply_wrench_req.wrench.force.x = (forceSlip[i][j] + terrainResistanceForce[i][j])*cos(wheelYaw[i][j]) + latTerrainResistanceForce[i][j]*sin(wheelYaw[i][j]); //this is good;
    		apply_wrench_req.wrench.force.y = (forceSlip[i][j] + terrainResistanceForce[i][j])*sin(wheelYaw[i][j]) + -latTerrainResistanceForce[i][j]*cos(wheelYaw[i][j]);//this is good
        apply_wrench_req.wrench.force.z = 0.0;

    		apply_wrench_req.wrench.torque.x = 0.0;
        apply_wrench_req.wrench.torque.y = 0.0;
        apply_wrench_req.wrench.torque.z = yawResistanceTorque[i][j];
        apply_wrench_req.body_name = model_name[i][j];

    		// call apply body wrench service
    		wrenchClient.call(apply_wrench_req, apply_wrench_resp);





        apply_joint_effort_req.effort = constrain(motorTorque[i][j] + terrainResistanceTorque[i][j] + frictionTorque[i][j],-1000,1000);
        //apply_joint_effort_req.effort = constrain(1 + terrainResistanceTorque[i][j] + frictionTorque[i][j],-1000,1000);
    		apply_joint_effort_req.joint_name = joint_name[i][j];

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
