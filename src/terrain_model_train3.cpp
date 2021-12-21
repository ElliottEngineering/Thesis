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


//11 is front left, 12 front right, 21 back left...
double normalForce[N][2][2];
double wheelAngularSpeed[N][2][2];
double wheelLonSpeed[N][2][2];
double wheelLatSpeed[N][2][2];
double wheelYaw[N][2][2];
double wheelYawRate[N][2][2];

double axleYaw[N][2];

double Xt[N][2][2] = {{{1,1},{1,1}},
                      {{1,1},{1,1}},
                      {{1,1},{1,1}}};
const double pi = 3.141592;
const double pi2 = 1.570796;

std::string model_name[N][2][2] = {{{"train3::wheel_1_1_1","train3::wheel_1_1_2"},{"train3::wheel_1_2_1","train3::wheel_1_2_2"}},
                                   {{"train3::wheel_2_1_1","train3::wheel_2_1_2"},{"train3::wheel_2_2_1","train3::wheel_2_2_2"}},
                                   {{"train3::wheel_3_1_1","train3::wheel_3_1_2"},{"train3::wheel_3_2_1","train3::wheel_3_2_2"}}};

//std::string joint_name[N][2][2] = {{{"train3::joint_wheel_1_1_1","train3::joint_wheel_1_1_2"},{"train3::joint_wheel_1_2_1","train3::joint_wheel_1_2_2"}},
//                                   {{"train3::joint_wheel_2_1_1","train3::joint_wheel_2_1_2"},{"train3::joint_wheel_2_2_1","train3::joint_wheel_2_2_2"}},
//                                   {{"train3::joint_wheel_3_1_1","train3::joint_wheel_3_1_2"},{"train3::joint_wheel_3_2_1","train3::joint_wheel_3_2_2"}}};

std::string joint_name[N][2][2] = {{{"joint_wheel_1_1_1","joint_wheel_1_1_2"},{"joint_wheel_1_2_1","joint_wheel_1_2_2"}},
                                  {{"joint_wheel_2_1_1","joint_wheel_2_1_2"},{"joint_wheel_2_2_1","joint_wheel_2_2_2"}},
                                  {{"joint_wheel_3_1_1","joint_wheel_3_1_2"},{"joint_wheel_3_2_1","joint_wheel_3_2_2"}}};

std::string axle_name[N][2] = {{"train3::axle_1_1","train3::axle_1_2"},{"train3::axle_2_1","train3::axle_2_2"},{"train3::axle_3_1","train3::axle_3_2"}};


const double loopRate = 100;






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





//all of the wheel bumper callbacks
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



void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  const std::vector<std::string> &names = msg->name;
  static double wheelPosition[N][2][2];
  static double yamlRate = 1000; // must be same as in YAML
  for(size_t h=0; h<names.size(); ++h){
    for(size_t i=0; i<N; ++i){
      for(size_t j=0; j<2; ++j){
        for(size_t k=0; k<2; ++k){
          if(names[h].compare(joint_name[i][j][k]) == 0){
            //ROS_INFO_STREAM(names[h] << " should equal " << model_name[i][j][k]);
            wheelAngularSpeed[i][j][k] = (msg->position[h] - wheelPosition[i][j][k]) * yamlRate; //calculate
            wheelPosition[i][j][k] = msg->position[h]; //update position with current
            //ROS_INFO_STREAM("update");
            goto found; //go to next point in message
          }
        }
      }
    }
    found:;
  }
}



double get_lonSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i,double yaw){
  double xVel = msg->twist[i].linear.x;
  double yVel = msg->twist[i].linear.y;

  double lonVel = xVel*cos(yaw) + yVel*sin(yaw);
  return lonVel;
}

double get_latSpeed_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i,double yaw){
  double xVel = msg->twist[i].linear.x;
  double yVel = msg->twist[i].linear.y;

  double latVel = -xVel*sin(yaw) + yVel*cos(yaw);
  return latVel;
}



double get_axleYaw_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
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


double get_wheelYawRate_from_link_states(const gazebo_msgs::LinkStates::ConstPtr& msg, int i){
  return msg->twist[i].angular.z;
}

void link_states_Callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    const std::vector<std::string> &names = msg->name;
    //first get axle yaws
    for(size_t h=0; h<names.size(); ++h){
      for(size_t i=0; i<N; ++i){
        for(size_t j=0; j<2; ++j){
          if(names[h].compare(axle_name[i][j]) == 0){
            axleYaw[i][j] = get_axleYaw_from_link_states(msg,h);
            goto found1; //go to next point in message
          }
        }
      }
      found1:;
    }

    //then go through for wheels
    for(size_t h=0; h<names.size(); ++h){
      for(size_t i=0; i<N; ++i){
        for(size_t j=0; j<2; ++j){
          for(size_t k=0; k<2; ++k){
            if(names[h].compare(model_name[i][j][k]) == 0){
              //ROS_INFO_STREAM(names[h] << " should equal " << model_name[i][j][k]);
              wheelLonSpeed[i][j][k] = get_lonSpeed_from_link_states(msg,h,axleYaw[i][j]);
              wheelLatSpeed[i][j][k] = get_latSpeed_from_link_states(msg,h,axleYaw[i][j]);
              wheelYaw[i][j][k] = axleYaw[i][j];
              wheelYawRate[i][j][k] = get_wheelYawRate_from_link_states(msg,h);
              goto found; //go to next point in message

            }
          }
        }
      }
      found:;
    }
}







void terrain_coefficient_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  for(int i=0; i<N; i++){
    for(int j=0; j<2; j++){
      for(int k=0; k<2; k++){
        Xt[i][j][k] = msg->data[(4*i)+(2*j)+k];
      }
    }
  }
  //ROS_INFO_STREAM("Xt " << Xt);
}



int main(int argc, char **argv){
	//ROS_INFO_STREAM("test");


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


  ros::Subscriber sub5 = terrain_model_node.subscribe("/dd_robot/joint_states", 10, joint_states_Callback);
  ros::Subscriber sub6 = terrain_model_node.subscribe("/gazebo/link_states", 10, link_states_Callback);


  ros::Subscriber sub8 = terrain_model_node.subscribe("/terrain_coefficient", 10, terrain_coefficient_Callback);

  ros::Subscriber sub10 = terrain_model_node.subscribe("/bumper_wheel_1_1_1", 10, bumper_wheel_1_1_1_Callback);
  ros::Subscriber sub11 = terrain_model_node.subscribe("/bumper_wheel_1_1_2", 10, bumper_wheel_1_1_2_Callback);
  ros::Subscriber sub12 = terrain_model_node.subscribe("/bumper_wheel_1_2_1", 10, bumper_wheel_1_2_1_Callback);
  ros::Subscriber sub13 = terrain_model_node.subscribe("/bumper_wheel_1_2_2", 10, bumper_wheel_1_2_2_Callback);

  ros::Subscriber sub14 = terrain_model_node.subscribe("/bumper_wheel_2_1_1", 10, bumper_wheel_2_1_1_Callback);
  ros::Subscriber sub15 = terrain_model_node.subscribe("/bumper_wheel_2_1_2", 10, bumper_wheel_2_1_2_Callback);
  ros::Subscriber sub16 = terrain_model_node.subscribe("/bumper_wheel_2_2_1", 10, bumper_wheel_2_2_1_Callback);
  ros::Subscriber sub17 = terrain_model_node.subscribe("/bumper_wheel_2_2_2", 10, bumper_wheel_2_2_2_Callback);

  ros::Subscriber sub18 = terrain_model_node.subscribe("/bumper_wheel_3_1_1", 10, bumper_wheel_3_1_1_Callback);
  ros::Subscriber sub19 = terrain_model_node.subscribe("/bumper_wheel_3_1_2", 10, bumper_wheel_3_1_2_Callback);
  ros::Subscriber sub20 = terrain_model_node.subscribe("/bumper_wheel_3_2_1", 10, bumper_wheel_3_2_1_Callback);
  ros::Subscriber sub21 = terrain_model_node.subscribe("/bumper_wheel_3_2_2", 10, bumper_wheel_3_2_2_Callback);




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



  double wheelSlip[N][2][2]; //left right then front to back
  double frictionTorque[N][2][2];
  double terrainResistanceForce[N][2][2];
  double latTerrainResistanceForce[N][2][2];
  double terrainResistanceTorque[N][2][2];
  double yawResistanceTorque[N][2][2];
  double forceSlip[N][2][2];


  ros::spinOnce();
  ros::Duration(1).sleep(); //allows callbacks to populate variables before calculations
  ros::spinOnce();
  ROS_INFO_STREAM("#########################################################");


	while(ros::ok()){
    //
    //calculate relevant vehicle variables
    //


    //slip
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          const double deadBand = 0.025;
          //if speed and wheel speed very small
          if(fabs(wheelRadius*wheelAngularSpeed[i][j][k]) < deadBand && fabs(wheelLonSpeed[i][j][k]) < deadBand){
            wheelSlip[i][j][k] = 0;
          }
          //if moving backwards slower than wheels
          else if(wheelLonSpeed[i][j][k] < 0 && wheelRadius*wheelAngularSpeed[i][j][k] < wheelLonSpeed[i][j][k]){
            wheelSlip[i][j][k] = -fabs(wheelLonSpeed[i][j][k] - wheelRadius*wheelAngularSpeed[i][j][k]) / fabs(wheelRadius*wheelAngularSpeed[i][j][k]);
          }
          //if moving backwards faster than wheels
          else if(wheelLonSpeed[i][j][k] < 0 && wheelRadius*wheelAngularSpeed[i][j][k] > wheelLonSpeed[i][j][k]){
            wheelSlip[i][j][k] = fabs(wheelRadius*wheelAngularSpeed[i][j][k] - wheelLonSpeed[i][j][k]) / fabs(wheelLonSpeed[i][j][k]);
          }
          //if moving forward faster than wheels
          else if(wheelLonSpeed[i][j][k] > 0 && wheelRadius*wheelAngularSpeed[i][j][k] < wheelLonSpeed[i][j][k]){
            wheelSlip[i][j][k] = -fabs(wheelLonSpeed[i][j][k] - wheelRadius*wheelAngularSpeed[i][j][k]) / fabs(wheelLonSpeed[i][j][k]);
          }
          //if moving forward slower than wheels
          else if(wheelLonSpeed[i][j][k] > 0 && wheelRadius*wheelAngularSpeed[i][j][k] > wheelLonSpeed[i][j][k]){
            wheelSlip[i][j][k] = fabs(wheelRadius*wheelAngularSpeed[i][j][k] - wheelLonSpeed[i][j][k]) / fabs(wheelRadius*wheelAngularSpeed[i][j][k]);
          }
          wheelSlip[i][j][k] = constrain(wheelSlip[i][j][k],-1,1);
        }
      }
    }


    //frictionTorque
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          //sign speed is used to add a non linear drop off when close to zero
          frictionTorque[i][j][k] = -(coeffFrictionTorque*fabs(wheelAngularSpeed[i][j][k])*signSpeed(wheelAngularSpeed[i][j][k]) + 0.1*signSpeed(wheelAngularSpeed[i][j][k],0.001));
          //ROS_INFO_STREAM("frictionTorque is " << frictionTorque[i][j][k]);
        }
      }
    }


    //terrainResistanceTorque
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          double COEFF = coeffCompactionResistanceTorque*Xt[i][j][k];
          if(wheelLonSpeed[i][j][k] > 0){
            if(wheelRadius*wheelAngularSpeed[i][j][k] > wheelLonSpeed[i][j][k]){ //spinning faster than moving
              terrainResistanceTorque[i][j][k] = -pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j][k] - wheelRadius*wheelAngularSpeed[i][j][k])*COEFF;
            }
            else if(wheelRadius*wheelAngularSpeed[i][j][k] < wheelLonSpeed[i][j][k]){  //spinning slower than moving
              terrainResistanceTorque[i][j][k] = pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j][k] - wheelRadius*wheelAngularSpeed[i][j][k])*COEFF;
            }
          }
          else if(wheelLonSpeed[i][j][k] < 0) { //wheel going backwards
            if(wheelRadius*wheelAngularSpeed[i][j][k] > wheelLonSpeed[i][j][k]){ //spinning slower than moving
              terrainResistanceTorque[i][j][k] = -pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j][k] - wheelRadius*wheelAngularSpeed[i][j][k])*COEFF;
            }
            else if(wheelRadius*wheelAngularSpeed[i][j][k] < wheelLonSpeed[i][j][k]){  //spinning faster than moving
              terrainResistanceTorque[i][j][k] = pow(wheelRadius,2)*wheelWidth*fabs(wheelLonSpeed[i][j][k] - wheelRadius*wheelAngularSpeed[i][j][k])*COEFF;
            }

          }
          //constrain, add Xt, and make sure wheel is on ground
          terrainResistanceTorque[i][j][k] = constrain(terrainResistanceTorque[i][j][k],-150,150)*normalForce[i][j][k];
        }
      }
    }

    //calculate terrain resistance forces
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          double COEFF = coeffTerrainResistanceForce/Xt[i][j][k];//0.2;
          //see equation 2.18 in thesis proposal
          //signSpeed prevents rapid switching between full forward and full backward values
          //terrainResistanceForce[i][j][k] = -COEFF*signDoubleBand(wheelLonSpeed[i][j][k],0,0.0001)*normalForce[i][j][k];
          terrainResistanceForce[i][j][k] = -COEFF*sign(wheelLonSpeed[i][j][k])*normalForce[i][j][k];
          //ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce[i][j][k]);
        }
      }
    }

    //calculate lateralTerrainResistanceForce
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          double COEFF = coeffLatTerrainResistanceForce/Xt[i][j][k];
          //terrainResistanceForce[i][j][k] = coeffTerrainResistanceForce*signSpeed(wheelLatSpeed[i][j][k])*isNotZero(normalForce[i][j][k]);
          //ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce[i][j][k]);
          //latTerrainResistanceForce[i][j][k] = COEFF*signDoubleBand(wheelLatSpeed[i][j][k],0.0001,0.05)*normalForce[i][j][k];
          latTerrainResistanceForce[i][j][k] = COEFF*sign(wheelLatSpeed[i][j][k])*normalForce[i][j][k];
        }
      }
    }

    // calculate force slip
    double D;
    double E = 1.0;
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          double B = 2.3*Xt[i][j][k];
          double C = 2.6/pow(Xt[i][j][k],0.5);
          //Equation from https://en.wikipedia.org/wiki/Hans_B._Pacejka
          //Pacejka, H. B. (2012). Tire and vehicle dynamics. Besselink, Igo (3rd ed.). Oxford: Butterworth-Heinemann. p. 165. ISBN 978-0-08-097016-5. OCLC 785829133
          D = pow(Xt[i][j][k],0.5)*normalForce[i][j][k];
          forceSlip[i][j][k] = D*sin(C*atan(B*wheelSlip[i][j][k] - E*(B*wheelSlip[i][j][k] - atan(B*wheelSlip[i][j][k]))))*isNotZero(normalForce[i][j][k]);

          //if the forceSlip isn't enough to overcome the resistance then set both forces to zero

          if(fabs(forceSlip[i][j][k]) < fabs(terrainResistanceForce[i][j][k])){
            //forceSlip[i][j][k] = 0;
            //terrainResistanceForce[i][j][k] = 0;
          }
        }
      }
    }


    //calculate yaw resistance torque
    //good terrain has better wheel bite makes it harder to spin in place
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          double COEFF = coeffYawResistanceTorque*Xt[i][j][k];
          yawResistanceTorque[i][j][k] = -COEFF*signSpeed(wheelYawRate[i][j][k])*normalForce[i][j][k];
        }
      }
    }

    ROS_INFO_STREAM("##################################");

    for(int i=0; i<1; ++i){
      for(int j=0; j<1; ++j){
          for(int k=0; k<1; ++k){
            ROS_INFO_STREAM("Xt is " << Xt[i][j][k]);
            ROS_INFO_STREAM("wheelYaw is " << wheelYaw[i][j][k]);
            //ROS_INFO_STREAM("normalForce is " << normalForce[i][j][k]);
            ROS_INFO_STREAM("wheelAngularSpeed*R is " << wheelAngularSpeed[i][j][k]*wheelRadius);
            //ROS_INFO_STREAM("frictionTorque is " << frictionTorque[i][j][k]);


            ROS_INFO_STREAM("wheelLonSpeed is " << wheelLonSpeed[i][j][k]);
            ROS_INFO_STREAM("terrainResistanceForce is " << terrainResistanceForce[i][j][k]);
            ROS_INFO_STREAM("terrainResistanceTorque is " << terrainResistanceTorque[i][j][k]);


            ROS_INFO_STREAM("wheelLatSpeed is " << wheelLatSpeed[i][j][k]);
            ROS_INFO_STREAM("latTerrainResistanceForce is " << latTerrainResistanceForce[i][j][k]);


            //ROS_INFO_STREAM("wheelYawRate is " << wheelYawRate[i][j][k]);
            //ROS_INFO_STREAM("yawResistanceTorque is " << yawResistanceTorque[i][j][k]);

            //ROS_INFO_STREAM("wheelSlip is " << wheelSlip[i][j][k]);
            //ROS_INFO_STREAM("forceSlip is " << forceSlip[i][j][k]);

            ROS_INFO_STREAM(" ");
        }
      }
    }

		ros::Duration duration_temp(1.0/loopRate);
    apply_wrench_req.duration = duration_temp;
    apply_joint_effort_req.duration = duration_temp;
    for(int i=0; i<N; ++i){
      for(int j=0; j<2; ++j){
        for(int k=0; k<2; ++k){
          //global reference frame
      		apply_wrench_req.wrench.force.x = (forceSlip[i][j][k] + terrainResistanceForce[i][j][k])*cos(wheelYaw[i][j][k]) + latTerrainResistanceForce[i][j][k]*sin(wheelYaw[i][j][k]); //this is good;
      		apply_wrench_req.wrench.force.y = (forceSlip[i][j][k] + terrainResistanceForce[i][j][k])*sin(wheelYaw[i][j][k]) + -latTerrainResistanceForce[i][j][k]*cos(wheelYaw[i][j][k]);//this is good
          apply_wrench_req.wrench.force.z = 0.0;
      		apply_wrench_req.wrench.torque.x = 0.0;
          apply_wrench_req.wrench.torque.y = 0.0;
          apply_wrench_req.wrench.torque.z = yawResistanceTorque[i][j][k];
          apply_wrench_req.body_name = model_name[i][j][k];
      		// call apply body wrench service
      		wrenchClient.call(apply_wrench_req, apply_wrench_resp);
          apply_joint_effort_req.effort = constrain(terrainResistanceTorque[i][j][k] + frictionTorque[i][j][k],-1000,1000);
          //apply_joint_effort_req.effort = constrain(1 + terrainResistanceTorque[i][j][k] + frictionTorque[i][j][k],-1000,1000);
      		apply_joint_effort_req.joint_name = joint_name[i][j][k];
      		// call apply body wrench service
      		jointEffortClient.call(apply_joint_effort_req, apply_joint_effort_resp);
        }
      }
    }

		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO_STREAM(" oops");
  return 0;
}
