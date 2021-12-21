//double coeffFrictionTorque = 0.1;//0.1;
//double coeffCompactionResistanceTorque = 0.42; //This had been 0.30
//double coeffTerrainResistanceForce = 0.165; // This had been 0.133
//const double coeffForceSlip = 0.7; //This had been 0.5 when I opened it, but 0.7 in the thesis
//double coeffYawResistanceTorque = 0.001;
//double coeffLatTerrainResistanceForce = 0.1;

const int notN = 6;


double max(double A, double B);

double min(double A, double B);

double minmax(double A, double B, double C);

double constrain(double number, double lowerLimit, double upperLimit);

double signSpeed(double number, double deadBand = 0.01);

double sign(double number, double deadBand = 0);

double signDoubleBand(double number, double deadBand1, double deadBand2);

int isNotZero(double number, double deadBand = 0.01);

double headingDifference(double goal, double current);


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
