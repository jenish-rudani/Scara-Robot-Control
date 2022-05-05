
#include "trajectoryPlanning.h"
#include <thread>
#include <iomanip>  

#define DEBUG_TRAJECTORY_PLANNING 1
#define sample_rate 10
bool TRAJECTORY_PLANNING = 0;

// Terajectory Planning ################################################################################################################
void trajectoryPlanning() {
  cout << "Trajectory Planning ( Three Via Points: A,B,C )" << endl;

  JOINT initialRobotJointParameters; // get the current joint values of the robotic arm
  GetConfiguration(initialRobotJointParameters);
  initialRobotJointParameters[0] = DEG2RAD(initialRobotJointParameters[0]);
  initialRobotJointParameters[1] = DEG2RAD(initialRobotJointParameters[1]);
  initialRobotJointParameters[3] = DEG2RAD(initialRobotJointParameters[3]);
  JOINT initialToolPositionWRTStation;
  WHERE(initialRobotJointParameters, initialToolPositionWRTStation);



  double trajectoryTime;
  double x_a = 0, y_a = 0, z_a = 0, phi_a = 0;
  double x_b = 0, y_b = 0, z_b = 0, phi_b = 0;
  double x_c = 0, y_c = 0, z_c = 0, phi_c = 0;
  double x_g = 0, y_g = 0, z_g = 0, phi_g = 0;
  JOINT A_positionVar, B_positionVar, C_positionVar, G_positionVar;
  cout << "Please, Enter time for Trajectory in Seconds: ";
  cin >> trajectoryTime;


  //A_positionVar[0] = 230;
  //A_positionVar[1] = 115;
  //A_positionVar[2] = 50;
  //A_positionVar[3] = DEG2RAD(0);
  //B_positionVar[0] = 230;
  //B_positionVar[1] = 0;
  //B_positionVar[2] = 50;
  //B_positionVar[3] = DEG2RAD(0);
  //C_positionVar[0] = 230;
  //C_positionVar[1] = -115;
  //C_positionVar[2] = 50;
  //C_positionVar[3] = DEG2RAD(0);
  //G_positionVar[0] = 230;
  //G_positionVar[1] = -230;
  //G_positionVar[2] = 50;
  //G_positionVar[3] = DEG2RAD(0);



  // Enter First Via Parameters here
  A_positionVar[0] = -200;
  A_positionVar[1] = 200;
  A_positionVar[2] = 120;
  A_positionVar[3] = DEG2RAD(25);

  // Enter Second Via Parameters here
  B_positionVar[0] = 290;
  B_positionVar[1] = 0;
  B_positionVar[2] = 90;
  B_positionVar[3] = DEG2RAD(30);

  // Enter Third Via Parameters here
  C_positionVar[0] = 300;
  C_positionVar[1] = -100;
  C_positionVar[2] = 40;
  C_positionVar[3] = DEG2RAD(40);

  // Enter Goal Position Parameters here
  G_positionVar[0] = -100;
  G_positionVar[1] = -200;
  G_positionVar[2] = 25;
  G_positionVar[3] = DEG2RAD(-80);

  cout << "\n\n*******************************************************\n";
  cout << " -- You have entered following parameters  --\n";
  cout << "\t    " << internal << setw(5) << "X" << internal << setw(7) << "Y" << internal << setw(7) << "Z" << internal << setw(8) << "phi" << endl;
  cout << "Current  :"; printXYZPhi(initialToolPositionWRTStation);
  cout << "\tA:"; printXYZPhi(A_positionVar);
  cout << "\tB:"; printXYZPhi(B_positionVar);
  cout << "\tC:"; printXYZPhi(C_positionVar);
  cout << "\tG:"; printXYZPhi(G_positionVar);
  cout << "*******************************************************\n\n";

  planPathBasedOnJointSpace(initialRobotJointParameters, A_positionVar, B_positionVar, C_positionVar, G_positionVar, trajectoryTime);
}

void planPathBasedOnJointSpace(JOINT& initialRobotJointParameters, JOINT& A_positionVar, JOINT& B_positionVar, JOINT& C_positionVar, JOINT& G_positionVar, double trajectoryTime) {

  printf("\n\n Current Config 1: %f |  2: %f |  d3: %f | 4: %f", RAD2DEG(initialRobotJointParameters[0]), RAD2DEG(initialRobotJointParameters[1]), (initialRobotJointParameters[2]), RAD2DEG(initialRobotJointParameters[3]));

  // Current to A
  JOINT nearSolution_A, farSolution_A;
  bool flagFirst = false, flagSecond = false;
  getSolutionsForInverseKIN(A_positionVar, initialRobotJointParameters, nearSolution_A, farSolution_A, flagFirst, flagSecond);
  if (!flagFirst && !flagSecond) {
    cout << "Error: A is out of workspace!" << endl;
    return;
  }

  // A to B
  JOINT nearSolution_B, farSolution_B;
  flagFirst = false, flagSecond = false;

  getSolutionsForInverseKIN(B_positionVar, nearSolution_A, nearSolution_B, farSolution_B, flagFirst, flagSecond);
  if (!flagFirst && !flagSecond) {
    cout << "Error: B is out of workspace!" << endl;
    return;
  }

  // B to C
  JOINT nearSolution_C, farSolution_C;
  flagFirst = false, flagSecond = false;
  getSolutionsForInverseKIN(C_positionVar, nearSolution_B, nearSolution_C, farSolution_C, flagFirst, flagSecond);
  if (!flagFirst && !flagSecond) {
    cout << "Error: C is out of workspace!" << endl;
    return;
  }

  // C to G
  JOINT nearSolution_G, farSolution_G;
  flagFirst = false, flagSecond = false;
  getSolutionsForInverseKIN(G_positionVar, nearSolution_C, nearSolution_G, farSolution_G, flagFirst, flagSecond);
  if (!flagFirst && !flagSecond) {
    cout << "Error: G is out of workspace!" << endl;
    return;
  }


  if (DEBUG_TRAJECTORY_PLANNING) {
    cout << "\n\n*******************************************************************\n";
    cout << " ------------ JOINT VALUES FROM INVERSE KIN --------------- " << endl;
    cout << "\t\t\t" << internal << setw(10) << "Theta1" << internal << setw(12) << "Theta2" << internal << setw(8) << "D3" << internal << setw(12) << "Theta4" << endl;
    cout << "\tCurrent Joint -: "; printJointParameters(initialRobotJointParameters); cout << endl;
    cout << "\tNear Solution A: "; printJointParameters(nearSolution_A);
    cout << "\tFar Solution  A: "; printJointParameters(farSolution_A); cout << endl;

    cout << "\tNear Solution B: "; printJointParameters(nearSolution_B);
    cout << "\tFar Solution  B: "; printJointParameters(farSolution_B); cout << endl;

    cout << "\tNear Solution C: "; printJointParameters(nearSolution_C);
    cout << "\tFar Solution  C: "; printJointParameters(farSolution_C); cout << endl;

    cout << "\tNear Solution G: "; printJointParameters(nearSolution_G);
    cout << "\tFar Solution  G: "; printJointParameters(farSolution_G); cout << endl;

    cout << "*******************************************************************\n\n";
  }

  arrayOf5 trajectoryTimeSegments = { trajectoryTime * 0, trajectoryTime * 0.25, trajectoryTime * 0.5, trajectoryTime * 0.75, trajectoryTime * 1 };

  if (DEBUG_TRAJECTORY_PLANNING) {
    cout << "\n\n*******************************************************************\n";
    cout << " -- Trajectory time segments based on User trajectory time -- \n\t";
    cout << "Segments: " << trajectoryTimeSegments[0] << " | " << trajectoryTimeSegments[1] << " | " << trajectoryTimeSegments[2] << " | " << trajectoryTimeSegments[3] << " | " << trajectoryTimeSegments[4] << endl;
    cout << "*******************************************************************\n\n";
  }

  /*
  * We have all of the joint values for positions: A, B, C, G
  * Computing the Coefficients in Cubic Polynomials for each of the joint values

  CurrentConfig: {theta1, theta2, d3, theta3}
  A: {theta1, theta2, d3, theta3}
  B: {theta1, theta2, d3, theta3}
  C: {theta1, theta2, d3, theta3}
  G: {theta1, theta2, d3, theta3}

  to

  jointArr_theta1 = CurrentConfig_theta1, A_theta1, B_theta1. C_theta1, G_theta1
  jointArr_theta2 = CurrentConfig_theta2, A_theta2, B_theta2. C_theta2, G_theta2
  jointArr_d3 = CurrentConfig_d3, A_d3, B_d3, C_d3, G_d3
  jointArr_theta4 = CurrentConfig_theta4, A_theta4, B_theta4, C_theta4, G_theta4

  */
  arrayOf5 jointArr_theta1, jointArr_theta2, jointArr_d3, jointArr_theta4;
  prepareJointParamForEachFrames(initialRobotJointParameters, nearSolution_A, nearSolution_B, nearSolution_C, nearSolution_G, jointArr_theta1, jointArr_theta2, jointArr_d3, jointArr_theta4);

  vector<vector<double>> currJConfig2A_coeff, A2B_coeff, B2C_coeff, C2G_coeff;
  JOINT currJConfig2A_coeff_1, A2B_coeff_1, B2C_coeff_1, C2G_coeff_1;
  calculateCoefficients(jointArr_theta1, trajectoryTimeSegments, currJConfig2A_coeff_1, A2B_coeff_1, B2C_coeff_1, C2G_coeff_1);


  JOINT currJConfig2A_coeff_2, A2B_coeff_2, B2C_coeff_2, C2G_coeff_2;
  calculateCoefficients(jointArr_theta2, trajectoryTimeSegments, currJConfig2A_coeff_2, A2B_coeff_2, B2C_coeff_2, C2G_coeff_2);

  JOINT currJConfig2A_coeff_3, A2B_coeff_3, B2C_coeff_3, C2G_coeff_3;
  calculateCoefficients(jointArr_d3, trajectoryTimeSegments, currJConfig2A_coeff_3, A2B_coeff_3, B2C_coeff_3, C2G_coeff_3);

  JOINT currJConfig2A_coeff_4, A2B_coeff_4, B2C_coeff_4, C2G_coeff_4;
  calculateCoefficients(jointArr_theta4, trajectoryTimeSegments, currJConfig2A_coeff_4, A2B_coeff_4, B2C_coeff_4, C2G_coeff_4);

  currJConfig2A_coeff.push_back(vector<double>(currJConfig2A_coeff_1, currJConfig2A_coeff_1 + 4));
  currJConfig2A_coeff.push_back(vector<double>(currJConfig2A_coeff_2, currJConfig2A_coeff_2 + 4));
  currJConfig2A_coeff.push_back(vector<double>(currJConfig2A_coeff_3, currJConfig2A_coeff_3 + 4));
  currJConfig2A_coeff.push_back(vector<double>(currJConfig2A_coeff_4, currJConfig2A_coeff_4 + 4));

  A2B_coeff.push_back(vector<double>(A2B_coeff_1, A2B_coeff_1 + 4));
  A2B_coeff.push_back(vector<double>(A2B_coeff_2, A2B_coeff_2 + 4));
  A2B_coeff.push_back(vector<double>(A2B_coeff_3, A2B_coeff_3 + 4));
  A2B_coeff.push_back(vector<double>(A2B_coeff_4, A2B_coeff_4 + 4));

  B2C_coeff.push_back(vector<double>(B2C_coeff_1, B2C_coeff_1 + 4));
  B2C_coeff.push_back(vector<double>(B2C_coeff_2, B2C_coeff_2 + 4));
  B2C_coeff.push_back(vector<double>(B2C_coeff_3, B2C_coeff_3 + 4));
  B2C_coeff.push_back(vector<double>(B2C_coeff_4, B2C_coeff_4 + 4));

  C2G_coeff.push_back(vector<double>(C2G_coeff_1, C2G_coeff_1 + 4));
  C2G_coeff.push_back(vector<double>(C2G_coeff_2, C2G_coeff_2 + 4));
  C2G_coeff.push_back(vector<double>(C2G_coeff_3, C2G_coeff_3 + 4));
  C2G_coeff.push_back(vector<double>(C2G_coeff_4, C2G_coeff_4 + 4));


  if (DEBUG_TRAJECTORY_PLANNING) {
    cout << "\n\n*******************************************************************\n";
    cout << " ------- Cubic Coefficients : [ a0, a1, a2, a3 ]  -------\n\n";
    displayJointVar(currJConfig2A_coeff, A2B_coeff, B2C_coeff, C2G_coeff);
    cout << "*******************************************************************\n\n";
  }


  vector<double> currTimeVec;
  vector<vector<double>> pathVec;
  generatePath(trajectoryTimeSegments, currJConfig2A_coeff, A2B_coeff, B2C_coeff, C2G_coeff, pathVec, currTimeVec);

  vector<vector<double>> velocityVec;
  generateVelocity(trajectoryTimeSegments, currJConfig2A_coeff, A2B_coeff, B2C_coeff, C2G_coeff, velocityVec, currTimeVec);

  vector<vector<double>> accelerationVec;
  generateAcceleration(trajectoryTimeSegments, currJConfig2A_coeff, A2B_coeff, B2C_coeff, C2G_coeff, accelerationVec, currTimeVec);

  // X vs Y Values
  vector<double> x, y, z, phi, xTarget, yTarget;
  genPos(pathVec[0], pathVec[1], pathVec[2], pathVec[3], x, y, z, phi);
  vector<pair<string, vector<double>>> xy_vals = { {"X", x}, {"Y", y}, {"Z", z}, {"Phi", phi} };
  WriteParamToCsvFile("./MATLAB/X_vs_Y.csv", xy_vals);


  JOINT initialToolPositionWRTStation;
  WHERE(initialRobotJointParameters, initialToolPositionWRTStation);
  // Get the target X and Y values for all (4) via points

  //TODO:
  xTarget.push_back(round(initialToolPositionWRTStation[0]));
  xTarget.push_back(A_positionVar[0]);
  xTarget.push_back(B_positionVar[0]);
  xTarget.push_back(C_positionVar[0]);
  xTarget.push_back(G_positionVar[0]);

  yTarget.push_back(initialToolPositionWRTStation[1]);
  yTarget.push_back(A_positionVar[1]);
  yTarget.push_back(B_positionVar[1]);
  yTarget.push_back(C_positionVar[1]);
  yTarget.push_back(G_positionVar[1]);

  vector<pair<string, vector<double>>> XY_TargetValues = { {"X_targ", xTarget}, {"Y_targ", yTarget} };
  WriteParamToCsvFile("./MATLAB/XY_InputParameters.csv", XY_TargetValues);

  // Checking Velocity and Acceleration limits
  bool vlStatus = checkVelocityLimits(velocityVec);
  if (!vlStatus) {
    cout << "\n!!!!!!!!!!!!!---- ERROR ----!!!!!!!!!!!!!!!!\n";
    cout << "Exceeded Velocity limits!\n";
  }
  bool accStatus = checkAccLimits(accelerationVec);
  if (!accStatus) {
    cout << "\n!!!!!!!!!!!!!---- ERROR ----!!!!!!!!!!!!!!!!\n";
    cout << "Exceeded Velocity limits!\n";
  }

  bool continueToMove;
  cout << "\n\n\tDo you want to move the robot ? [ 1/0 ] : ";
  cin >> continueToMove;

  vector<double> currJConfig2A_PositionVec, A2B_PositionVec, B2C_PositionVec, C2G_PositionVec;
  vector<double> currJConfig2A_VelVec, A2B_VelVec, B2C_VelVec, C2G_VelVec;
  vector<double> currJConfig2A_AccVec, A2B_AccVec, B2C_AccVec, C2G_AccVec;

  currJConfig2A_PositionVec = pathVec[0];
  A2B_PositionVec = pathVec[1];
  B2C_PositionVec = pathVec[2];
  C2G_PositionVec = pathVec[3];

  currJConfig2A_VelVec = velocityVec[0];
  A2B_VelVec = velocityVec[1];
  B2C_VelVec = velocityVec[2];
  C2G_VelVec = velocityVec[3];

  currJConfig2A_AccVec = accelerationVec[0];
  A2B_AccVec = accelerationVec[1];
  B2C_AccVec = accelerationVec[2];
  C2G_AccVec = accelerationVec[3];

  // Calculating Increment
  int lengthOfVector = currJConfig2A_PositionVec.size();
  double maximumTime = currTimeVec[lengthOfVector - 1];
  int inc = int(1000 * (maximumTime) / lengthOfVector);

  printf("Moving to (%f, %f, %f, %f)\n", RAD2DEG(currJConfig2A_PositionVec[lengthOfVector - 1]), RAD2DEG(A2B_PositionVec[lengthOfVector - 1]), B2C_PositionVec[lengthOfVector - 1], RAD2DEG(C2G_PositionVec[lengthOfVector - 1]));

  for (int i = 0; i < lengthOfVector; i++) {


    JOINT pos{ RAD2DEG(currJConfig2A_PositionVec[i]), RAD2DEG(A2B_PositionVec[i]), B2C_PositionVec[i], RAD2DEG(C2G_PositionVec[i]) };
    JOINT vel{ RAD2DEG(currJConfig2A_VelVec[i]), RAD2DEG(A2B_VelVec[i]), B2C_VelVec[i], RAD2DEG(C2G_VelVec[i]) };
    JOINT acc{ RAD2DEG(currJConfig2A_AccVec[i]), RAD2DEG(A2B_AccVec[i]), B2C_AccVec[i], RAD2DEG(C2G_AccVec[i]) };

    MoveWithConfVelAcc(pos, vel, acc);

    //printf("Pos #%d: (%f, %f, %f, %f)\n", i + 1, RAD2DEG(currJConfig2A_PositionVec[i]), RAD2DEG(A2B_PositionVec[i]), B2C_PositionVec[i], RAD2DEG(C2G_PositionVec[i]));

    std::this_thread::sleep_for(std::chrono::milliseconds(inc));
    continue;
  }
  StopRobot();
  ResetRobot();
  return;



}

bool checkVelocityLimits(vector<vector<double>> vals) {
  // Checks to see if velocity limits are exceeded
  vector<double> j1, j2, j3, j4;
  j1 = vals[0];
  j2 = vals[1];
  j3 = vals[2];
  j4 = vals[3];
  bool valid;
  int sz = size(j1);
  for (int i = 0; i < sz; i++) {
    bool j1_val = abs(j1[i]) <= JOINT1_VEL_LIM;
    bool j2_val = abs(j2[i]) <= JOINT2_VEL_LIM;
    bool j3_val = abs(j3[i]) <= JOINT3_VEL_LIM;
    bool j4_val = abs(j4[i]) <= JOINT4_VEL_LIM;

    valid = j1_val && j2_val && j3_val && j4_val;
    if (!valid) {
      return false;
    }
  }
  return true;
}

bool checkAccLimits(vector<vector<double>> vals) {

  // Check If acceleration limits are exceeded
  vector<double> j1, j2, j3, j4;
  j1 = vals[0];
  j2 = vals[1];
  j3 = vals[2];
  j4 = vals[3];
  bool valid;
  int sz = size(j1);
  for (int i = 0; i < sz; i++) {
    bool j1_val = abs(j1[i]) <= JOINT1_ACC_LIM;
    bool j2_val = abs(j2[i]) <= JOINT2_ACC_LIM;
    bool j3_val = abs(j3[i]) <= JOINT3_ACC_LIM;
    bool j4_val = abs(j4[i]) <= JOINT4_ACC_LIM;

    valid = j1_val && j2_val && j3_val && j4_val;
    if (!valid) {
      return false;
    }
  }
  return true;
}

void genPos(vector<double> theta1, vector<double> theta2, vector<double>& d3, vector<double> theta4, vector<double>& x, vector<double>& y, vector<double>& z, vector<double>& phi) {

  // Computes the X vs Y and returns it
  int sizeOfVar = size(theta1);
  for (int i = 0; i < sizeOfVar; i++) {
    JOINT currJointConf{ theta1[i], theta2[i], d3[i], theta4[i] };
    JOINT currPos;

    // Use WHERE to get the (x, y, z, phi) of TrelS
    WHERE(currJointConf, currPos);

    // Save position to the outputs
    x.push_back(currPos[0]);
    y.push_back(currPos[1]);
    z.push_back(currPos[2]);
    phi.push_back(currPos[3]);
  }
}

void generateAcceleration(arrayOf5& timeArr, vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff, vector<vector<double>>& accelerationVec, vector<double>& currTimeVec) {

  vector<double> theta1_acc, theta2_acc, d3_acc, theta4_acc;

  for (int i = 0; i < 4; i++) {

    switch (i)
    {
    case 0:
      genAccelerationHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta1_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta1_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta1_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta1_acc, currTimeVec, true);
      accelerationVec.push_back(theta1_acc);
      break;
    case 1:
      genAccelerationHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta2_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta2_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta2_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta2_acc, currTimeVec, true);
      accelerationVec.push_back(theta2_acc);
      break;
    case 2:
      genAccelerationHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], d3_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], d3_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], d3_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], d3_acc, currTimeVec, true);
      accelerationVec.push_back(d3_acc);
      break;
    case 3:
      genAccelerationHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta4_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta4_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta4_acc, currTimeVec, true);
      genAccelerationHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta4_acc, currTimeVec, true);
      accelerationVec.push_back(theta4_acc);
      break;
    }
  }
}

void genAccelerationHelperFunction(double ti, double tf, vector<double>& coeff, vector<double>& acc, vector<double>& currTimeVec, bool isFull) {

  // Units: rad/s^2
  double t = tf - ti;
  int totalNumberOfPoints = (t * sample_rate) + 1; // + 2 to get the final position as well
  for (int i = 0; i < totalNumberOfPoints; i++) {
    if (isFull == false) currTimeVec.push_back(ti + i / sample_rate);
    acc.push_back(2 * coeff[2] + 6 * coeff[3] * currTimeVec[i]);
  }

}

void generateVelocity(arrayOf5& timeArr, vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff, vector<vector<double>>& velocityVec, vector<double>& currTimeVec) {

  vector<double> theta1_vel, theta2_vel, d3_vel, theta4_vel;

  for (int i = 0; i < 4; i++) {

    switch (i)
    {
    case 0:
      genVelocityHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta1_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta1_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta1_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta1_vel, currTimeVec, true);
      velocityVec.push_back(theta1_vel);
      break;
    case 1:
      genVelocityHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta2_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta2_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta2_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta2_vel, currTimeVec, true);
      velocityVec.push_back(theta2_vel);
      break;
    case 2:
      genVelocityHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], d3_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], d3_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], d3_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], d3_vel, currTimeVec, true);
      velocityVec.push_back(d3_vel);
      break;
    case 3:
      genVelocityHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta4_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta4_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta4_vel, currTimeVec, true);
      genVelocityHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta4_vel, currTimeVec, true);
      velocityVec.push_back(theta4_vel);
      break;
    }
  }


}

void genVelocityHelperFunction(double ti, double tf, vector<double>& coeff, vector<double>& vel, vector<double>& currTimeVec, bool isFull) {

  // Computes the velocity of the path vs. time
  // Units: rad/s
  double t = tf - ti;
  int totalPoints = (t * sample_rate) + 1;
  for (int i = 0; i < totalPoints; i++) {
    if (isFull == false) currTimeVec.push_back(ti + i / sample_rate);
    vel.push_back(coeff[1] + 2 * coeff[2] * currTimeVec[i] + 3 * coeff[3] * pow(currTimeVec[i], 2));
  }

}

void generatePath(arrayOf5& timeArr, vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff, vector<vector<double>>& pathVector, vector<double>& currTimeVec) {


  vector<double> theta1_pos, theta2_pos, d3_pos, theta4_pos;

  for (int i = 0; i < 4; i++) {

    switch (i)
    {
    case 0:
      genPathHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta1_pos, currTimeVec, false);
      genPathHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta1_pos, currTimeVec, false);
      genPathHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta1_pos, currTimeVec, false);
      genPathHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta1_pos, currTimeVec, false);
      pathVector.push_back(theta1_pos);
      break;
    case 1:
      genPathHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta2_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta2_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta2_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta2_pos, currTimeVec, true);
      pathVector.push_back(theta2_pos);
      break;
    case 2:
      genPathHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], d3_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], d3_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], d3_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], d3_pos, currTimeVec, true);
      pathVector.push_back(d3_pos);
      break;
    case 3:
      genPathHelperFunction(timeArr[0], timeArr[1], currJConfig2A_coeff[i], theta4_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[1], timeArr[2], A2B_coeff[i], theta4_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[2], timeArr[3], B2C_coeff[i], theta4_pos, currTimeVec, true);
      genPathHelperFunction(timeArr[3], timeArr[4], C2G_coeff[i], theta4_pos, currTimeVec, true);
      pathVector.push_back(theta4_pos);
      break;
    }
  }

}

void genPathHelperFunction(double ti, double tf, vector<double>& coeff, vector<double>& pos, vector<double>& currTimeVec, bool isFull) {

  double t = tf - ti;
  int totalNumberOfPoints = (t * sample_rate) + 1; // + 2 to get the final position as well
  for (double i = 0; i < totalNumberOfPoints; i++) {
    if (isFull == false) currTimeVec.push_back(ti + i / sample_rate);
    pos.push_back(coeff[0] + coeff[1] * currTimeVec[i] + coeff[2] * pow(currTimeVec[i], 2) + coeff[3] * pow(currTimeVec[i], 3));
  }
}

void calculateCoefficients(arrayOf5& jointParamArr, arrayOf5& trajectoryTimeSegments, JOINT& currJConfig2A_coeff, JOINT& A2B_coeff, JOINT& B2C_coeff, JOINT& C2G_coeff) {

  arrayOf4 slopes;

  arrayOf4 diffTrajectoryTimeSegments{ trajectoryTimeSegments[1] - trajectoryTimeSegments[0], trajectoryTimeSegments[2] - trajectoryTimeSegments[1], trajectoryTimeSegments[3] - trajectoryTimeSegments[2], trajectoryTimeSegments[4] - trajectoryTimeSegments[3] };

  int length = sizeof(slopes) / sizeof(*slopes);
  for (int i = 0; i < length; i++) {
    slopes[i] = (jointParamArr[i + 1] - jointParamArr[i]) / (trajectoryTimeSegments[i + 1] - trajectoryTimeSegments[i]);
  }


  /* computes the velocity at each via point
  - if slope's sign change, vel = 0
  - if slope's sign does not change, vel = average of two slopes */
  arrayOf3 temp1;
  length = sizeof(temp1) / sizeof(*temp1);
  for (int i = 0; i < length; i++) {
    if (slopes[i] > 0 && slopes[i + 1] < 0 || slopes[i] < 0 && slopes[i + 1] > 0) {
      temp1[i] = 0;
    }
    else {
      temp1[i] = (slopes[i] + slopes[i + 1]) / 2;
    }
  }
  arrayOf5 vels{ 0 , temp1[0], temp1[1],  temp1[2], 0 };

  // curr -> A
  calculateCubicCoefficients(jointParamArr[0], jointParamArr[1], vels[0], vels[1], diffTrajectoryTimeSegments[0], currJConfig2A_coeff);

  // A -> B
  calculateCubicCoefficients(jointParamArr[1], jointParamArr[2], vels[1], vels[2], diffTrajectoryTimeSegments[1], A2B_coeff);

  // B -> C
  calculateCubicCoefficients(jointParamArr[2], jointParamArr[3], vels[2], vels[3], diffTrajectoryTimeSegments[2], B2C_coeff);

  // C -> G
  calculateCubicCoefficients(jointParamArr[3], jointParamArr[4], vels[3], vels[4], diffTrajectoryTimeSegments[3], C2G_coeff);
}

void calculateCubicCoefficients(double theta0, double thetaf, double vel0, double velf, double tf, JOINT& coeff) {

  // Calculates the cubic coefficients
  // Assumes constant velocity between points
  // See eqn 7.11 of text (pg 207 of text OR 215/408 of pdfr)

  double a0, a1, a2, a3;
  a0 = theta0;
  a1 = vel0;
  a2 = (3 / pow(tf, 2)) * (thetaf - theta0) - (2 / tf) * vel0 - (1 / tf) * velf;
  a3 = -(2 / pow(tf, 3)) * (thetaf - theta0) + (1 / pow(tf, 2)) * (velf + vel0);

  JOINT tempJointArr{ a0, a1, a2, a3 };
  copyArray(tempJointArr, coeff);
}

void prepareJointParamForEachFrames(JOINT& currentJointConfiguration, JOINT& nearSolution_A, JOINT& nearSolution_B, JOINT& nearSolution_C, JOINT& nearSolution_G, arrayOf5& jointArr_theta1, arrayOf5& jointArr_theta2, arrayOf5& jointArr_d3, arrayOf5& jointArr_theta4) {

  seprateJointParametersPerIndex(jointArr_theta1, currentJointConfiguration, nearSolution_A, nearSolution_B, nearSolution_C, nearSolution_G, 0);
  seprateJointParametersPerIndex(jointArr_theta2, currentJointConfiguration, nearSolution_A, nearSolution_B, nearSolution_C, nearSolution_G, 1);
  seprateJointParametersPerIndex(jointArr_d3, currentJointConfiguration, nearSolution_A, nearSolution_B, nearSolution_C, nearSolution_G, 2);
  seprateJointParametersPerIndex(jointArr_theta4, currentJointConfiguration, nearSolution_A, nearSolution_B, nearSolution_C, nearSolution_G, 3);


  if (DEBUG_TRAJECTORY_PLANNING) {

    cout << "\n\n*******************************************************************\n";
    cout << "  -- Combined Joint Values from all Frames --\n";
    cout << "\tTheta1: " << RAD2DEG(jointArr_theta1[0]) << " | " << RAD2DEG(jointArr_theta1[1]) << " | " << RAD2DEG(jointArr_theta1[2]) << " | " << RAD2DEG(jointArr_theta1[3]) << " | " << RAD2DEG(jointArr_theta1[4]) << endl;
    cout << "\tTheta2: " << RAD2DEG(jointArr_theta2[0]) << " | " << RAD2DEG(jointArr_theta2[1]) << " | " << RAD2DEG(jointArr_theta2[2]) << " | " << RAD2DEG(jointArr_theta2[3]) << " | " << RAD2DEG(jointArr_theta2[4]) << endl;
    cout << "\tD3    : " << jointArr_d3[0] << " | " << jointArr_d3[1] << " | " << jointArr_d3[2] << " | " << jointArr_d3[3] << " | " << jointArr_d3[4] << endl;
    cout << "\tTheta4: " << RAD2DEG(jointArr_theta4[0]) << " | " << RAD2DEG(jointArr_theta4[1]) << " | " << RAD2DEG(jointArr_theta4[2]) << " | " << RAD2DEG(jointArr_theta4[3]) << " | " << RAD2DEG(jointArr_theta4[4]) << endl;
    cout << "*******************************************************************\n\n";
  }

}
