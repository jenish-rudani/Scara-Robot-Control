#include "ensc894.h"
#include "ensc-488.h"
#include "StdAfx.h"
#include <thread>

using namespace std;

/*

** Important : All Rotations are around Z-axis


Units
– Deg for revolute joints.
– MM for prismatic joints.
– Predefined Macros
• DEG2RAD(x) / RAD2DEG(x)
• MM2M(x) / M2MM(x)
*/

#define DEBUG_TRAJECTORY_PLANNING 1
#define sample_rate 100
bool TRAJECTORY_PLANNING = 0;


transformMatrix base_Relativeto_Station_T, tool_Relativeto_Wrist_T;
bool gripStatus = false;

int main(void) {

  // Console UI
  /* Menu Options:
  1. Forward Kinematics
  2. Inverse Kinematics
  3. Pick and Place
  3. Stop and Reset Robot
  4. Close Gripper
  0. Exit
  */

  int inputOption;

  JOINT jointParameters, sPt;
  JOINT T{ 0, 0, 140, 0 };
  JOINT B{ 0, 0, 405, 0 };
  parametersToTransformMatrix(B, base_Relativeto_Station_T);
  parametersToTransformMatrix(T, tool_Relativeto_Wrist_T);

  while (true) {

    cout << "\n\n****************************************\n\n";
    cout << "Please choose a number [1,2,3,4] :\n\n";
    cout << "\t1. Forward Kinematics\n";
    cout << "\t2. Inverse Kinematics\n";
    cout << "\t3. Pick and Place\n";
    cout << "\t4. Stop+Reset Robot\n";
    cout << "\t5. Trajectory Planning, Input: 4x1 V(x,y,z,phi)\n";
    cout << "\t6. Toggle Gripper\n";
    cout << "\t0. Exit\n" << endl;
    cout << "****************************************\n\n";
    printf("Please Choose an option from above: ");

    cin >> inputOption;

    switch (inputOption) {

    case 1:
      forwardKinematics(jointParameters, sPt);
      break;
    case 2:
      inverseKinematics(sPt, false, false);
      break;
    case 3:
      runPickAndPlace();
      break;
    case 4:
      printf("\n\nStopping and Resetting Robot\n\n");
      StopRobot();
      ResetRobot();
      break;
    case 5:
      printf("\n\nStarting Trajectory Planning\n\n");
      TRAJECTORY_PLANNING = 1;
      trajectoryPlanning();
      TRAJECTORY_PLANNING = 0;
    case 6:
      toggleGripper(gripStatus);
      break;
    case 0: // Exit
      return 0;
      break;
    default:
      printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\nInvalid Option, Enter Again. To Exit Enter '0'.\n\n\n");
      break;
    }
  }

  return 0;
}

// Terajectory Planning ################################################################################################################
void trajectoryPlanning() {
  cout << "Trajectory Planning ( Three Via Points: A,B,C )" << endl;

  double trajectoryTime;
  double x_a, y_a, z_a, phi_a;
  double x_b, y_b, z_b, phi_b;
  double x_c, y_c, z_c, phi_c;
  double x_g, y_g, z_g, phi_g;
  JOINT A_positionVar, B_positionVar, C_positionVar, G_positionVar;
  cout << "Please, Enter time for Trajectory in Seconds: ";
  cin >> trajectoryTime;

  if (!DEBUG_TRAJECTORY_PLANNING)
  {
    char tempString[] = "Please, Enter Parameters for Via Point 'A'";
    getToolPositionFromUser(x_a, y_a, z_a, phi_a, tempString);

    strcpy_s(tempString, "Please, Enter Parameters Via Point 'B'");
    getToolPositionFromUser(x_b, y_b, z_b, phi_b, tempString);

    strcpy_s(tempString, "Please, Enter Parameters Via Point 'C'");
    getToolPositionFromUser(x_c, y_c, z_c, phi_c, tempString);


    strcpy_s(tempString, "Please, Enter Parameters for Point 'G'");
    getToolPositionFromUser(x_g, y_g, z_g, phi_g, tempString);

    JOINT A_positionVar{ x_a, y_a, z_a, DEG2RAD(phi_a) };
    JOINT B_positionVar{ x_b, y_b, z_b, DEG2RAD(phi_b) };
    JOINT C_positionVar{ x_c, y_c, z_c, DEG2RAD(phi_c) };
    JOINT G_positionVar{ x_g, y_g, z_g, DEG2RAD(phi_g) };
    cout << "\n\n*******************************************************\n";
    cout << " -- You have entered following parameters  --\n";
    cout << "\tFrame:  X   |  Y  |  Z  | phi\n";
    cout << "\tA:   [ " << A_positionVar[0] << "  | " << A_positionVar[1] << " | " << A_positionVar[2] << " | " << A_positionVar[3] << " ]" << endl;
    cout << "\tB:   [" << B_positionVar[0] << "  | " << B_positionVar[1] << " | " << B_positionVar[2] << " | " << B_positionVar[3] << " ]" << endl;
    cout << "\tC:   [" << C_positionVar[0] << "  |" << C_positionVar[1] << " | " << C_positionVar[2] << " | " << C_positionVar[3] << " ]" << endl;
    cout << "\tG:   [ " << G_positionVar[0] << "  |" << G_positionVar[1] << " | " << G_positionVar[2] << " | " << G_positionVar[3] << " ]" << endl;
    cout << "*******************************************************\n\n";
  }
  else {
    A_positionVar[0] = 100;
    A_positionVar[1] = 250;
    A_positionVar[2] = 100;
    A_positionVar[3] = 30;

    B_positionVar[0] = -100;
    B_positionVar[1] = 250;
    B_positionVar[2] = 100;
    B_positionVar[3] = 30;

    C_positionVar[0] = -100;
    C_positionVar[1] = -250;
    C_positionVar[2] = 100;
    C_positionVar[3] = 30;

    G_positionVar[0] = 100;
    G_positionVar[1] = -250;
    G_positionVar[2] = 100;
    G_positionVar[3] = 30;


    cout << "\n\n*******************************************************\n";
    cout << " -- You have entered following parameters  --\n";
    cout << "\tFrame:  X   |  Y  |  Z  | phi\n";
    cout << "\tA:   [ " << A_positionVar[0] << "  | " << A_positionVar[1] << " | " << A_positionVar[2] << " | " << A_positionVar[3] << " ]" << endl;
    cout << "\tB:   [" << B_positionVar[0] << "  | " << B_positionVar[1] << " | " << B_positionVar[2] << " | " << B_positionVar[3] << " ]" << endl;
    cout << "\tC:   [" << C_positionVar[0] << "  |" << C_positionVar[1] << " | " << C_positionVar[2] << " | " << C_positionVar[3] << " ]" << endl;
    cout << "\tG:   [ " << G_positionVar[0] << "  |" << G_positionVar[1] << " | " << G_positionVar[2] << " | " << G_positionVar[3] << " ]" << endl;
    cout << "*******************************************************\n\n";
  }


  JOINT currentJointConfiguration; // get the current joint values of the robotic arm
  GetConfiguration(currentJointConfiguration);
  currentJointConfiguration[0] = DEG2RAD(currentJointConfiguration[0]);
  currentJointConfiguration[1] = DEG2RAD(currentJointConfiguration[1]);
  currentJointConfiguration[3] = DEG2RAD(currentJointConfiguration[3]);

  vector<vector<double>> trajectoryValues;
  planPathBasedOnJointSpace(currentJointConfiguration, A_positionVar, B_positionVar, C_positionVar, G_positionVar, trajectoryTime);
}

void planPathBasedOnJointSpace(JOINT& currentJointConfiguration, JOINT& A_positionVar, JOINT& B_positionVar, JOINT& C_positionVar, JOINT& G_positionVar, double trajectoryTime) {



  // Current to A
  JOINT nearSolution_A, farSolution_A;
  bool flagFirst = false, flagSecond = false;
  getSolutionsForInverseKIN(A_positionVar, currentJointConfiguration, nearSolution_A, farSolution_A, flagFirst, flagSecond);
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
    cout << " -- Joint Values in Order : " << " Theta1 | " << "Theta2 | " << "D3 |" << " Theta4  --" << endl;
    cout << "\tCurrent Joint  : " << currentJointConfiguration[0] << " | " << currentJointConfiguration[1] << " | " << currentJointConfiguration[2] << " | " << currentJointConfiguration[3] << endl;
    cout << "\tNear Solution A: " << nearSolution_A[0] << " | " << nearSolution_A[1] << " | " << nearSolution_A[2] << " | " << nearSolution_A[3] << endl;
    cout << "\tNear Solution B: " << nearSolution_B[0] << " | " << nearSolution_B[1] << " | " << nearSolution_B[2] << " | " << nearSolution_B[3] << endl;
    cout << "\tNear Solution C: " << nearSolution_C[0] << " | " << nearSolution_C[1] << " | " << nearSolution_C[2] << " | " << nearSolution_C[3] << endl;
    cout << "\tNear Solution G: " << nearSolution_G[0] << " | " << nearSolution_G[1] << " | " << nearSolution_G[2] << " | " << nearSolution_G[3] << endl;
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
  prepareJointParamForEachFrames(currentJointConfiguration, nearSolution_A, nearSolution_B, nearSolution_C, nearSolution_G, jointArr_theta1, jointArr_theta2, jointArr_d3, jointArr_theta4);

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
  WriteParamToCsvFile("X_vs_Y.csv", xy_vals);

  // Get the target X and Y values for all (4) via points
  xTarget.push_back(A_positionVar[0]);
  xTarget.push_back(B_positionVar[0]);
  xTarget.push_back(C_positionVar[0]);
  xTarget.push_back(G_positionVar[0]);
  yTarget.push_back(A_positionVar[1]);
  yTarget.push_back(B_positionVar[1]);
  yTarget.push_back(C_positionVar[1]);
  yTarget.push_back(G_positionVar[1]);

  vector<pair<string, vector<double>>> XY_TargetValues = { {"X_targ", xTarget}, {"Y_targ", yTarget} };
  WriteParamToCsvFile("XY_InputParameters.csv", XY_TargetValues);

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
  cout << "\n\n*****************************\n]\nDo you want to move the robot ? [ 1/0 ] : ";
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
  int inc = 1000 * (maximumTime) / lengthOfVector;

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

void WriteParamToCsvFile(string filename, vector<pair<string, vector<double>>> data) {
  ofstream myFile(filename);
  // send column names to the stream
  for (int j = 0; j < data.size(); ++j) {
    myFile << data.at(j).first;
    if (j != data.size() - 1) myFile << ",";
  }
  myFile << "\n";
  // send data to the stream
  for (int i = 0; i < data.at(0).second.size(); ++i)
  {
    for (int j = 0; j < data.size(); ++j)
    {
      myFile << data.at(j).second.at(i);
      if (j != data.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
  }
  // Close the file
  myFile.close();
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

void displayJointVar(vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff) {
  for (int i = 0; i < 4; i++) {

    switch (i)
    {
    case 0:
      cout << "******* Coefficients For 'Theta 1' *******\n";
      break;
    case 1:
      cout << "******* Coefficients For 'Theta 2' *******\n";
      break;
    case 2:
      cout << "******* Coefficients For 'D3' *******\n";
      break;
    case 3:
      cout << "******* Coefficients For 'Theta 4' *******\n";
      break;
    }
    cout << "\tS to A : [" << currJConfig2A_coeff[i][0] << " , " << currJConfig2A_coeff[i][1] << " , " << currJConfig2A_coeff[i][2] << " , " << currJConfig2A_coeff[i][3] << " ]" << endl;
    cout << "\tA to B : [" << A2B_coeff[i][0] << " , " << A2B_coeff[i][1] << " , " << A2B_coeff[i][2] << " , " << A2B_coeff[i][3] << " ]" << endl;
    cout << "\tB to C : [" << B2C_coeff[i][0] << " , " << B2C_coeff[i][1] << " , " << B2C_coeff[i][2] << " , " << B2C_coeff[i][3] << " ]" << endl;
    cout << "\tC to G : [" << C2G_coeff[i][0] << " , " << C2G_coeff[i][1] << " , " << C2G_coeff[i][2] << " , " << C2G_coeff[i][3] << " ]" << endl;
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
    cout << "\tTheta1: " << jointArr_theta1[0] << " | " << jointArr_theta1[1] << " | " << jointArr_theta1[2] << " | " << jointArr_theta1[3] << " | " << jointArr_theta1[4] << endl;
    cout << "\tTheta2: " << jointArr_theta2[0] << " | " << jointArr_theta2[1] << " | " << jointArr_theta2[2] << " | " << jointArr_theta2[3] << " | " << jointArr_theta2[4] << endl;
    cout << "\tD3    : " << jointArr_d3[0] << " | " << jointArr_d3[1] << " | " << jointArr_d3[2] << " | " << jointArr_d3[3] << " | " << jointArr_d3[4] << endl;
    cout << "\tTheta4: " << jointArr_theta4[0] << " | " << jointArr_theta4[1] << " | " << jointArr_theta4[2] << " | " << jointArr_theta4[3] << " | " << jointArr_theta4[4] << endl;
    cout << "*******************************************************************\n\n";
  }

}

// Forward Kinematics #################################################################################################################
void KIN(JOINT& jointVar, transformMatrix& writstRelativeBaseT) {

  double theta1 = jointVar[0];
  double theta2 = jointVar[1];
  double D3 = jointVar[2];
  double theta4 = jointVar[3];

  double phi = theta1 + theta2 - theta4; // Clockwise, Clockwise, Anti-Clockwise

  double cos_phi = cosf(phi);
  double sin_phi = sinf(phi);

  transformMatrix temp = {
    {cos_phi,  sin_phi, 	 0,  D1_BASE_TO_JOINT2_X * cos(theta1) + D2_WRIST_TO_JOINT2_X * cos(theta1 + theta2)},
    {sin_phi, -cos_phi,    0,  D1_BASE_TO_JOINT2_X * sin(theta1) + D2_WRIST_TO_JOINT2_X * sin(theta1 + theta2)},
    {0,         0,        -1,  BASE_TO_JOINT2_Z - (STATION_TO_BASE_Z + D3)},
    {0, 	      0, 		     0,  1}
  };

  copyTransformMatrix(temp, writstRelativeBaseT);
}

void WHERE(JOINT& jointVar, JOINT& sPt) {

  transformMatrix writstRelativeBaseT, wristRelativeStationT, toolRelativeStationT;

  KIN(jointVar, writstRelativeBaseT); // computes the position (x, y, z, phi) of the tool frame with respect to the station get writstRelativeBaseT
  multiplyTwoTransformMatrices(base_Relativeto_Station_T, writstRelativeBaseT, wristRelativeStationT);
  multiplyTwoTransformMatrices(wristRelativeStationT, tool_Relativeto_Wrist_T, toolRelativeStationT);
  transformMatrixToParameters(toolRelativeStationT, sPt);
}

void forwardKinematics(JOINT& jointVariables, JOINT& sPt_toolPostionWRTStation) {

  printf("\n\n****************************************\nForward Kinematics\n****************************************\n\n");

  // All angles are in degrees
  double theta_1, theta_2, d3, theta_4;
  bool isValid = false;


  while (!isValid) {

    getJointParametersFromUser(theta_1, theta_2, d3, theta_4); // Get user input
    convertJointPramaterAnglesToRadian(theta_1, theta_2, d3, theta_4, jointVariables); // Convert Degrees to radians
    isValid = checkIfJointsAreWithinConstraints(jointVariables); // Check if prameters are within constraints
    if (!isValid) {
      cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\nInvalid inputs!please try again.\n\n\n";
    }
  }

  JOINT inputConfig{ RAD2DEG(jointVariables[0]), RAD2DEG(jointVariables[1]), jointVariables[2], RAD2DEG(jointVariables[3]) };
  printf("\n\tInput Joint Parameters: [ (%f , %f , %f (mm), %f) ]\n\n", RAD2DEG(jointVariables[0]), RAD2DEG(jointVariables[1]), jointVariables[2], RAD2DEG(jointVariables[3]));
  MoveToConfiguration(inputConfig, true);

  // report position and orientation of the tool (x, y, z, phi)
  WHERE(jointVariables, sPt_toolPostionWRTStation);
  printf("\tCorresponding pose (position and orientation) of the Tool Frame  i.e. [X , Y , Z , Theta]: ");
  printf("[%f, %f, %f, %f]\n", sPt_toolPostionWRTStation[0], sPt_toolPostionWRTStation[1], sPt_toolPostionWRTStation[2], RAD2DEG(sPt_toolPostionWRTStation[3]));
  printf("\n\n");
  cout << "***** Would You Like to Followup Inverse Kinematics? [Y/N]:   ";
  uint8_t option;

  cin >> option;

  switch (option)
  {
  case 'Y':
    inverseKinematics(sPt_toolPostionWRTStation, true, false);
    break;
  case 'N':
    break;
  default:
    break;
  }

  return;
}
//#####################################################################################################################################

// Inverse Kinematics #################################################################################################################
bool inverseKinematics(JOINT& sPt_toolPostionWRTStation, uint8_t isItAFollowUp, uint8_t isItPickAndPlace) {

  printf("\n\n****************************************\nInverse Kinematics\n****************************************\n\n");

  // All angles are in degrees
  double x, y, z, phi;
  bool isValid = false;


  if (!isItAFollowUp)
  {
    char tempString[] = "**Please, Enter Tool Position With Respect To Base[X, Y, Z, phi] **";
    getToolPositionFromUser(x, y, z, phi, tempString);
    printf("Entered Inputs: [%f , %f , %f , %f (Deg)]\n", x, y, z, phi);
  }
  else
  {
    x = sPt_toolPostionWRTStation[0];
    y = sPt_toolPostionWRTStation[1];
    z = sPt_toolPostionWRTStation[2];
    if (isItPickAndPlace) {
      phi = sPt_toolPostionWRTStation[3];
      printf("Inputs for Pick And Place: [%f , %f , %f , %f (Deg)]\n", x, y, z, phi);
    }
    else
    {
      phi = RAD2DEG(sPt_toolPostionWRTStation[3]);
      printf("Calculated Inputs for Inverse Kinematics: [%f , %f , %f , %f (Deg)]\n", x, y, z, phi);
    }
  }

  JOINT toolPosition{ x, y, z, DEG2RAD(phi) };
  JOINT currentJointConfig, firstSolution, secondSolution;
  uint8_t status = GetConfiguration(currentJointConfig);
  /*if (status == 1) {
    continue;
  }*/

  currentJointConfig[0] = DEG2RAD(currentJointConfig[0]);
  currentJointConfig[1] = DEG2RAD(currentJointConfig[1]);
  currentJointConfig[3] = DEG2RAD(currentJointConfig[3]);


  bool firstFlag, secondFlag;
  uint8_t status1 = getSolutionsForInverseKIN(toolPosition, currentJointConfig, firstSolution, secondSolution, firstFlag, secondFlag);
  if (!status1) {
    return 0;
  }
  if (!firstFlag && !secondFlag)
  {
    printf("ERROR: (%f, %f, %f, %f)\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));
    printf("ERROR: (%f, %f, %f, %f)\n\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));
    cout << "\n\n$$$$$$$$$$$$$$$$$$$$$$$$\n\nNo Solutions\n\n";
    return 0;
  }

  else if (firstFlag && !secondFlag)
  {
    cout << "First Valid Solution:\n";
    printf("[1st] (%f, %f, %f, %f)\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));

    cout << "\n\n$$$$$$$$$$$$$$$$$$$$$$$$\n\nSecond Solution Invalid\n";
    printf("[2nd] ERROR: (%f, %f, %f, %f)\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));

  }
  else if (!firstFlag && secondFlag)
  {
    cout << "\n\n$$$$$$$$$$$$$$$$$$$$$$$$\n\nFirst Solution Invalid\n";
    printf("[1st] ERROR: (%f, %f, %f, %f)\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));

    cout << "Second Valid Solution:\n";
    printf("[2nd] (%f, %f, %f, %f)\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));
  }
  else
  {
    cout << "First Valid Solution:\n";
    printf("[1st] (%f, %f, %f, %f)\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));

    cout << "Second Valid Solution:\n";
    printf("[2nd] (%f, %f, %f, %f)\n\n\n\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));
  }

  // Move TO Closest Solution
  firstSolution[0] = RAD2DEG(firstSolution[0]);
  firstSolution[1] = RAD2DEG(firstSolution[1]);
  firstSolution[3] = RAD2DEG(firstSolution[3]);
  MoveToConfiguration(firstSolution, true);
  return 1;
}

bool calculateAllTwoSolutions(transformMatrix& bTw, JOINT& closestSolution, JOINT& farthestSolution, bool& firstFlag, bool& secondFlag)
{
  double x = bTw[0][3];
  double y = bTw[1][3];
  double z = bTw[2][3];
  double cPhi = bTw[0][0];
  double sPhi = bTw[0][1];

  bool firstInvalid = false;
  bool secondInvalid = false;

  double c2 = (pow(x, 2) + pow(y, 2) - pow(D2_WRIST_TO_JOINT2_X, 2) - pow(D1_BASE_TO_JOINT2_X, 2)) / (2 * D2_WRIST_TO_JOINT2_X * D1_BASE_TO_JOINT2_X);
  double s2 = sqrt(1 - pow(c2, 2));
  double temp111 = sqrt(pow(x, 2) + pow(y, 2));
  double temp123 = D1_BASE_TO_JOINT2_X + D2_WRIST_TO_JOINT2_X;

  if (!TRAJECTORY_PLANNING) {

  }

  if (abs(temp111 - temp123) < 2)//Checks if there is 1 solution  
  {
    double D3;
    D3 = BASE_TO_JOINT2_Z - STATION_TO_BASE_Z - z;
    double Theta2 = 0;
    double Theta1 = atan2(y / (D2_WRIST_TO_JOINT2_X + D1_BASE_TO_JOINT2_X), x / (D2_WRIST_TO_JOINT2_X + D1_BASE_TO_JOINT2_X));
    double Phi = atan2(sPhi, cPhi);
    double Theta4 = Phi - Theta1 - Theta2;
    printf("\n\n*** There is 1 solution and it is: (%f deg, %f deg, %f mm, %f deg) \n", Theta1 * 180 / PI, Theta2 * 180 / PI, D3, Theta4 * 180 / PI);
    firstFlag = true;
    secondFlag = true;
    closestSolution[0] = Theta1;
    closestSolution[1] = Theta2;
    closestSolution[2] = D3;
    closestSolution[3] = Theta4;
    return 0;
  }


  double Theta2_1 = atan2(s2, c2);
  double Theta2_2 = atan2(-s2, c2);

  double k1_1 = D1_BASE_TO_JOINT2_X + double(D2_WRIST_TO_JOINT2_X * double(cosf(Theta2_1)));
  double k2_1 = double(D2_WRIST_TO_JOINT2_X * double(sinf(Theta2_1)));

  double k1_2 = D1_BASE_TO_JOINT2_X + double(D2_WRIST_TO_JOINT2_X * double(cosf(Theta2_2)));
  double k2_2 = double(D2_WRIST_TO_JOINT2_X * double(sinf(Theta2_2)));

  double temp_1 = atan2(y, x) - atan2(k2_1, k1_1);
  double theta1_1 = temp_1;

  double temp2_1 = abs(abs(temp_1) - DEG2RAD(360));

  if (abs(temp_1) > DEG2RAD(THETA_1_DEG_CON))
  {
    if (temp2_1 < DEG2RAD(ANGLE_210) && temp2_1 > DEG2RAD(ANGLE_150))
    {
      firstInvalid = true;
    }
    else
    {
      theta1_1 = (temp_1 / abs(temp_1)) * (abs(temp_1) - DEG2RAD(360));
    }
  }

  double temp_2 = atan2(y, x) - atan2(k2_2, k1_2);
  double theta1_2 = temp_2;
  double temp2_2 = abs(abs(temp_2) - DEG2RAD(360));

  if (abs(temp_2) > DEG2RAD(THETA_1_DEG_CON))
  {
    if (temp2_2 < DEG2RAD(ANGLE_210) && temp2_2 > DEG2RAD(ANGLE_150))
    {
      secondInvalid = true;
    }
    else
    {
      theta1_2 = (temp_2 / abs(temp_2)) * (abs(temp_2) - DEG2RAD(360));
    }
  }
  double phi = atan2(sPhi, cPhi);
  double alpha41_p = theta1_1 + Theta2_1 - phi;
  double theta4_1 = alpha41_p;
  double alpha42_p = abs(abs(alpha41_p) - DEG2RAD(360));
  if (abs(alpha41_p) > DEG2RAD(THETA_4_DEG_CON))
  {
    if (alpha42_p < DEG2RAD(ANGLE_210 - 0.0001) && alpha42_p > DEG2RAD(ANGLE_150 + 0.0001))
    {
      firstInvalid = true;
    }
    else
    {
      theta4_1 = (alpha41_p / abs(alpha41_p)) * (abs(alpha41_p) - DEG2RAD(360));
      if (abs(theta4_1) > DEG2RAD(180))
      {
        theta4_1 = (theta4_1 / abs(theta4_1)) * (theta4_1 - DEG2RAD(360));
      }
    }
  }

  double alpha41_n = theta1_2 + Theta2_2 - phi;
  double theta4_2 = alpha41_n;
  double alpha42_n = abs(abs(alpha41_n) - DEG2RAD(360));
  if (abs(alpha41_n) > DEG2RAD(THETA_4_DEG_CON))
  {
    if (alpha42_n < DEG2RAD(ANGLE_210 - 0.0001) && alpha42_n > DEG2RAD(ANGLE_150 + 0.0001))
    {
      firstInvalid = true;
    }
    else
    {
      theta4_2 = (alpha41_n / abs(alpha41_n)) * (abs(alpha41_n) - DEG2RAD(360));
      if (abs(theta4_2) > DEG2RAD(180))
      {
        theta4_2 = (theta4_2 / abs(theta4_2)) * (theta4_2 - DEG2RAD(360));
      }
    }
  }

  double d3 = -z - STATION_TO_BASE_Z + BASE_TO_JOINT2_Z;

  JOINT firstSol{ theta1_1, Theta2_1, d3, theta4_1 };
  JOINT secondSol{ theta1_2, Theta2_2, d3, theta4_2 };

  // check joint values
  firstFlag = checkIfJointsAreWithinConstraints(firstSol);
  secondFlag = checkIfJointsAreWithinConstraints(secondSol);

  copyArray(firstSol, closestSolution);
  copyArray(secondSol, farthestSolution);
  return 1;
}

bool getSolutionsForInverseKIN(JOINT& toolPosition, JOINT& currentJointConfig, JOINT& firstSol, JOINT& secondSol, bool& flagFirst, bool& flagSecond) {

  transformMatrix wrels, bTw, sTt, bTt;
  transformMatrix bTs, tTw;

  parametersToTransformMatrixForInV(toolPosition, sTt);
  invertTransformMatrix(base_Relativeto_Station_T, bTs);
  invertTransformMatrix(tool_Relativeto_Wrist_T, tTw);
  multiplyTwoTransformMatrices(bTs, sTt, bTt);
  multiplyTwoTransformMatrices(bTt, tTw, bTw);
  bool status = calculateAllTwoSolutions(bTw, firstSol, secondSol, flagFirst, flagSecond);
  if (!status) {
    return 0;
  }
  JOINT temp, itr;

  if (flagFirst && flagSecond)
  {
    float sums[2] = { 0, 0 };
    copyArray(firstSol, itr);
    float w[4] = { 1, 1, 1, 1 };

    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        sums[i] += w[j] * (abs(itr[j] - currentJointConfig[j]));
      }
      copyArray(secondSol, itr);
    }
    if (sums[1] < sums[0])
    {
      copyArray(firstSol, temp);
      copyArray(secondSol, firstSol);
      copyArray(temp, secondSol);
    }
  }
  return 1;
}
//#####################################################################################################################################

// Pick And Place #####################################################################
void runPickAndPlace() {
  printf("\n\n****************************************\nRunning Pick and Place\n****************************************\n\n");

  cout << "Please Enter the first target's parameters [X,Y,Z,PHI]\n";
  double x_1, y_1, z_1, phi_1;
  char tempString[] = "**[First Target] Please, Enter Tool Position With Respect To Base[X, Y, Z, phi] **";
  getToolPositionFromUser(x_1, y_1, z_1, phi_1, tempString);

  strcpy_s(tempString, "**[Second Target] Please, Enter Tool Position With Respect To Base[X, Y, Z, phi] **");
  cout << "Please Enter the Second target's parameters [X,Y,Z,PHI]\n";
  double x_2, y_2, z_2, phi_2;
  getToolPositionFromUser(x_2, y_2, z_2, phi_2, tempString);

  JOINT firstLocation{ x_1, y_1, z_1, phi_1 };
  uint8_t status = inverseKinematics(firstLocation, true, true);

  if (status) {

    cout << "\nClosing Gripper to Pick the Object \n";
    toggleGripper(gripStatus);
    cout << "\nNow Moving to second location\n";
    JOINT secondLocation{ x_2, y_2, z_2, phi_2 };

    status = inverseKinematics(secondLocation, true, true);

    if (status) {
      cout << "\nToggling Gripper to Place the Object \n";
      toggleGripper(gripStatus);
    }
    else
    {
      cout << "\n\nNO SOLUTIONS\n\n";
    }

  }
  else
  {
    cout << "\n\nNO SOLUTIONS\n\n";
  }


  return;
}


void invertTransformMatrix(transformMatrix& aTb, transformMatrix& bTa)
{
  rotationMatrix aRb;
  extractRotationMatrix(aTb, aRb); // b relative a (Rotation)

  arrayOf3 aPb;
  extractPositionFromTransformMatrix(aTb, aPb); // b Relative a (Position)

  rotationMatrix bRa;
  getTransposedMatrix(aRb, bRa); // bra // Get Transposed Rotation Matrix (A relative B) from (B relative A)

  arrayOf3 negative_aPb, bPa;
  multiplyPositionArrayWithValue(aPb, -1, negative_aPb);				// Output = -aPb
  rotationMatrixMultiplication(bRa, negative_aPb, bPa); //Calcluate bPa = bRa * -aPb 

  // Creating The Transform Matrix from bPa , and bTa
  createTransformMatrixFrom_R_and_P(bRa, bPa, bTa);
}

void multiplyPositionArrayWithValue(arrayOf3& inputPositionArray, double value, arrayOf3& outputPositionArray)
{
  for (int i = 0; i < 3; i++)
  {
    outputPositionArray[i] = value * inputPositionArray[i];
  }
}

void getTransposedMatrix(rotationMatrix& a_R_b, rotationMatrix& b_R_a)
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      b_R_a[i][j] = a_R_b[j][i];
    }
  }
}

void extractPositionFromTransformMatrix(transformMatrix& transformMatrix, arrayOf3& excractedPositionVector)
{
  for (int i = 0; i < 3; i++)
  {
    excractedPositionVector[i] = transformMatrix[i][3]; // Extract the Position vector from the Transform matrix (X,Y,Z)
  }
}

void getToolPositionFromUser(double& x, double& y, double& z, double& phi, char* instructionString) {
  // ** Please, Enter Tool Position With Respect To Base [X,Y,Z,phi]**
  cout << "\n" << instructionString << endl;
  printf("\tEnter X in MM :  ");
  cin >> x;
  printf("\tEnter Y in MM :  ");
  cin >> y;
  printf("\tEnter Z in MM :  ");
  cin >> z;
  printf("\tEnter Phi in Degree :  ");
  cin >> phi;
}

// Helping Hands

void seprateJointParametersPerIndex(arrayOf5& outputJointParam, JOINT& currentJointParam, JOINT& jointAParam, JOINT& jointBParam, JOINT& jointCParam, JOINT& jointGParam, int index) {

  /*
  if index == 0
    outputJointParam = CurrentConfig[0], A[0], B[0], C[0], G[0]
  if index == 1
    outputJointParam = CurrentConfig[1], A[1], B[1], C[1], G[1]
  .
  .

  */
  outputJointParam[0] = currentJointParam[index];
  outputJointParam[1] = jointAParam[index];
  outputJointParam[2] = jointBParam[index];
  outputJointParam[3] = jointCParam[index];
  outputJointParam[4] = jointGParam[index];
}

bool checkIfJointsAreWithinConstraints(JOINT& joinVar) {

  double x1 = joinVar[0];
  double x2 = joinVar[1];
  double d3 = joinVar[2];
  double x4 = joinVar[3];


  bool isTheta1_valid = (x1 <= THETA_1_RADIAN_CON && x1 >= -THETA_1_RADIAN_CON);
  bool isTheta2_valid = (x2 <= THETA_2_RADIAN_CON && x2 >= -THETA_2_RADIAN_CON);
  bool isD3_valid = (d3 <= D3_MAX_BOUND && d3 >= D3_MIN_BOUND);
  bool isTheta4_valid = (x4 <= THETA_4_RADIAN_CON && x4 >= -THETA_4_RADIAN_CON);

  return isTheta1_valid && isTheta2_valid && isD3_valid && isTheta4_valid;
}

void copyArray(JOINT& inputArray, JOINT& outputArray) {

  for (int i = 0; i < 4; i++) {
    outputArray[i] = inputArray[i];
  }

}

void getJointParametersFromUser(double& theta_1, double& theta_2, double& d3, double& theta_4) {

  cout << "** Please, Enter Joint Parameters **\n";
  printf("\tEnter Theta 1 in Degree. Range [-%d, %d]:  ", THETA_1_DEG_CON, THETA_1_DEG_CON);
  cin >> theta_1;
  printf("\tEnter Theta 2 in Degree. Range [-%d, %d]:  ", THETA_2_DEG_CON, THETA_2_DEG_CON);
  cin >> theta_2;
  printf("\tEnter D3 in mm. Range [%d, %d]:  ", D3_MIN_BOUND, D3_MAX_BOUND);
  cin >> d3;
  printf("\tEnter Theta 4 in Degree [-%d, %d]:  ", THETA_4_DEG_CON, THETA_4_DEG_CON);
  cin >> theta_4;

}

void convertJointPramaterAnglesToRadian(double& theta_1, double& theta_2, double d3, double& theta_4, JOINT& jointVariables)
{
  double theta_1_radian, theta_2_radian, theta_4_radian;
  theta_1_radian = DEG2RAD(theta_1);
  theta_2_radian = DEG2RAD(theta_2);
  theta_4_radian = DEG2RAD(theta_4);
  JOINT temp{ theta_1_radian, theta_2_radian, d3, theta_4_radian };
  copyArray(temp, jointVariables);
}

void parametersToTransformMatrix(JOINT& inputParameters, transformMatrix& outputTransformMatrix)
{
  /*******************************************
    Assume all rotations are about Z axis
    Input:

    Input Parameters = (x, y, z, phi)

    Output:

    T = | cos(phi)    -sin(phi)     0   x |
        | sin(phi)    cos(phi)      0   y |
        | 0             0           1   z |
        | 0             0           0   1 |

  *******************************************/

  // Get Position
  double x = inputParameters[0];
  double y = inputParameters[1];
  double z = inputParameters[2];

  // Get Angle
  double theta = inputParameters[3];

  // calculate parameters for transformation matrix, T
  double sineTheta = sin(theta);
  double costTheta = cos(theta);

  transformMatrix temp{
    {costTheta, -sineTheta,  0, x},
    {sineTheta,  costTheta,  0, y},
    {0,             0,       1, z},
    {0,             0,       0, 1}
  };

  copyTransformMatrix(temp, outputTransformMatrix);
  return;
}

void parametersToTransformMatrixForInV(JOINT& inputParameters, transformMatrix& outputTransformMatrix)
{
  double z = inputParameters[2];
  double phi = inputParameters[3];
  double y = inputParameters[1];
  double x = inputParameters[0];
  double cPhi = cos(phi);
  double sPhi = sin(phi);

  transformMatrix temp{

        {cPhi,  sPhi,   0,  x},
        {sPhi, -cPhi,   0,  y},
        { 0,     0,    -1,  z},
        { 0,     0,     0,  1}

  };

  copyTransformMatrix(temp, outputTransformMatrix);

  return;
}

void transformMatrixToParameters(transformMatrix& inputTransformMatrix, JOINT& outPutParameters) {
  /*******************************************
    Assume all rotations are about Z axis
    ##########################################
    Input:

    T = | cos(phi)    -sin(phi)     0   x |
        | sin(phi)    cos(phi)      0   y |
        | 0             0           1   z |
        | 0             0           0   1 |

    ##########################################
    Output:

    Output Parameters = (x, y, z, phi)

  *******************************************/

  float x = inputTransformMatrix[0][3];
  float y = inputTransformMatrix[1][3];
  float z = inputTransformMatrix[2][3];
  float theta = atan2(inputTransformMatrix[1][0], inputTransformMatrix[0][0]);

  JOINT temp{ x, y, z, theta };
  copyArray(temp, outPutParameters);
}

void extractRotationMatrix(transformMatrix& transformMatrix, rotationMatrix& rotationMatrix) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      rotationMatrix[i][j] = transformMatrix[i][j];
    }
  }
}

void rotationMatrixMultiplication(rotationMatrix& firstMatrix, rotationMatrix& secondMatrix, rotationMatrix& outputMatrix)
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      double total = 0;
      for (int k = 0; k < 3; k++)
      {
        total += firstMatrix[i][k] * secondMatrix[k][j];
      }
      outputMatrix[i][j] = total;
    }
  }
}

void rotationMatrixMultiplication(rotationMatrix& rotationMatrix, arrayOf3& positionVector, arrayOf3& result)
{
  for (int i = 0; i < 3; i++)
  {
    double sum = 0;
    for (int j = 0; j < 3; j++)
    {
      sum += rotationMatrix[i][j] * positionVector[j];
    }
    result[i] = sum;
  }
}

void getPositionVectorFromTransformMatrix(transformMatrix& tmat, arrayOf3& pos)
{
  // gets the Position vector from the Transformation matrix
  for (int i = 0; i < (4 - 1); i++)
  {
    pos[i] = tmat[i][(4 - 1)];
  }
}

void sumPositionVectors(arrayOf3& pos1, arrayOf3& pos2, arrayOf3& res)
{
  // adds pos1 and pos2 togheter
  for (int i = 0; i < (4 - 1); i++)
  {
    res[i] = pos1[i] + pos2[i];
  }
}

void createTransformMatrixFrom_R_and_P(rotationMatrix& rotationMatrix, arrayOf3& pos, transformMatrix& tMatrix)
{
  transformMatrix temp = {
    {rotationMatrix[0][0], rotationMatrix[0][1], rotationMatrix[0][2], pos[0]},
    {rotationMatrix[1][0], rotationMatrix[1][1], rotationMatrix[1][2], pos[1]},
    {rotationMatrix[2][0], rotationMatrix[2][1], rotationMatrix[2][2], pos[2]},
    {0,                            0,                    0,              1   }
  };
  copyTransformMatrix(temp, tMatrix);
}

void multiplyTwoTransformMatrices(transformMatrix& aTb, transformMatrix& bTc, transformMatrix& aTc) {
  /*
      Input: bTa, bTc

      Output: aTc = bTa * bTc

   */
  arrayOf3 bPa, cPb, cPa;
  getPositionVectorFromTransformMatrix(aTb, bPa);
  getPositionVectorFromTransformMatrix(bTc, cPb);

  rotationMatrix bTa, cTb, cTa;
  extractRotationMatrix(bTc, cTb);
  extractRotationMatrix(aTb, bTa);

  rotationMatrixMultiplication(bTa, cTb, cTa); //aTc = aTb * bTc

  arrayOf3 temp;
  rotationMatrixMultiplication(bTa, cPb, temp);               //temp = bTa * cPb
  sumPositionVectors(temp, bPa, cPa);  //cPa = temp + bPa

  createTransformMatrixFrom_R_and_P(cTa, cPa, aTc);  // Rotation Matrix and Position Vector : aTc (C relative A)
}

void copyTransformMatrix(transformMatrix& inputTMatrix, transformMatrix& outputTMatrix) {

  /*
  Input  : Input Matrix
  Output : Output Matrix
  */

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      outputTMatrix[i][j] = inputTMatrix[i][j];
    }
  }

}

void toggleGripper(bool& negativeCurrentGripperStatus)
{
  // if status False (0) : Gripper Open (1) 
  // if status True  (1) : Gripper Close (0)
  if (negativeCurrentGripperStatus == true)
  {
    cout << "Opening Gripper\n";
    negativeCurrentGripperStatus = false;
  }
  else
  {
    cout << "Closing Gripper\n";
    negativeCurrentGripperStatus = true;
  }
  Grasp(negativeCurrentGripperStatus);
}