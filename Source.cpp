#include "ensc894.h"
#include "ensc-488.h"
#include "StdAfx.h"

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



transformMatrix base_Relativeto_Station_T, tool_Relativeto_Wrist_T;


int main(void) {
  // runs UI

  /* Menu Options:
  1. Forward Kinematics
  2. Inverse Kinematics
  3. Pick and Place
  3. Stop and Reset Robot
  4. Close Gripper
  0. Exit
  */

  int inputOption;
  bool gripStatus = false;
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
    cout << "\t5. Toggle Gripper\n";
    cout << "\t0. Exit\n" << endl;
    cout << "****************************************\n\n";
    printf("Please Choose an option from above: ");

    cin >> inputOption;

    switch (inputOption) {

    case 1:
      forwardKinematics(jointParameters, sPt);
      break;
    case 2:
      inverseKinematics(jointParameters, sPt, false);
      break;
    case 3:
      inverseKinematics(jointParameters, sPt, false);
      break;
    case 4:
      printf("\n\nStopping and Resetting Robot\n\n");
      StopRobot();
      ResetRobot();
      break;
    case 5:
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

// Forward Kinematics
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
  printf("Input Joint Parameters: [ (%f , %f , %f (mm), %f) ]\n\n", RAD2DEG(jointVariables[0]), RAD2DEG(jointVariables[1]), jointVariables[2], RAD2DEG(jointVariables[3]));
  MoveToConfiguration(inputConfig, true);

  // report position and orientation of the tool (x, y, z, phi)
  WHERE(jointVariables, sPt_toolPostionWRTStation);
  printf("Corresponding pose (position and orientation) of the Tool Frame  i.e. [X , Y , Z , Theta]: ");
  printf("[%f, %f, %f, %f]\n", sPt_toolPostionWRTStation[0], sPt_toolPostionWRTStation[1], sPt_toolPostionWRTStation[2], RAD2DEG(sPt_toolPostionWRTStation[3]));
  printf("\n\n");
  cout << "Would You Like to Followup Inverse Kinematics? [Y/N]:   ";
  uint8_t option;

  cin >> option;

  switch (option)
  {
  case 'Y':
    inverseKinematics(inputConfig, sPt_toolPostionWRTStation, true);
    break;
  case 'N':
    break;
  default:
    break;
  }

  return;
}

//Inverse Kinematics
void inverseKinematics(JOINT& jointVariables, JOINT& sPt_toolPostionWRTStation, uint8_t isItAFollowUp) {

  printf("\n\n****************************************\nForward Kinematics\n****************************************\n\n");

  // All angles are in degrees
  double x, y, z, phi;
  bool isValid = false;
  if (!isItAFollowUp)
  {
    getToolPositionFromUser(x, y, z, phi);
    printf("Entered Inputs: [%f , %f , %f , %f (Deg)]\n", x, y, z, phi);
  }
  else
  {
    x = sPt_toolPostionWRTStation[0];
    y = sPt_toolPostionWRTStation[1];
    z = sPt_toolPostionWRTStation[2];
    phi = RAD2DEG(sPt_toolPostionWRTStation[3]);
    printf("Calculated Inputs for Inverse Kinematics: [%f , %f , %f , %f (Deg)]\n", x, y, z, phi);
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
  getSolutionsForInverseKIN(toolPosition, currentJointConfig, firstSolution, secondSolution, firstFlag, secondFlag);
  if (!firstFlag && !secondFlag)
  {
    printf("ERROR: (%f, %f, %f, %f)\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));
    printf("ERROR: (%f, %f, %f, %f)\n\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));
    cout << "\n\n$$$$$$$$$$$$$$$$$$$$$$$$\n\nNo Solutions\n\n";
    return;
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
  MoveToConfiguration(firstSolution);
  return;
}

// Inverse Kinematics
void calculateAllTwoSolutions(transformMatrix& bTw, JOINT& closestSolution, JOINT& farthestSolution, bool& firstFlag, bool& secondFlag)
{
  double x = bTw[0][3];
  double y = bTw[1][3];
  double z = bTw[2][3];
  double cPhi = bTw[0][0];
  double sPhi = bTw[0][1];

  bool firstInvalid = false;
  bool secondInvalid = false;

  double cTheta2 = (pow(x, 2) + pow(y, 2) - pow(D2_WRIST_TO_JOINT2_X, 2) - pow(D1_BASE_TO_JOINT2_X, 2)) / (2 * D2_WRIST_TO_JOINT2_X * D1_BASE_TO_JOINT2_X);
  double sTheta2 = sqrt(1 - pow(cTheta2, 2));

  double Theta2_1 = atan2(sTheta2, cTheta2);
  double Theta2_2 = atan2(-sTheta2, cTheta2);

  double k1_p = D1_BASE_TO_JOINT2_X + double(D2_WRIST_TO_JOINT2_X * double(cosf(Theta2_1)));
  double k2_p = double(D2_WRIST_TO_JOINT2_X * double(sinf(Theta2_1)));

  double k1_n = D1_BASE_TO_JOINT2_X + double(D2_WRIST_TO_JOINT2_X * double(cosf(Theta2_2)));
  double k2_n = double(D2_WRIST_TO_JOINT2_X * double(sinf(Theta2_2)));

  double alpha11_p = atan2(y, x) - atan2(k2_p, k1_p);
  double theta1_1 = alpha11_p;

  double alpha12_p = abs(abs(alpha11_p) - DEG2RAD(360));

  if (abs(alpha11_p) > DEG2RAD(THETA_1_DEG_CON))
  {
    if (alpha12_p < DEG2RAD(A210) && alpha12_p > DEG2RAD(A150))
    {
      firstInvalid = true;
    }
    else
    {
      theta1_1 = (alpha11_p / abs(alpha11_p)) * (abs(alpha11_p) - DEG2RAD(360));
    }
  }

  double alpha11_n = atan2(y, x) - atan2(k2_n, k1_n);
  double theta1_2 = alpha11_n;
  double alpha12_n = abs(abs(alpha11_n) - DEG2RAD(360));

  if (abs(alpha11_n) > DEG2RAD(THETA_1_DEG_CON))
  {
    if (alpha12_n < DEG2RAD(A210) && alpha12_n > DEG2RAD(A150))
    {
      secondInvalid = true;
    }
    else
    {
      theta1_2 = (alpha11_n / abs(alpha11_n)) * (abs(alpha11_n) - DEG2RAD(360));
    }
  }
  double phi = atan2(sPhi, cPhi);
  double alpha41_p = theta1_1 + Theta2_1 - phi;
  double theta4_1 = alpha41_p;
  double alpha42_p = abs(abs(alpha41_p) - DEG2RAD(360));
  if (abs(alpha41_p) > DEG2RAD(THETA_4_DEG_CON))
  {
    if (alpha42_p < DEG2RAD(A210 - 0.0001) && alpha42_p > DEG2RAD(A150 + 0.0001))
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
    if (alpha42_n < DEG2RAD(A210 - 0.0001) && alpha42_n > DEG2RAD(A150 + 0.0001))
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
}

void getSolutionsForInverseKIN(JOINT& toolPosition, JOINT& currentJointConfig, JOINT& firstSol, JOINT& secondSol, bool& flagFirst, bool& flagSecond) {

  transformMatrix wrels, bTw, sTt, bTt;
  transformMatrix bTs, tTw;

  parametersToTransformMatrixForInV(toolPosition, sTt);
  invertTransformMatrix(base_Relativeto_Station_T, bTs);
  invertTransformMatrix(tool_Relativeto_Wrist_T, tTw);
  multiplyTwoTransformMatrices(bTs, sTt, bTt);
  multiplyTwoTransformMatrices(bTt, tTw, bTw);
  calculateAllTwoSolutions(bTw, firstSol, secondSol, flagFirst, flagSecond);

  if (flagFirst && flagSecond)
  {
    float sums[2] = { 0, 0 };
    // temp used for swapping near and far, itr used to iterate over near and far values when computing sums
    JOINT temp, itr;
    copyArray(firstSol, itr);
    // weights for each joint
    float w[4] = { 1, 1, 1, 1 };
    int M = size(sums);

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        sums[i] += w[j] * (abs(itr[j] - currentJointConfig[j]));
      }
      copyArray(secondSol, itr);
    }
    // swap near and far if it is the closer joint
    if (sums[1] < sums[0])
    {
      copyArray(firstSol, temp);
      copyArray(secondSol, firstSol);
      copyArray(temp, secondSol);
    }
  }
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

void getToolPositionFromUser(double& x, double& y, double& z, double& phi) {
  cout << "** Please, Enter Tool Position With Respect To Base [X,Y,Z,phi]**\n";
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