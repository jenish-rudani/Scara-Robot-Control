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


//SCARA Constraints

#define THETA_1_DEG_CON 150
#define THETA_2_DEG_CON 100
#define THETA_4_DEG_CON 160
#define D3_MIN_BOUND -200
#define D3_MAX_BOUND -100

#define THETA_1_RADIAN_CON DEG2RAD(THETA_1_DEG_CON)
#define THETA_2_RADIAN_CON DEG2RAD(THETA_2_DEG_CON)
#define THETA_4_RADIAN_CON DEG2RAD(THETA_4_DEG_CON)

transformMatrix baseRelativeStationT, toolRelativeWristT;


int main(void) {
  // runs UI

  /* Menu Options:
  1. Forward Kinematics
  2. Inverse Kinematics
  3. Stop and Reset Robot
  4. Close Gripper
  0. Exit
  */

  int inputOption;
  bool gripStatus = false;
  JOINT jointParameters, spt;
  JOINT T{ 0, 0, 135, 0 };
  JOINT B{ 0, 0, 405, 0 };

  parametersToTransformMatrix(B, baseRelativeStationT);
  parametersToTransformMatrix(T, toolRelativeWristT);

  while (true) {

    cout << "\n\n****************************************\n\n";
    cout << "Please choose a number [1,2,3,4] :\n\n";
    cout << "\t1. Forward Kinematics\n";
    cout << "\t2. Inverse Kinematics\n";
    cout << "\t3. Stop+Reset Robot\n";
    cout << "\t4. Toggle Gripper\n";
    cout << "\t0. Exit\n" << endl;
    cout << "****************************************\n\n";
    printf("Please Choose an option from above: ");

    cin >> inputOption;

    switch (inputOption) {

    case 1:
      forwardKinematics(jointParameters, spt);
      break;
    case 2:
      //InvKin(spt);
      break;
    case 3:
      printf("Stopping and Resetting Robot\n");
      StopRobot();
      ResetRobot();
      break;
    case 4:
      toggleGripper(gripStatus);
      break;
    case 0: // Exit
      return 0;
      break;
    default:
      printf("Invalid input. Please try again.\n\n\n");
      break;
    }
  }

  return 0;
}

// Forward Kinematics
void KIN(JOINT& jointVar, transformMatrix& writstRelativeBaseT) {

  double theta1 = jointVar[0];
  double theta2 = jointVar[1];
  double d3 = jointVar[2];
  double theta4 = jointVar[3];
  double phi = theta1 + theta2 - theta4; // Clockwise, Clockwise, Anti-Clockwise

  double cos_phi = cosf(phi);
  double sin_phi = sinf(phi);

  transformMatrix temp = {
    {cos_phi,  sin_phi, 	 0,  D2 * cos(theta1 + theta2) + D1 * cos(theta1)},
    {sin_phi, -cos_phi,    0,  D2 * sin(theta1 + theta2) + D1 * sin(theta1)},
    {0,         0,        -1,  L70 - (L410 + d3)},
    {0, 	      0, 		     0,  1}
  };

  copyTransformMatrix(temp, writstRelativeBaseT);
}

void WHERE(JOINT& jointVar, JOINT& sPt) {

  transformMatrix writstRelativeBaseT, wristRelativeStationT, toolRelativeStationT;

  KIN(jointVar, writstRelativeBaseT); // computes the position (x, y, z, phi) of the tool frame with respect to the station get writstRelativeBaseT
  multiplyTwoMatrices(baseRelativeStationT, writstRelativeBaseT, wristRelativeStationT);
  multiplyTwoMatrices(wristRelativeStationT, toolRelativeWristT, toolRelativeStationT);
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
  MoveToConfiguration(inputConfig, true);
  printf("Input Joint Parameters: [ (%f , %f , %f (mm), %f) ]\n\n", RAD2DEG(jointVariables[0]), RAD2DEG(jointVariables[1]), jointVariables[2], RAD2DEG(jointVariables[3]));

  // report position and orientation of the tool (x, y, z, phi)
  WHERE(jointVariables, sPt_toolPostionWRTStation);
  printf("Corresponding pose (position and orientation) of the Tool Frame  i.e. [X , Y , Z , Theta]: ");
  printf("[%f, %f, %f, %f]\n", sPt_toolPostionWRTStation[0], sPt_toolPostionWRTStation[1], sPt_toolPostionWRTStation[2], RAD2DEG(sPt_toolPostionWRTStation[3]));
  printf("\n\n");

  return;
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
  for (int i = 0; i < 4 - 1; i++)
  {
    double sum = 0;
    for (int j = 0; j < 4 - 1; j++)
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

void createTransformMatrixFromRandP(rotationMatrix& rotationMatrix, arrayOf3& pos, transformMatrix& tMatrix)
{
  transformMatrix temp = {
    {rotationMatrix[0][0], rotationMatrix[0][1], rotationMatrix[0][2], pos[0]},
    {rotationMatrix[1][0], rotationMatrix[1][1], rotationMatrix[1][2], pos[1]},
    {rotationMatrix[2][0], rotationMatrix[2][1], rotationMatrix[2][2], pos[2]},
    {0,                            0,                    0,              1   }
  };
  copyTransformMatrix(temp, tMatrix);
}


void multiplyTwoMatrices(transformMatrix& aTb, transformMatrix& bTc, transformMatrix& aTc) {
  /*
      Input: bTa, bTc

      Output: aTc = bTa * bTc

    */
  rotationMatrix bTa, cTb, cTa;
  arrayOf3 bPa, cPb, cPa;

  extractRotationMatrix(aTb, bTa);
  extractRotationMatrix(bTc, cTb);


  rotationMatrixMultiplication(bTa, cTb, cTa); //aTc = aTb * bTc

  getPositionVectorFromTransformMatrix(aTb, bPa);
  getPositionVectorFromTransformMatrix(bTc, cPb);

  arrayOf3 temp;
  rotationMatrixMultiplication(bTa, cPb, temp);               //temp = bTa * cPb
  sumPositionVectors(temp, bPa, cPa);  //cPa = temp + bPa

  createTransformMatrixFromRandP(cTa, cPa, aTc);  // Rotation Matrix and Position Vector : aTc (C relative A)

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