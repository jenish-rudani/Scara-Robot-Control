#include "helperFunctions.h"
#include<cmath>
#include <iomanip>
const char separator = ' ';
const int positionVectoryPrintWidth = 4;
const int jointPrintWidth = 7;

void printJointParameters(JOINT& var) {

  cout << "  [ ";
  cout << setw(jointPrintWidth) << setfill(' ') << right << roundOff(RAD2DEG(var[0])) << " | ";
  cout << setw(jointPrintWidth) << setfill(' ') << right << roundOff(RAD2DEG(var[1])) << " | ";
  cout << setw(jointPrintWidth - 3) << setfill(' ') << right << roundOff((var[2])) << " | ";
  cout << setw(jointPrintWidth) << setfill(' ') << right << roundOff(RAD2DEG(var[1]));
  cout << " ]" << endl;
}

void printXYZPhi(JOINT& var) {
  cout << "  [ ";
  for (size_t i = 0; i < 4; i++)
  {
    if (i == 3) cout << internal << setw(positionVectoryPrintWidth) << setfill(separator) << roundOff(RAD2DEG(var[i]));
    else cout << internal << setw(positionVectoryPrintWidth) << setfill(separator) << roundOff(var[i]) << " | ";
  }
  cout << " ]" << endl;
}

void printJoint(JOINT& joint) {

  cout << endl;
  for (size_t i = 0; i < 4; i++)
  {
    printf("Joint Parameter {%d}: {%f}\n", i, joint[i]);
  }
  cout << endl;
}

float roundOff(JOINT& joint)
{
  printf("#############################################\n\n");
  printf("Rounding Off Joint Parameters, Before Rouding off: ");
  printJoint(joint);
  for (size_t i = 0; i < 4; i++)
  {
    joint[i] = std::ceil(joint[i] * 100.0) / 100.0;
  }
  printf("After Rouding off");
  printJoint(joint);
  printf("####################################\n");
  return 1;
}

float roundOff(float var)
{
  return std::ceil(var * 100.0) / 100.0;
}

void displayJointVar(vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff) {
  for (int i = 0; i < 4; i++) {

    switch (i)
    {
    case 0:
      cout << "\n\t******* Coefficients For 'Theta 1' *******\n";
      break;
    case 1:
      cout << "\n\t******* Coefficients For 'Theta 2' *******\n";
      break;
    case 2:
      cout << "\n\t******* Coefficients For 'D3' *******\n";
      break;
    case 3:
      cout << "\n\t******* Coefficients For 'Theta 4' *******\n";
      break;
    }
    cout << "\tS to A : [" << currJConfig2A_coeff[i][0] << " , " << currJConfig2A_coeff[i][1] << " , " << currJConfig2A_coeff[i][2] << " , " << currJConfig2A_coeff[i][3] << " ]" << endl;
    cout << "\tA to B : [" << A2B_coeff[i][0] << " , " << A2B_coeff[i][1] << " , " << A2B_coeff[i][2] << " , " << A2B_coeff[i][3] << " ]" << endl;
    cout << "\tB to C : [" << B2C_coeff[i][0] << " , " << B2C_coeff[i][1] << " , " << B2C_coeff[i][2] << " , " << B2C_coeff[i][3] << " ]" << endl;
    cout << "\tC to G : [" << C2G_coeff[i][0] << " , " << C2G_coeff[i][1] << " , " << C2G_coeff[i][2] << " , " << C2G_coeff[i][3] << " ]" << endl;
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
  rotationMatrixMultiplication(bRa, negative_aPb, bPa);         //Calcluate bPa = bRa * -aPb 

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