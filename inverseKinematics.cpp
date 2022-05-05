#include "inverseKinematics.h"

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

  if (!firstFlag && !secondFlag)
  {
    printf("ERROR: (%f, %f, %f, %f)\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));
    printf("ERROR: (%f, %f, %f, %f)\n\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));
    cout << "\n\n$$$$$$$$$$$$$$$$$$$$$$$$\n\nNo Solutions\n\n";
    return 0;
  }

  else if (firstFlag && !secondFlag)
  {
    cout << "First Valid Solution:";
    printf("[1st] (%f, %f, %f, %f)\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));

    cout << "\n\n$$$$$$$$$$$$$$$$$$$$$$$$\n\nSecond Solution Invalid :  ";
    printf("[2nd] ERROR: (%f, %f, %f, %f)\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));

  }
  else if (!firstFlag && secondFlag)
  {
    cout << "\n\n$$$$$$$$$$$$$$$$$$$$$$$$\n\nFirst Solution Invalid:  ";
    printf("[1st] ERROR: (%f, %f, %f, %f)\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));

    cout << "Second Valid Solution:  ";
    printf("[2nd] (%f, %f, %f, %f)\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));
  }
  else
  {
    cout << "First Valid Solution:  ";
    printf("[1st] (%f, %f, %f, %f)\n", RAD2DEG(firstSolution[0]), RAD2DEG(firstSolution[1]), firstSolution[2], RAD2DEG(firstSolution[3]));

    cout << "Second Valid Solution:  ";
    printf("[2nd] (%f, %f, %f, %f)\n\n\n\n", RAD2DEG(secondSolution[0]), RAD2DEG(secondSolution[1]), secondSolution[2], RAD2DEG(secondSolution[3]));
  }

  // Move TO Closest Solution
  firstSolution[0] = RAD2DEG(firstSolution[0]);
  firstSolution[1] = RAD2DEG(firstSolution[1]);
  firstSolution[3] = RAD2DEG(firstSolution[3]);

  //secondSolution[0] = RAD2DEG(secondSolution[0]);
  //secondSolution[1] = RAD2DEG(secondSolution[1]);
  //secondSolution[3] = RAD2DEG(secondSolution[3]);

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

  double denominator = (2 * (142 * 195));
  double c2 = (pow(x, 2) + pow(y, 2) - pow(142, 2) - pow(195, 2)) / denominator;
  double s2 = sqrt(1 - pow(c2, 2));
  double temp111 = sqrt(pow(x, 2) + pow(y, 2));
  double temp123 = (double)D1_BASE_TO_JOINT2_X + (double)D2_WRIST_TO_JOINT2_X;

  if (!std::floor(roundOff(temp111 - temp123)))//Checks if there is 1 solution  
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

  transformMatrix base_Relativeto_Station_T, tool_Relativeto_Wrist_T;
  JOINT T{ 0, 0, 140, 0 };
  JOINT B{ 0, 0, 405, 0 };
  parametersToTransformMatrix(B, base_Relativeto_Station_T);
  parametersToTransformMatrix(T, tool_Relativeto_Wrist_T);

  transformMatrix wrels, bTw, sTt, bTt;
  transformMatrix bTs, tTw;

  parametersToTransformMatrixForInV(toolPosition, sTt);
  invertTransformMatrix(base_Relativeto_Station_T, bTs);
  invertTransformMatrix(tool_Relativeto_Wrist_T, tTw);
  multiplyTwoTransformMatrices(bTs, sTt, bTt);
  multiplyTwoTransformMatrices(bTt, tTw, bTw);
  bool status = calculateAllTwoSolutions(bTw, firstSol, secondSol, flagFirst, flagSecond);
  printf("\n\nK1: %d  |  K2 = %d\n\n", flagFirst, flagSecond);

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
      printf("Copying into First Solution, as both solutions are same");
      copyArray(firstSol, temp);
      copyArray(secondSol, firstSol);
      copyArray(temp, secondSol);
    }
  }
  return 1;
}
//#####################################################################################################################################
