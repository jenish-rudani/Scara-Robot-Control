#include "forwardKinematics.h"


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
    {cos_phi,  sin_phi, 	 0,  ((195 * cos(theta1)) + (142 * cos(theta1 + theta2)))},
    {sin_phi, -cos_phi,    0,  ((195 * sin(theta1)) + (142 * sin(theta1 + theta2)))},
    {0,         0,        -1,  BASE_TO_JOINT2_Z - (STATION_TO_BASE_Z + D3)},
    {0, 	      0, 		     0,  1}
  };

  copyTransformMatrix(temp, writstRelativeBaseT);
}


void WHERE(JOINT& jointVar, JOINT& sPt) {

  transformMatrix base_Relativeto_Station_T, tool_Relativeto_Wrist_T;
  JOINT T{ 0, 0, 140, 0 };
  JOINT B{ 0, 0, 405, 0 };
  parametersToTransformMatrix(B, base_Relativeto_Station_T);
  parametersToTransformMatrix(T, tool_Relativeto_Wrist_T);


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