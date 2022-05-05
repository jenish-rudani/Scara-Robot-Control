#include "testKinematics.h"


// Inverse Kinematics #################################################################################################################
bool inverseKinematics2(JOINT& sPt_toolPostionWRTStation, uint8_t isItAFollowUp, uint8_t isItPickAndPlace) {

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