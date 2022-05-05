#include "ensc894.h"
#include "ensc-488.h"
#include "StdAfx.h"
#include<thread>

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
  std::this_thread::sleep_for(std::chrono::seconds(1));
  int inputOption;

  JOINT jointParameters, sPt;
  JOINT T{ 0, 0, 140, 0 };
  JOINT B{ 0, 0, 405, 0 };
  parametersToTransformMatrix(B, base_Relativeto_Station_T);
  parametersToTransformMatrix(T, tool_Relativeto_Wrist_T);


  JOINT test{ 90, 0, -175, 0 };
  MoveToConfiguration(test, true);


#if 1
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
      trajectoryPlanning();
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
#endif
  return 0;
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