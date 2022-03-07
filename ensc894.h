#pragma once
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include <chrono>
#include <algorithm>
#include "ensc-488.h"
#include "StdAfx.h"

//SCARA Various Lenghts

#define STATION_TO_BASE_Z 410
#define BASE_TO_JOINT2_Z 70
#define WRIST_TO_TOOL_Z 140

#define D2_WRIST_TO_JOINT2_X 142
#define D1_BASE_TO_JOINT2_X 195

//SCARA Constraints

#define THETA_1_DEG_CON 150
#define THETA_2_DEG_CON 100
#define THETA_4_DEG_CON 160
#define D3_MIN_BOUND -200
#define D3_MAX_BOUND -100
#define A210 210
#define A150 150

#define THETA_1_RADIAN_CON DEG2RAD(THETA_1_DEG_CON)
#define THETA_2_RADIAN_CON DEG2RAD(THETA_2_DEG_CON)
#define THETA_4_RADIAN_CON DEG2RAD(THETA_4_DEG_CON)


typedef double transformMatrix[4][4];
typedef double rotationMatrix[3][3];

typedef double arrayOf3[3];

int main(void);

void KIN(JOINT& jointVar, transformMatrix& writstRelativeBaseT);
void WHERE(JOINT& jointVar, JOINT& sPt);
void forwardKinematics(JOINT& jointVariables, JOINT& sPt_toolPostionWRTStation);

void getSolutionsForInverseKIN(JOINT& toolPosition, JOINT& currentJointConfig, JOINT& first, JOINT& second, bool& flagFirst, bool& flagSecond);
void calculateAllTwoSolutions(transformMatrix& bTw, JOINT& closestSolution, JOINT& farthestSolution, bool& firstFlag, bool& secondFlag);
void inverseKinematics(JOINT& jointVariables, JOINT& sPt_toolPostionWRTStation, uint8_t isItAFollowUp);

bool checkIfJointsAreWithinConstraints(JOINT& joinVar);
void copyArray(JOINT& inputArray, JOINT& outputArray);
void getJointParametersFromUser(double& theta_1, double& theta_2, double& d3, double& theta_4);
void convertJointPramaterAnglesToRadian(double& theta_1, double& theta_2, double d3, double& theta_4, JOINT& jointVariables);
void toggleGripper(bool& negativeCurrentGripperStatus);


void getPositionVectorFromTransformMatrix(transformMatrix& tmat, arrayOf3& pos);
void sumPositionVectors(arrayOf3& pos1, arrayOf3& pos2, arrayOf3& res);
void createTransformMatrixFrom_R_and_P(rotationMatrix& rotationMatrix, arrayOf3& pos, transformMatrix& tMatrix);
void multiplyTwoTransformMatrices(transformMatrix& aTb, transformMatrix& bTc, transformMatrix& aTc);
void copyTransformMatrix(transformMatrix& inputTMatrix, transformMatrix& outputTMatrix);
void parametersToTransformMatrix(JOINT& inputParameters, transformMatrix& outputTransformMatrix);
void parametersToTransformMatrixForInV(JOINT& inputParameters, transformMatrix& outputTransformMatrix);
void transformMatrixToParameters(transformMatrix& inputTransformMatrix, JOINT& outPutParameters);
void invertTransformMatrix(transformMatrix& aTb, transformMatrix& bTa);
void extractRotationMatrix(transformMatrix& transformMatrix, rotationMatrix& rotationMatrix);
void rotationMatrixMultiplication(rotationMatrix& firstMatrix, rotationMatrix& secondMatrix, rotationMatrix& outputMatrix);
void rotationMatrixMultiplication(rotationMatrix& rotationMatrix, arrayOf3& positionVector, arrayOf3& result);

void multiplyPositionArrayWithValue(arrayOf3& inputPositionArray, double value, arrayOf3& outputPositionArray);
void getToolPositionFromUser(double& x, double& y, double& z, double& phi);
void getTransposedMatrix(rotationMatrix& a_R_b, rotationMatrix& b_R_a);
void extractPositionFromTransformMatrix(transformMatrix& transformMatrix, arrayOf3& excractedPositionVector);