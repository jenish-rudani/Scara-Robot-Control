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

#define L10 10
#define L30 30
#define L70 70
#define L80 80
#define L130 130
#define L140 140
#define D2 142
#define D1 195
#define L410 410


typedef double transformMatrix[4][4];
typedef double rotationMatrix[3][3];

typedef double arrayOf3[3];


JOINT sPt_toolPositionRelativeToStation;

int main(void);

void KIN(JOINT& jointVar, transformMatrix& writstRelativeBaseT);
void WHERE(JOINT& jointVar, JOINT& sPt);
void forwardKinematics(JOINT& jointVariables, JOINT& sPt_toolPostionWRTStation);
bool checkIfJointsAreWithinConstraints(JOINT& joinVar);
void copyArray(JOINT& inputArray, JOINT& outputArray);
void getJointParametersFromUser(double& theta_1, double& theta_2, double& d3, double& theta_4);
void convertJointPramaterAnglesToRadian(double& theta_1, double& theta_2, double d3, double& theta_4, JOINT& jointVariables);
void toggleGripper(bool& negativeCurrentGripperStatus);


void getPositionVectorFromTransformMatrix(transformMatrix& tmat, arrayOf3& pos);
void sumPositionVectors(arrayOf3& pos1, arrayOf3& pos2, arrayOf3& res);
void createTransformMatrixFromRandP(rotationMatrix& rotationMatrix, arrayOf3& pos, transformMatrix& tMatrix);
void multiplyTwoMatrices(transformMatrix& aTb, transformMatrix& bTc, transformMatrix& aTc);
void copyTransformMatrix(transformMatrix& inputTMatrix, transformMatrix& outputTMatrix);
void parametersToTransformMatrix(JOINT& inputParameters, transformMatrix& outputTransformMatrix);
void transformMatrixToParameters(transformMatrix& inputTransformMatrix, JOINT& outPutParameters);

void extractRotationMatrix(transformMatrix& transformMatrix, rotationMatrix& rotationMatrix);
void rotationMatrixMultiplication(rotationMatrix& firstMatrix, rotationMatrix& secondMatrix, rotationMatrix& outputMatrix);
void rotationMatrixMultiplication(rotationMatrix& rotationMatrix, arrayOf3& positionVector, arrayOf3& result);
