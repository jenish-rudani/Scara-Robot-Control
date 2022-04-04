#pragma once
#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>
#include <conio.h>
#include <fstream>
#include <chrono>
#include <algorithm>
#include "ensc-488.h"
#include "StdAfx.h"

using namespace std;

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
#define ANGLE_210 210
#define ANGLE_150 150
#define WRIST_TO_TOOL_Z 140
#define THETA_1_RADIAN_CON DEG2RAD(THETA_1_DEG_CON)
#define THETA_2_RADIAN_CON DEG2RAD(THETA_2_DEG_CON)
#define THETA_4_RADIAN_CON DEG2RAD(THETA_4_DEG_CON)


// Velocity constraints
#define JOINT1_VEL_LIM 150 
#define JOINT2_VEL_LIM 150
#define JOINT3_VEL_LIM 50
#define JOINT4_VEL_LIM 150

// Acceleration constraints
#define JOINT1_ACC_LIM 600
#define JOINT2_ACC_LIM 600
#define JOINT3_ACC_LIM 200
#define JOINT4_ACC_LIM 600

typedef double transformMatrix[4][4];
typedef double rotationMatrix[3][3];

typedef double arrayOf3[3];
typedef double arrayOf5[5];
typedef double arrayOf4[4];

int main(void);

void KIN(JOINT& jointVar, transformMatrix& writstRelativeBaseT);
void WHERE(JOINT& jointVar, JOINT& sPt);
void forwardKinematics(JOINT& jointVariables, JOINT& sPt_toolPostionWRTStation);

bool getSolutionsForInverseKIN(JOINT& toolPosition, JOINT& currentJointConfig, JOINT& first, JOINT& second, bool& flagFirst, bool& flagSecond);
bool calculateAllTwoSolutions(transformMatrix& bTw, JOINT& closestSolution, JOINT& farthestSolution, bool& firstFlag, bool& secondFlag);
bool inverseKinematics(JOINT& sPt_toolPostionWRTStation, uint8_t isItAFollowUp, uint8_t isItPickAndPlace);

void trajectoryPlanning();
void planPathBasedOnJointSpace(JOINT& currentJointConfiguration, JOINT& A_positionVar, JOINT& B_positionVar, JOINT& C_positionVar, JOINT& G_positionVar, double trajectoryTime);

bool checkIfJointsAreWithinConstraints(JOINT& joinVar);
void copyArray(JOINT& inputArray, JOINT& outputArray);
void getJointParametersFromUser(double& theta_1, double& theta_2, double& d3, double& theta_4);
void convertJointPramaterAnglesToRadian(double& theta_1, double& theta_2, double d3, double& theta_4, JOINT& jointVariables);
void toggleGripper(bool& negativeCurrentGripperStatus);

//##########################
void seprateJointParametersPerIndex(arrayOf5& outputJointParam, JOINT& currentJointParam, JOINT& jointAParam, JOINT& jointBParam, JOINT& jointCParam, JOINT& jointGParam, int index);
void prepareJointParamForEachFrames(JOINT& currentJointConfiguration, JOINT& nearSolution_A, JOINT& nearSolution_B, JOINT& nearSolution_C, JOINT& nearSolution_G, arrayOf5& jointArr_theta1, arrayOf5& jointArr_theta2, arrayOf5& jointArr_d3, arrayOf5& jointArr_theta4);

void generatePath(arrayOf5& timeArr, vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff, vector<vector<double>>& pathVector, vector<double>& currTimeVec);
void genPathHelperFunction(double ti, double tf, vector<double>& coeff, vector<double>& pos, vector<double>& currTimeVec, bool isFull);

void generateVelocity(arrayOf5& timeArr, vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff, vector<vector<double>>& velocityVec, vector<double>& currTimeVec);
void genVelocityHelperFunction(double ti, double tf, vector<double>& coeff, vector<double>& acc, vector<double>& currTimeVec, bool isFull);

void generateAcceleration(arrayOf5& timeArr, vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff, vector<vector<double>>& accelerationVec, vector<double>& currTimeVec);
void genAccelerationHelperFunction(double ti, double tf, vector<double>& coeff, vector<double>& acc, vector<double>& currTimeVec, bool isFull);

void calculateCoefficients(arrayOf5& jointParamArr, arrayOf5& trajectoryTimeSegments, JOINT& currJConfig2A_coeff, JOINT& A2B_coeff, JOINT& B2C_coeff, JOINT& C2G_coeff);
void calculateCubicCoefficients(double theta0, double thetaf, double vel0, double velf, double tf, JOINT& coeff);
void displayJointVar(vector<vector<double>>& currJConfig2A_coeff, vector<vector<double>>& A2B_coeff, vector<vector<double>>& B2C_coeff, vector<vector<double>>& C2G_coeff);

bool checkVelocityLimits(vector<vector<double>> vals);
bool checkAccLimits(vector<vector<double>> vals);
void genPos(vector<double> theta1, vector<double> theta2, vector<double>& d3, vector<double> theta4, vector<double>& x, vector<double>& y, vector<double>& z, vector<double>& phi);
void WriteParamToCsvFile(string filename, vector<pair<string, vector<double>>> data);
//##########################

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
void getToolPositionFromUser(double& x, double& y, double& z, double& phi, char* instructionString);
void getTransposedMatrix(rotationMatrix& a_R_b, rotationMatrix& b_R_a);
void extractPositionFromTransformMatrix(transformMatrix& transformMatrix, arrayOf3& excractedPositionVector);

void runPickAndPlace(void);