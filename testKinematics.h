#pragma once
#include "ensc-488.h"
#include "ensc894.h"
#include "stateid.h"
#include "StdAfx.h"

bool inverseKinematics2(JOINT& sPt_toolPostionWRTStation, uint8_t isItAFollowUp, uint8_t isItPickAndPlace);