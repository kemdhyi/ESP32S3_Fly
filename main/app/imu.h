// IMU（接口）

#pragma once

#include "app/math/vector.h"

void setupIMU();
void configureIMU();
void readIMU();
void calibrateGyroOnce();
void calibrateAccel();
void calibrateAccelOnce();
void printIMUCalibration();
void printIMUInfo();
