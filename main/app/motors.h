// 电机输出（接口）

#pragma once

void setupMotors();
int  getDutyCycle(float value);
void sendMotors();
bool motorsActive();
void testMotor(int n);
