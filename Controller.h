#ifndef CONTROLLER_H
#define CONTROLLER_H

/*
File: Controller.h
Author: Gerritt Graham and Jacob Hansen
Description: Class implementing the PID controller to control the rocket in flight. This controller uses
a reference trajectory created by the Generator class to calculate error values, and uses a simple PID
formula to calculate a paddle deployment angle at each time step.
*/

#include "consts.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

using namespace std;

class Controller
{
    public:
    Controller(double kp, double ki, double kd, double h0, double V0);
    double calcAngle(double currTime, double currHeight, double currVelocity, double currAccel);
    int getTrajectoryNum();

    private:
    double kp, ki, kd;
    double ref_alpha, cmd_alpha;
    double mecoHeight, mecoVelocity;
    vector<double> refTimes, refHeights, refVelocities, refAccels;
    int selectedTrajectoryNum;
    
    int findTimeIndex(double t);
    double getRefHeight(double t);
    double getRefVelocity(double t);
    double getRefAccel(double t);

    void loadData();
    string selectFile();
    vector<string> split(const string& s, char delimiter);
    double getCurrentAngle();

};


#endif //CONTROLLER_H