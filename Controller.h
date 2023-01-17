#ifndef CONTROLLER_H
#define CONTROLLER_H

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
    Controller(double kp, double ki, double kd);
    double calcAngle(double currTime, double currHeight, double currVelocity, double currAccel);

    private:
    double kp, ki, kd;
    double ref_alpha, cmd_alpha;
    vector<double> refTimes, refHeights, refVelocities, refAccels;
    
    int findTimeIndex(double t);
    double getRefHeight(double t);
    double getRefVelocity(double t);
    double getRefAccel(double t);

    void loadData(string dataFile);
    vector<string> split(const string& s, char delimiter);
    double getCurrentAngle();

};


#endif //CONTROLLER_H