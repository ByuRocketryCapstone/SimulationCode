#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

class Controller
{
    public:
    Controller(double kp, double ki, double kd);
    double calcAngle();

    private:
    double kp, ki, kd;
    vector<double> refTime, refHeight, refVelocity, refAccel;

    void loadData(string dataFile);
    vector<string> split(const string& s, char delimiter);
    int findTimeIndex(double t);

};


#endif //CONTROLLER_H