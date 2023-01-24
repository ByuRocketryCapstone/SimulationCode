#ifndef SIMULATOR_H
#define SIMULATOR_H


/*
File: Simulator.h
Author: Gerritt Graham
Description: Class containing logic to simulate unpowered rocket flight. Simulation starts with
rocket height and velocity values at MECO, then performs an energy balance at discrete height
steps until apogee. 
*/


#include "consts.h"
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <ctime>

using namespace std;

class Simulator
{
    public:
    Simulator(double h0, double V0, double theta);
    ~Simulator();
    void calcNextStep(double& hOut, double& VOut, double& aOut, double& tOut, double alpha);
    double getApogee();
    void writeRecord(string fileSpec = "");
    double calcError(int refFileNum);

    private:
    const string PARAMETERS_FILE = "parameters.txt";
    const string RECORDS_DIRECTORY = "SimRecords/";

    double h, V, a, theta, currTime;
    double m_r, Cd_r, D_r, A_r, L_p, W_p, g, launchHeight;
    double heightStep;
    
    vector<double> timeVals, heightVals, velocityVals, accelVals, alphaVals;

    double getAirDensity(double h);
    double getPaddleDrag(double alpha);

    void populateParameters(ifstream& reader);
    vector<string> split(const string& s, char delimiter);
    

};

#endif //SIMULATOR_H