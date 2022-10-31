#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>

using namespace std;

class Simulator
{
    public:
    Simulator(double h0, double V0, double theta, double stepSize);
    void calcNextStep(double& hOut, double& VOut, double& aOut, double alpha);

    private:
    const string PARAMETERS_FILE = "parameters.txt";

    double h, V, a, theta;
    double m_r, Cd_r, D_r, A_r, L_p, W_p, g, launchHeight;
    double heightStep;
    vector<double> heightVals, velocityVals, alphaVals;

    double getAirDensity(double h);
    double getPaddleDrag(double alpha);

    void populateParameters(ifstream& reader);
    

};

#endif //SIMULATOR_H