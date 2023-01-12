#ifndef GENERATOR_H
#define GENERATOR_H

#include "Simulator.h"
#include <iostream>
#include <vector>
#include <cmath>

class Generator
{
    public:
    Generator();
    void generateTrajectories();
    double simulate(Simulator* currSim, double deploymentAngle);
    void controlPaddles();

    private:
    vector<Simulator*> simulations;
    double desiredApogee = 3048; // m
    double tolerance = 0.001; //0.1 percent
    double maxAngle = 70 * (M_PI/180);  //radians
    void populateInitialConditions(double, double, vector<double>&, vector<double>&);
    void adjustAngle(double& deploymentAngle, double& angleStep, double finalApogee);

};


#endif //GENERATOR_H