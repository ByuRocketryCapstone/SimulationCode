#ifndef GENERATOR_H
#define GENERATOR_H

/*
File: Generator.h
Author: Gerritt Graham
Description: Class that uses the Simulator class to generate an optimal reference trajectory for 
the PID controller. This is a brute force solution that tries imposing a constant paddle deployment
angle over the whole flight. The chosen angle is adjusted until the target apogee is reached, then flight
information is recorded to be used as a reference by the PID controller.
*/

#include "Simulator.h"
#include "Controller.h"
#include "consts.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class Generator
{
    public:
    Generator();
    void generateTrajectories();

    private:
    vector<Simulator*> simulations;
    double tolerance, maxAngle, seedHeight, seedVelocity;
    void populateInitialConditions(double, double, vector<double>&, vector<double>&);
    void adjustAngle(double& deploymentAngle, double& angleStep, double finalApogee);

};


#endif //GENERATOR_H