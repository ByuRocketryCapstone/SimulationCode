#ifndef GAIN_OPTIMIZER_H
#define GAIN_OPTIMIZER_H

/*
File: GainOptimizer.h
Author: Gerritt Graham
Description: Class implementing a simluated annealing optimizer to automatically tune the gains of the PID
controller. The optimizer uses a linear annealing schedule and the Metropolis acceptance criteria. The 
objective function runs a simulation using the Simulator class, and compares the final trajectory to the
reference trajectory with a weighted average to calculate error.  
*/

#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

class GainOptimizer
{
    public:
    GainOptimizer();
    void evaluate();

    private:
    int numIterations;
    double initialTemp, bestSoln, bestScore, currSoln, currScore;
    vector<double> bounds;
    // double kp, ki, kd;
    
    double objectiveFunction(double solution);


};


#endif //GAIN_OPTIMIZER_H