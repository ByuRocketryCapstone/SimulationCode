#ifndef GAIN_OPTIMIZER_H
#define GAIN_OPTIMIZER_H

/*
File: GainOptimizer.h
Author: Gerritt Graham and Jacob Hansen
Description: Class implementing a simluated annealing optimizer to automatically tune the gains of the PID
controller. The optimizer uses a linear annealing schedule and the Metropolis acceptance criteria. The 
objective function runs a simulation using the Simulator class, and compares the final trajectory to the
reference trajectory with a weighted average to calculate error.  
*/

#include "OptimizerSolution.h"
#include "Controller.h"
#include "Simulator.h"
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

class GainOptimizer
{
    public:
    GainOptimizer();
    Solution evaluate();
    void findPertibationSolution();
    // another evaulate function that evaulates 5 times 5 sets of gains, then try all five at different pertubations
    private:
    int numIterations;
    double initialTemp;
    Solution bestSoln, currSoln;
    vector<double> bounds;
    // double kp, ki, kd;
    
    double objectiveFunction(Solution soln);
    Solution takeStep(Solution currSoln, double currTemp);
    void enforceBounds(Solution& candidateSoln);
    double randn();
    double height_pertibation = 0;
    double vel_pertibation = 0;


};


#endif //GAIN_OPTIMIZER_H