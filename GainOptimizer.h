#ifndef GAIN_OPTIMIZER_H
#define GAIN_OPTIMIZER_H

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