#include "GainOptimizer.h"


GainOptimizer::GainOptimizer()
{
    //initialize temperature and iterations
    initialTemp = 20;
    numIterations = 400;

    //set bounds
    bounds.push_back(0);
    bounds.push_back(21);

    //seed random number generator with current time
    srand(time(0));
}


void GainOptimizer::evaluate()
{
    //generate and evaluate random starting solution
    bestSoln = rand() % int(bounds.at(1)); 
    bestScore = objectiveFunction(bestSoln);

    //set starting point as current solution
    currSoln = bestSoln;
    currScore = bestScore;

    //initialize temperature variable
    double currTemp = initialTemp;

    for (int i = 0; i < numIterations; i++)
    {
        //take random step
        double step = ((rand() % 3000 / 200.0) - 7.5);
        double candidateSoln = currSoln + step;

        //enforce bounds
        if(candidateSoln < bounds.at(0)) candidateSoln = bounds.at(0);
        else if(candidateSoln > bounds.at(1)) candidateSoln = bounds.at(1);

        //evaluate new candidate solution
        double candidateScore = objectiveFunction(candidateSoln);
        
        //update best if candidate solution is better
        if(candidateScore < bestScore)
        {
            bestScore = candidateScore;
            bestSoln = candidateSoln;
        }

        //decrease temperature with linear annealing schedule
        currTemp -= initialTemp/numIterations;

        //calculate metropolis acceptance criteria
        double diff = candidateScore - currScore;
        double metropolisCriteria = exp(-diff/currTemp);

        //compare metropolis criteria to random value between 0 and 1 for acceptance
        //if diff is negative, solution is automatically accepted (represents better soln)
        if (diff < 0 || (rand() % 100) / 100.0 < metropolisCriteria)
        {
            currSoln = candidateSoln;
            currScore = candidateScore;
        }
    }
    cout << "Best solution occurred at x = " << bestSoln << ", with a value of " << bestScore << endl;
}


double GainOptimizer::objectiveFunction(double solution)
{
    double result = 0.01*solution*solution*sin(solution) + 1.5*sin(2.718*solution);
    return result;
}