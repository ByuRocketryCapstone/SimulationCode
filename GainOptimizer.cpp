#include "GainOptimizer.h"


GainOptimizer::GainOptimizer()
{
    initialTemp = 20;
    numIterations = 400;
    bounds.push_back(0);
    bounds.push_back(21);
    srand(time(0));
}


void GainOptimizer::evaluate()
{
    bestSoln = rand() % int(bounds.at(1));   //generate random starting solution
    cout << "Best soln: " << bestSoln << endl;
    bestScore = objectiveFunction(bestSoln);
    currSoln = bestSoln;
    currScore = bestScore;
    double currTemp = initialTemp;

    for (int i = 0; i < numIterations; i++)
    {
        double step = ((rand() % 3000 / 200.0) - 7.5);
        double candidateSoln = currSoln + step;
        if(candidateSoln < bounds.at(0)) candidateSoln = bounds.at(0);
        else if(candidateSoln > bounds.at(1)) candidateSoln = bounds.at(1);
        double candidateScore = objectiveFunction(candidateSoln);

        if(candidateScore < bestScore)
        {
            bestScore = candidateScore;
            bestSoln = candidateSoln;
        }

        currTemp -= initialTemp/numIterations;
        double diff = candidateScore - currScore;
        double metropolisCriteria = exp(-diff/currTemp);
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