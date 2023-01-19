#include "GainOptimizer.h"


GainOptimizer::GainOptimizer()
{
    //initialize temperature and iterations
    initialTemp = 20;
    numIterations = 400;

    //set bounds
    bounds = {0, 20, 0, 20, 0, 20};

    //seed random number generator with current time
    srand(time(0));
}


void GainOptimizer::evaluate()
{
    //generate and evaluate random starting solution
    bestSoln.setGains(1,1,1);
    bestSoln.setScore(objectiveFunction(bestSoln));

    //set starting point as current solution
    currSoln.equals(bestSoln);

    //initialize temperature variable
    double currTemp = initialTemp;

    for (int i = 0; i < numIterations; i++)
    {
        //take random step
        Solution candidateSoln = takeStep(currSoln);

        //ensure solution is within bounds
        enforceBounds(candidateSoln);

        //evaluate new candidate solution
        candidateSoln.setScore(objectiveFunction(candidateSoln));
        
        //update best if candidate solution is better
        if(candidateSoln.score < bestSoln.score) bestSoln.equals(candidateSoln);

        //decrease temperature with linear annealing schedule
        currTemp -= initialTemp/numIterations;

        //calculate metropolis acceptance criteria
        double diff = candidateSoln.score - currSoln.score;
        double metropolisCriteria = exp(-diff/currTemp);

        //compare metropolis criteria to random value between 0 and 1 for acceptance
        //if diff is negative, solution is automatically accepted (represents better soln)
        if (diff < 0 || (rand() % 100) / 100.0 < metropolisCriteria)
        {
            currSoln.equals(candidateSoln);
        }
    }
    
}


double GainOptimizer::objectiveFunction(Solution soln)
{
    double result = 0;
    //FIXME: implement objective function
    return result;
}


Solution GainOptimizer::takeStep(Solution currSoln)
{
    Solution candidateSoln;

    double kpStep = 0;
    double kiStep = 0;
    double kdStep = 0;



    return candidateSoln;
}


void GainOptimizer::enforceBounds(Solution& candidateSoln)
{
    if (candidateSoln.kp < bounds.at(0)) candidateSoln.kp = bounds.at(0);
    else if (candidateSoln.kp > bounds.at(1)) candidateSoln.kp = bounds.at(1);

    if (candidateSoln.ki < bounds.at(2)) candidateSoln.ki = bounds.at(2);
    else if (candidateSoln.ki > bounds.at(3)) candidateSoln.ki = bounds.at(3);

    if (candidateSoln.kd < bounds.at(4)) candidateSoln.kd = bounds.at(4);
    else if (candidateSoln.kd > bounds.at(5)) candidateSoln.kd = bounds.at(5);
}


double GainOptimizer::randn()
{
    const double e = 2.78128;
    const double sigma = 2;
    const double mu = 0;

    double returnVal = 0;
    bool acceptVal = false;

    while(!acceptVal)
    {
        double x = (rand() % 200 / 100.0) - 1;
        double expTerm = -1*pow((x - mu), 2) / (2*pow(sigma, 2));
        double prob = 1/(sigma*sqrt(2*M_PI))*expTerm;
        if (prob > rand() % 100 / 100.0)
        {
            acceptVal = true;
            returnVal = x;
        }
    }

    return returnVal;
}
