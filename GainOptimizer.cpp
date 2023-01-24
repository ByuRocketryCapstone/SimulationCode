#include "GainOptimizer.h"


GainOptimizer::GainOptimizer()
{
    //initialize temperature and iterations
    initialTemp = 100;
    numIterations = 2000;

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
        //move currSoln to the optimal close to the end of the annealing
        if (i == int(0.8*numIterations)) currSoln.equals(bestSoln);

        //take random step
        Solution candidateSoln = takeStep(currSoln, currTemp);

        //ensure solution is within bounds
        enforceBounds(candidateSoln);

        //evaluate new candidate solution
        candidateSoln.setScore(objectiveFunction(candidateSoln));
        
        //update best if candidate solution is better
        if(candidateSoln.score < bestSoln.score) 
        {
            bestSoln.equals(candidateSoln);
            cout << "New best found. kp = " << bestSoln.kp << ", ki = " 
                << bestSoln.ki << ", kd = " << bestSoln.kd << endl;
        }

        //decrease temperature with piecewise annealing schedule
        if (i < 0.8*numIterations) 
        {
            currTemp = initialTemp - (0.1*initialTemp/(0.8*numIterations)) * i;
        }
        else if (i >= 0.8*numIterations && i <= numIterations) 
        {
            currTemp = initialTemp - (0.9*initialTemp/(0.2*numIterations)) * (i - 0.8*numIterations);
        }
        else cout << "Error in GainOptimizer::evaluate(), i index out of bounds." << endl;

        //calculate metropolis acceptance criteria
        double diff = candidateSoln.score - currSoln.score;
        double metropolisCriteria = exp(-diff/currTemp);

        //compare metropolis criteria to random value between 0 and 1 for acceptance
        //if diff is negative, solution is automatically accepted (represents better soln)
        if (diff < 0 || ((rand() % 100) / 100.0) < metropolisCriteria)
        {
            currSoln.equals(candidateSoln);
        }

        if(i % 50 == 0) cout << "Iteration: " << i << endl;
    }
    cout << "kp: " << bestSoln.kp << endl;
    cout << "ki: " << bestSoln.ki << endl;
    cout << "kd: " << bestSoln.kd << endl;
}


double GainOptimizer::objectiveFunction(Solution soln)
{
    double result = 0;

    Controller controller(soln.kp, soln.ki, soln.kd);
    Simulator currSim(762.9144+20, 284.57+10, 0);
    //double getCurrentAngle();
    double currH, currV, currA, currTime, lastTime, rate;
    double alpha = 0, cmd_alpha = 0; 
    rate = 30*(M_PI/180); lastTime = 0; currTime = 0;
    do
    {
        currSim.calcNextStep(currH, currV, currA, currTime, alpha);
        cmd_alpha = controller.calcAngle(currTime, currH, currV, currA);
        //rate is 30 deg per second 
        //insert code here that matches hardware deployment capability
        if (cmd_alpha > alpha){
            //If the cmd angle is larger than the current angle ie the paddles need to open
          alpha += rate * (currTime - lastTime);
        }
        else if (cmd_alpha < alpha){
            //If the cmd angle is less than the current angle ie the paddles need to close
            alpha += -(rate * (currTime - lastTime));
        }
        lastTime = currTime;
        //Saturation Limits for extra robustness cause reasons
        if (alpha >= 70 * (M_PI/180)) alpha = 70 * (M_PI/180);
        else if (alpha <= 0) alpha = 0;
        else alpha = alpha;
        
    } while(currV > 0.1);

    result = currSim.calcError(1);

    return result;
}


Solution GainOptimizer::takeStep(Solution currSoln, double currTemp)
{
    Solution candidateSoln;

    double kpStep = (rand() % 2000*(currTemp/initialTemp) / 500.0) - 2;
    double kiStep = (rand() % 2000*(currTemp/initialTemp) / 500.0) - 2;
    double kdStep = (rand() % 2000*(currTemp/initialTemp) / 500.0) - 2;

    candidateSoln.kp = currSoln.kp + kpStep;
    candidateSoln.ki = currSoln.ki + kiStep;
    candidateSoln.kd = currSoln.kd + kdStep;

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

