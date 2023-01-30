#include "GainOptimizer.h"
#include <iostream>
#include <vector>
#include "Simulator.h"

GainOptimizer::GainOptimizer()
{
    //initialize temperature and iterations
    initialTemp = 200;
    numIterations = 50;

    //set bounds
    bounds = {0, 20, 0, 5, 0, 3};

    //seed random number generator with current time
    srand(time(0));
}


Solution GainOptimizer::evaluate()
{
    // vector<Solution> bestSolnsList;
    // for(int j = 0; j < 5; j++)
    // {
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
            if (i == int(0.6*numIterations)) currSoln.equals(bestSoln);

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
            if (i < 0.6*numIterations) 
            {
                currTemp = initialTemp - (0.1*initialTemp/(0.6*numIterations)) * i;
            }
            else if (i >= 0.6*numIterations && i <= numIterations) 
            {
                currTemp = initialTemp - (0.9*initialTemp/(0.4*numIterations)) * (i - 0.6*numIterations);
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

        return bestSoln;

    //     Solution tmpBest;
    //     tmpBest.equals(bestSoln);
    //     bestSolnsList.push_back(tmpBest);
    // }
    // for(int i = 0; i < bestSolnsList.size(); i++)
    // {
    //     cout << "kp: " << bestSolnsList.at(i).kp << ", ki: " << bestSolnsList.at(i).ki << ", kd: " << bestSolnsList.at(i).kd << endl; 
    // }
}


double GainOptimizer::objectiveFunction(Solution soln)
{
    double result = 0;

     ifstream reader(PARAMETERS_FILE);
    if(!reader.is_open()) 
    {
        cout << "Parameters file not opened in Simulator::Simulator()." << endl;
        return;
    }

    populateParameters(reader);

    Controller controller(soln.kp, soln.ki, soln.kd);
    Simulator currSim(762.9144+height_pertibation, 284.57+vel_pertibation, 0);
    
    currSim.simulate(controller);

    result = currSim.calcError(1);

    return result;
}


Solution GainOptimizer::takeStep(Solution currSoln, double currTemp)
{
    Solution candidateSoln;

    double kpStep = (rand() % 5000*(currTemp/initialTemp) / 500.0) - 5;
    double kiStep = (rand() % 1000*(currTemp/initialTemp) / 500.0) - 1;
    double kdStep = (rand() % 500*(currTemp/initialTemp) / 500.0) - 0.5;

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

void GainOptimizer::findPertibationSolution(){

    vector<Solution> Solution_Options;


    for (int i = 1; i <= 5; i++){

    evaluate();

    Solution_Options.push_back(bestSoln);

    }
    
    for (int i = 0; i <= 4; i++){
    
    cout << Solution_Options.at(i).kp <<" "<< Solution_Options.at(i).ki <<" "<< Solution_Options.at(i).kd << endl;
    }

   for (int i = 0; i <= 4; i++){

    Solution_Options.at(i);
    objectiveFunction(Solution_Options.at(i));   
    
    }

}
