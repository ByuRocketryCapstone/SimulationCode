#include "Generator.h"

Generator::Generator()
{
    // do we need code here?
}


void Generator::generateTrajectories()
{
    // starting height and velocity values at MECO obtained from OpenRocket
    double seedVelocity = 284.57;        //m/s
    double seedHeight = 762.9144;          //m
    vector<double> initialVelocities, initialHeights;
    populateInitialConditions(seedVelocity, seedHeight, initialVelocities, initialHeights);
    string filename = "";
    int simNum = 0;

    for (int i = 0; i < initialHeights.size(); i++)
    {
        for(int j = 0; j < initialVelocities.size(); j++)
        {
            simNum++;
            filename = "datafile" + to_string(simNum) + ".txt";
            Simulator* currSim;
            double finalApogee = 0;     //m
            double deploymentAngle = 10 * (M_PI/180);   //radians
            double angleStep = 0 * (M_PI/180);   //radians
            bool keepLooping = true;
            int numRuns = 0;

            while(keepLooping)
            {
                numRuns++;
                keepLooping = false;
                currSim = new Simulator(initialHeights.at(i), initialVelocities.at(j), 0);
                finalApogee = simulate(currSim, deploymentAngle);
                double previousAngle = deploymentAngle;
                adjustAngle(deploymentAngle, angleStep, finalApogee);

                if (deploymentAngle > maxAngle || deploymentAngle < 0)
                {
                    cout << "Simulation not possible." << endl;
                    break;
                }

                // check if angle changed using double comparison method
                // if the angle changed, loop again
                if (abs(previousAngle - deploymentAngle) > 0.0001) keepLooping = true;
            }
            simulations.push_back(currSim);
            currSim->writeRecord("SimRecords/" + filename);
            // cout << simNum << ": " << "Final Angle: " << deploymentAngle * (180/M_PI) << endl;
            // cout << "Num Runs: " << numRuns << endl;
        }
    }
    // for (int i = 0; i < simulations.size(); i++)
    // {
    //     simulations.at(i)->writeRecord("SimRecords/" + filename);
    // }
}


double Generator::simulate(Simulator* currSim, double deploymentAngle)
{
    double hOut, VOut, aOut;  
    currSim->calcNextStep(hOut, VOut, aOut, deploymentAngle);
    while (VOut > 0.1)
    {
        currSim->calcNextStep(hOut, VOut, aOut, deploymentAngle);
    }
    return currSim->getApogee();
}


void Generator::controlPaddles()
{

}


void Generator::populateInitialConditions(double seedVel, double seedHeight, 
    vector<double>& velocities, vector<double>& heights)
{
    heights.push_back(seedHeight);
    velocities.push_back(seedVel);

    const int NUM_STEPS = 0;
    const int HEIGHT_STEP = 40;     //m
    const int VELOCITY_STEP = 10;   //m/s

    for(int i = 1; i <= NUM_STEPS; i++)
    {
        velocities.push_back(seedVel + i*VELOCITY_STEP);
        velocities.push_back(seedVel - i*VELOCITY_STEP);
        heights.push_back(seedHeight + i*HEIGHT_STEP);
        heights.push_back(seedHeight - i*HEIGHT_STEP);
    }
}


void Generator::adjustAngle(double& deploymentAngle, double& angleStep, double finalApogee)
{
    // adjust angle step based on how close the simulation is to the target
    if (abs(finalApogee - desiredApogee) > desiredApogee*0.5) angleStep = 3 * (M_PI/180);
    else if (abs(finalApogee - desiredApogee) > desiredApogee*0.25) angleStep = 1 * (M_PI/180);
    else if (abs(finalApogee - desiredApogee) > desiredApogee*0.1) angleStep = 0.5 * (M_PI/180);
    else if (abs(finalApogee - desiredApogee) > desiredApogee*0.03) angleStep = 0.1 * (M_PI/180);

    // adjust deployment angle up if the rocket overshot the target, else adjust down
    if (finalApogee > desiredApogee*(1+tolerance)) deploymentAngle += angleStep;
    else if (finalApogee < desiredApogee*(1-tolerance)) deploymentAngle -= angleStep;
}