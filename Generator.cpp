#include "Generator.h"

Generator::Generator()
{
    desiredApogee = 3048;   // m
    tolerance = 0.001;      //0.1 percent
    maxAngle = 70 * (M_PI/180);     //radians

    // starting height and velocity values at MECO obtained from OpenRocket
    seedVelocity = 284.57;        //m/s
    seedHeight = 762.9144;          //m
}


// Brute force solution to generate optimal reference trajectories. A seed height and velocity at 
// main engine cutoff (MECO) is obtained from OpenRocket, and is used to generate a suite of height
// and velocity combinations that could potentially be seen in flight. For each combination of height
// and velocity, and constant paddle angle is found that results in the desired apogee
void Generator::generateTrajectories()
{
    vector<double> initialVelocities, initialHeights;
    populateInitialConditions(seedVelocity, seedHeight, initialVelocities, initialHeights);
    string outputFilename = "";
    int simNum = 0;

    for (int i = 0; i < initialHeights.size(); i++)
    {
        for(int j = 0; j < initialVelocities.size(); j++)
        {
            simNum++;
            outputFilename = "datafile" + to_string(simNum) + ".txt";
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
                adjustAngle(deploymentAngle, angleStep, finalApogee);   //FIXME: restrict speed of angle change

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
            currSim->writeRecord("SimRecords/" + outputFilename);
        }
    }
}


// Exectutes the simulation of the currSim argument at a specified paddle deployment angle
// in radians
double Generator::simulate(Simulator* currSim, double deploymentAngle)
{
    double hOut, VOut, aOut, tOut;  
    currSim->calcNextStep(hOut, VOut, aOut, tOut, deploymentAngle);
    while (VOut > 0.1)
    {
        currSim->calcNextStep(hOut, VOut, aOut, tOut, deploymentAngle);
    }
    return currSim->getApogee();
}


// Generate combinations of MECO heights and velocities using the given seed values
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


// Increase or decrease the selected deployment angle for the next simulation depending on the results
// of the previous simulation. A variable angle step is used to reduce computation time
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