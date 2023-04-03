#include "Generator.h"

Generator::Generator()
{
    tolerance = 0.001;      //0.1 percent
    maxAngle = MAX_PADDLE_ANGLE;

    // starting height and velocity values at MECO obtained from OpenRocket
    seedVelocity = mecoVelocity;        //m/s
    seedHeight = mecoHeight;          //m
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

    ofstream indexWriter(REF_DIRECTORY + INDEX_FILE_NAME);
    if (!indexWriter.is_open())
    {
        cout << "Index file not opened in Generator::generateTrajectories()" << endl;
    }

    //dummy controller object to satisfy argument of Simulator::simulate()
    Controller dummyController(0,0,0,0,0);

    for (int i = 0; i < initialHeights.size(); i++)
    {
        for(int j = 0; j < initialVelocities.size(); j++)
        {
            simNum++;
            cout << "Sim " << simNum << endl;
            outputFilename = REF_FILE_BASE + to_string(simNum) + ".txt";

            Simulator currSim(0,0,0);
            double finalApogee = 0;     //m
            double deploymentAngle = 10 * (M_PI/180);   //radians
            double angleStep = 0 * (M_PI/180);   //radians
            bool keepLooping = true;
            int numRuns = 0;

            while(keepLooping)
            {
                numRuns++;
                keepLooping = false;
                currSim.reset(initialHeights.at(i), initialVelocities.at(j), deploymentAngle);
                currSim.simulate(dummyController);
                finalApogee = currSim.getApogee();
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
            if (abs(finalApogee - TARGET_APOGEE) < TARGET_APOGEE*tolerance)
            {
                currSim.writeRecord(REF_DIRECTORY + outputFilename);
                indexWriter << initialHeights.at(i) << " " << initialVelocities.at(j) << " " 
                << REF_FILE_BASE + to_string(simNum) + ".txt";
                if (simNum < initialHeights.size()*initialVelocities.size()) indexWriter << endl;
            }
        }
    }
}


// Generate combinations of MECO heights and velocities using the given seed values
void Generator::populateInitialConditions(double seedVel, double seedHeight, 
    vector<double>& velocities, vector<double>& heights)
{
    heights.push_back(seedHeight);
    velocities.push_back(seedVel);

    const int NUM_HEIGHT_STEPS = 2;
    const int NUM_VEL_STEPS = 3;
    const int HEIGHT_STEP = 40;     //m
    const int VELOCITY_STEP = 20;   //m/s

    for(int i = 1; i <= NUM_HEIGHT_STEPS; i++)
    {
        heights.push_back(seedHeight + i*HEIGHT_STEP);
        heights.push_back(seedHeight - i*HEIGHT_STEP);
    }
    for(int i = 1; i <= NUM_VEL_STEPS; i++)
    {
        velocities.push_back(seedVel + i*VELOCITY_STEP);
        velocities.push_back(seedVel - i*VELOCITY_STEP);
    }
}


// Increase or decrease the selected deployment angle for the next simulation depending on the results
// of the previous simulation. A variable angle step is used to reduce computation time
void Generator::adjustAngle(double& deploymentAngle, double& angleStep, double finalApogee)
{
    // adjust angle step based on how close the simulation is to the target
    if (abs(finalApogee - TARGET_APOGEE) > TARGET_APOGEE*0.5) angleStep = 3 * (M_PI/180);
    else if (abs(finalApogee - TARGET_APOGEE) > TARGET_APOGEE*0.25) angleStep = 1 * (M_PI/180);
    else if (abs(finalApogee - TARGET_APOGEE) > TARGET_APOGEE*0.1) angleStep = 0.5 * (M_PI/180);
    else /*if (abs(finalApogee - TARGET_APOGEE) > TARGET_APOGEE*0.03)*/ angleStep = 0.05 * (M_PI/180);

    // adjust deployment angle up if the rocket overshot the target, else adjust down
    if (finalApogee > TARGET_APOGEE*(1+tolerance)) deploymentAngle += angleStep;
    else if (finalApogee < TARGET_APOGEE*(1-tolerance)) deploymentAngle -= angleStep;
}