// Toggles which mode the simulator is in. To test the control algorithm, set the
// TEST_CONTROL_SCHEME variable to true. To generate optimal reference trajectories for the PID
// controller, set GENERATE_TRAJECTORIES to true. To tune gains for the PID controller with the
// optimizer, set TUNE_CONTROLLER_GAINS to true. Set the other two options to false.
#define TEST_CONTROL_SCHEME false
#define GENERATE_TRAJECTORIES true
#define TUNE_CONTROLLER_GAINS false

#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#if TEST_CONTROL_SCHEME

#include <chrono>

#include "Simulator.h"
#include "Controller.h"

using namespace std;
using namespace std::chrono;

// h_0 = 762.9144;         //height at MECO, m
// V_0 = 284.57;           //velocity at MECO, m/s

void simulate(Simulator& currSim, Controller& controller);
double controlSchemeUpdate(Controller& controller, double h, double V, double a, double t, double theta);


int main()
{
    Simulator currSim(762.9144+50, 284.57+50, 0);
    Controller controller(4,1,2);

    auto start = high_resolution_clock::now();

    simulate(currSim, controller);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    //cout << "time: " << duration.count() << endl;
    
    currSim.writeRecord("SimRecords/simulation1.txt");

    return 0;
}


void simulate(Simulator& currSim, Controller& controller)
{
    double currH, currV, currA, currTime, lastTime;
    double alpha = 0, cmd_alpha = 0;
    do
    {
        currSim.calcNextStep(currH, currV, currA, currTime, alpha);
        if (1 || currTime >= lastTime + 0.5)
        {
            alpha = controlSchemeUpdate(controller, currH, currV, currA, currTime, 0);
            //lastTime = currTime;
        }
        
        //insert code here that matches hardware deployment capability
        
    } while(currV > 0.1);
}


double controlSchemeUpdate(Controller& controller, double h, double V, double a, double t, double theta)
{
    return controller.calcAngle(t, h, V, a);
    //FIXME: add stepper motor speed limits so paddles can't instantly deploy
}


#endif //TEST_CONTROL_SCHEME



#if GENERATE_TRAJECTORIES

#include "Generator.h"
#include "Controller.h"
#include "GainOptimizer.h"

double randn()
{
    const double e = 2.78128;
    const double sigma = 1;
    const double mu = 0;

    double returnVal = 0;
    bool acceptVal = false;

    // while(!acceptVal)
    // {
        double x = 1; //(rand() % 200 / 100.0) - 1;
        double expTerm = -1*pow((x - mu), 2) / (2*pow(sigma, 2));
        double prob = 1/(sigma*sqrt(2*M_PI))*expTerm;
        cout << prob << endl;
        if (prob > rand() % 100 / 100.0)
        {
            acceptVal = true;
            returnVal = x;
        }
    // }
    
    return returnVal;
}

int main()
{
    // Generator generator;
    // generator.generateTrajectories();

    // GainOptimizer optimizer;
    // optimizer.evaluate();

    for(int i = 0; i < 10; i++)
    {
        randn();
    }

    return 0;
}

#endif //GENERATE_TRAJECTORIES