// Toggles which mode the simulator is in. To test the control algorithm, set the
// TEST_CONTROL_SCHEME variable to true. To generate optimal reference trajectories for the PID
// controller, set GENERATE_TRAJECTORIES to true. To tune gains for the PID controller with the
// optimizer, set TUNE_CONTROLLER_GAINS to true. Set the other two options to false.
#define TEST_CONTROL_SCHEME true
#define GENERATE_TRAJECTORIES false
#define TUNE_CONTROLLER_GAINS false

#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#if TEST_CONTROL_SCHEME

#include <chrono>

#include "Simulator.h"

using namespace std;
using namespace std::chrono;

// h_0 = 762.9144;         //height at MECO, m
// V_0 = 284.57;           //velocity at MECO, m/s

void simulate(Simulator& currSim);
double controlSchemeUpdate(double h, double V, double a, double theta);


int main()
{
    Simulator currSim(762.9144, 284.57, 0);

    auto start = high_resolution_clock::now();

    simulate(currSim);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    //cout << "time: " << duration.count() << endl;

    return 0;
}


void simulate(Simulator& currSim)
{
    double currH, currV, currA;
    double alpha = 40 * (3.14159/180);
    do
    {
        currSim.calcNextStep(currH, currV, currA, alpha);
        alpha = controlSchemeUpdate(currH, currV, currA, 0);
    } while(currV > 0.1);
    
    //cout << "h: " << currH << endl;
    
}


double controlSchemeUpdate(double h, double V, double a, double theta)
{
    return 40 * (3.14159/180);
    //FIXME: add stepper motor speed limits so paddles can't instantly deploy
}


#endif //TEST_CONTROL_SCHEME



#if GENERATE_TRAJECTORIES

#include "Generator.h"
#include "Controller.h"
#include "GainOptimizer.h"

int main()
{
    Generator generator;
    generator.generateTrajectories();

    // GainOptimizer optimizer;
    // optimizer.evaluate();

    return 0;
}

#endif //GENERATE_TRAJECTORIES