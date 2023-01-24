// Toggles which mode the simulator is in. To test the control algorithm, set the
// TEST_CONTROL_SCHEME variable to true. To generate optimal reference trajectories for the PID
// controller, set GENERATE_TRAJECTORIES to true. To tune gains for the PID controller with the
// optimizer, set TUNE_CONTROLLER_GAINS to true. Set the other two options to false.
#define TEST_CONTROL_SCHEME true
#define GENERATE_TRAJECTORIES !TEST_CONTROL_SCHEME
//#define TUNE_CONTROLLER_GAINS false

#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#if TEST_CONTROL_SCHEME

#include <chrono>

#include "Simulator.h"
#include "Controller.h"
#include <cmath>

using namespace std;
using namespace std::chrono;

// h_0 = 762.9144;         //height at MECO, m
// V_0 = 284.57;           //velocity at MECO, m/s


int main()
{
    Simulator currSim(762.9144+20, 284.57+10, 0);
    Controller controller(6.178,0.66,1.826);

    auto start = high_resolution_clock::now();

    currSim.simulate(controller);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    //cout << "time: " << duration.count() << endl;
    
    currSim.writeRecord("SimRecords/simulation1.txt");

    return 0;
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