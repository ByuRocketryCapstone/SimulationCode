#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

#include "Simulator.h"
#include "Controller.h"
#include "Generator.h"
#include "GainOptimizer.h"

using namespace std;

// h_0 = 762.9144;         //height at MECO, m
// V_0 = 284.57;           //velocity at MECO, m/s


int main()
{
    Simulator currSim(762.9144+20, 284.57+10, 0);
    Controller controller(6.178,0.66,1.826);

    currSim.simulate(controller);
    
    currSim.writeRecord("SimRecords/simulation1.txt");

    return 0;
}