#include <chrono>
#include <iostream>

#include "Simulator.h"

using namespace std;
using namespace std::chrono;

// h_0 = 762.9144;         //height at MECO, m
// V_0 = 284.57;           //velocity at MECO, m/s

void simulate(Simulator& currSim);
double controlSchemeUpdate(double h, double V, double a, double theta);

int main()
{
    Simulator currSim(762.9144, 284.57, 0, 0.05);

    auto start = high_resolution_clock::now();

    simulate(currSim);
    
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "time: " << duration.count() << endl;

    return 0;
}

void simulate(Simulator& currSim)
{
    double currH, currV, currA;
    double alpha = 0;
    do
    //for (int i = 0; i < 100000; i++)
    {
        currSim.calcNextStep(currH, currV, currA, alpha);
        alpha = controlSchemeUpdate(currH, currV, currA, 0);
    } while(currV > 0.1);
    
    cout << "h: " << currH << endl;
    
}


double controlSchemeUpdate(double h, double V, double a, double theta)
{
    return 0.0;
}