/*
File: main.cpp
Author: Gerritt Graham
Description: Main file used to run the various simulation analyses. 
*/

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
    double openRocketHeight = 762.9144;     //m
    double openRocketVelocity = 284.57;     //m/s

    //string operationMode = "Simulate";
    string operationMode = "Generate";
    //string operationMode = "Optimize";
    

    if (operationMode == "Simulate")
    {
        Simulator currSim(openRocketHeight-25, openRocketVelocity+13, 0);
        Controller controller(9.83,1.46,0.67, openRocketHeight-25, openRocketVelocity+13);

        currSim.simulate(controller);
        
        currSim.writeRecord("SimRecords/simulation1.txt");
    }

    else if (operationMode == "Generate")
    {
        Generator trajectoryGenerator;
        trajectoryGenerator.generateTrajectories();
    }

    else if (operationMode == "Optimize")
    {
        GainOptimizer optimizer;
        //optimizer.evaluate();
        optimizer.findPertibationSolution();
    }

    else
    {
        cout << "Invalid value of operationMode used: " << operationMode << endl;
    }

    return 0;
}