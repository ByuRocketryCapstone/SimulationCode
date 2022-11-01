#include "Simulator.h"

// Constructor for Simulator objects. Takes in initial height, velocity, inclination angle, and
// constant step size to initialize the simulation. Also reads in characteristics of the rocket
// from the parameters file.
Simulator::Simulator(double h0, double V0, double theta0, double stepSize)
{
    ifstream reader(PARAMETERS_FILE);
    if(!reader.is_open()) 
    {
        cout << "Parameters file not opened in Simulator::Simulator()." << endl;
        return;
    }

    populateParameters(reader);
    h = h0;
    V = V0;
    theta = theta0;

    heightStep = stepSize;  //m
}


// Performs an energy balance for one height step. Output values are pass-by-reference parameters.
void Simulator::calcNextStep(double& hOut, double& VOut, double& aOut, double alpha)
{   
    double totalEnergy = m_r*g*h + 0.5*m_r*V*V; //calc total energy at current step
    double energyLoss = 0.5*getAirDensity(h)*V*V*(A_r*Cd_r +
        getPaddleDrag(alpha)) * heightStep; //calc energy loss
    totalEnergy -= energyLoss; 
    h += heightStep;
    double V_prev = V;

    if (totalEnergy > (m_r*g*h)) //check if rocket can make it another height step
    {
        V = sqrt(2*(totalEnergy - m_r*g*h)/m_r); //calc new velocity
    }
    else
    {
        h += 0.5*g*V*V; //convert last bit of velocity to height
        V = 0;
    }

    double timeStep = heightStep/V;
    double accel = (V - V_prev) / timeStep;

    hOut = h;
    VOut = V;
    aOut = accel;

}


// Calculate air density as a function of height. 
// Data from https://www.engineeringtoolbox.com/air-altitude-density-volume-d_195.html
double Simulator::getAirDensity(double h)
{
    return 1.2 - 0.00012*(h+launchHeight); //kg/m^3
}


// Calculates frontal area and coefficient of drag of the paddles as a function of the 
// deployment angle. alpha is the paddle deployment in degrees
double Simulator::getPaddleDrag(double alpha)
{
    double Cd_p = 0;
    double A_p = W_p * L_p * sin(alpha * (M_PI/180.0));

    return Cd_p * A_p;
}


// Reads in parameters of the rocket from the parameters file. Additional parameters can be
// added by adding to the if/else block.
void Simulator::populateParameters(ifstream& reader)
{
    string line;
    while(getline(reader, line))
    {
        stringstream parser(line);
        string parameterName;
        parser >> parameterName;

        if (parameterName == "rocketDryMass") parser >> m_r;
        else if (parameterName == "rocketDragCoefficient") parser >> Cd_r;
        else if (parameterName == "rocketDiameter") parser >> D_r;
        else if (parameterName == "paddleLength") parser >> L_p;
        else if (parameterName == "paddleWidth") parser >> W_p;
        else if (parameterName == "launchPadHeight") parser >> launchHeight;
    } 
    A_r = M_PI*(D_r/2)*(D_r/2);
    g = 9.80665;
}