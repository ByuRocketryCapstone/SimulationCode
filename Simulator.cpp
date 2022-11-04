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
    currTime = 0;
}


Simulator::~Simulator()
{
    writeRecord();
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
    currTime += timeStep;

    timeVals.push_back(currTime);
    heightVals.push_back(h);
    velocityVals.push_back(V);
    alphaVals.push_back(alpha);

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


void Simulator::writeRecord()
{
    
    string filename = RECORDS_DIRECTORY + to_string(time(0)) + ".txt";
    ofstream writer(filename);
    if(!writer.is_open())
    {
        cout << "Output file did not open in Simulator::writeRecord()." << endl;
        return;
    }

    if(!(timeVals.size() == heightVals.size() 
        && timeVals.size() == velocityVals.size()
        && timeVals.size() == alphaVals.size()))
    {
        cout << "Vectors not of same size in Simulator::writeRecord()." << endl;
        cout << "Time: " << timeVals.size() << endl;
        cout << "Height: " << heightVals.size() << endl;
        cout << "Vel: " << velocityVals.size() << endl;
        cout << "Angle: " << alphaVals.size() << endl;
        return;
    }

    time_t tt;
    struct tm * ti;
    time(&tt);
    ti = localtime(&tt);


    writer << "Simulation created and run on: " << endl;
    writer << asctime(ti) << endl << endl;
    writer << "Time (s), Height (m), Velocity (m/s), Deployment Angle (radians)" << endl;
    for(int i = 0; i < timeVals.size(); i++)
    {
        writer << timeVals.at(i) << ", " << heightVals.at(i) << ", " << velocityVals.at(i) << ", "
            << alphaVals.at(i) << endl;
    }
}