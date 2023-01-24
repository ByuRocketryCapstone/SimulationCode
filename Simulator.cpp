#include "Simulator.h"

// Constructor for Simulator objects. Takes in initial height, velocity, inclination angle, and sets
// a constant step size to initialize the simulation. Also reads in characteristics of the rocket
// from the parameters file.
Simulator::Simulator(double h0, double V0, double theta0)
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

    heightStep = 0.05;  //m
    currTime = 0;

    // record data for the rocket at MECO
    timeVals.push_back(0);
    heightVals.push_back(h);
    velocityVals.push_back(V);
    accelVals.push_back(-g);
    alphaVals.push_back(0);
}


Simulator::~Simulator()
{
    //writeRecord();
}


// Performs an energy balance for one height step. Output values are pass-by-reference parameters.
void Simulator::calcNextStep(double& hOut, double& VOut, double& aOut, double& tOut, double alpha)
{   
    double totalEnergy = m_r*g*h + 0.5*m_r*V*V; //calc total energy at current step
    double energyLoss = 0.5*getAirDensity(h)*V*V*(A_r*Cd_r +
        getPaddleDrag(alpha)) * heightStep; //calc energy loss due to drag (drag force*distance)
    totalEnergy -= energyLoss; 
    h += heightStep;
    double V_prev = V;

    if (totalEnergy > (m_r*g*h)) //check if rocket can make it another height step
    {
        V = sqrt(2*(totalEnergy - m_r*g*h)/m_r); //calculate new velocity after losses and height increase
    }
    else
    {
        h += 0.5*V*V/g; //convert last bit of velocity to height
        V = 0;
    }

    double timeStep = heightStep/V;     //calculate how much time it took to cross the height step
    double accel = (V - V_prev) / timeStep;     //numerical acceleration calculation
    if (V == 0) timeStep = heightStep/V_prev;   //makes sure last time stamp is not inf
    currTime += timeStep;

    //record rocket information
    timeVals.push_back(currTime);
    heightVals.push_back(h);
    velocityVals.push_back(V);
    accelVals.push_back(accel);
    alphaVals.push_back(alpha);

    //set output variables
    hOut = h;
    VOut = V;
    aOut = accel;
    tOut = currTime;
}


// Calculate air density as a function of height. 
// Data from https://www.engineeringtoolbox.com/air-altitude-density-volume-d_195.html
double Simulator::getAirDensity(double h)
{
    return 1.2 - 0.00012*(h+launchHeight); //kg/m^3
}


// Calculates frontal area and coefficient of drag of the paddles as a function of the 
// deployment angle. alpha is the paddle deployment angle in radians
double Simulator::getPaddleDrag(double alpha)
{
    double Cd_p = alpha * 0.8431;  
    double A_p = W_p * L_p * sin(alpha);

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
    A_r = M_PI*(D_r/2)*(D_r/2);     //calculate frontal area of just the rocket (no paddles)
    g = 9.80665;                    //acceleration of gravity (m/s^2)
}


// Writes the results of the simulation to a file whose directory is the fileSpec argument. 
// Recorded values are spaced out every 0.1 seconds to reduce data volume.
void Simulator::writeRecord(string fileSpec)
{
    //open output file stream 
    string filename;
    if (fileSpec == "") filename = RECORDS_DIRECTORY + to_string(time(0)) + ".txt";
    else filename = fileSpec;
    ofstream writer(filename);
    if(!writer.is_open())
    {
        cout << "Output file did not open in Simulator::writeRecord()." << endl;
        return;
    }

    //check that all information vectors are the same size
    if(!(timeVals.size() == heightVals.size() 
        && timeVals.size() == velocityVals.size()
        && timeVals.size() == alphaVals.size()
        && timeVals.size() == accelVals.size()))
    {
        cout << "Vectors not of same size in Simulator::writeRecord()." << endl;
        cout << "Time: " << timeVals.size() << endl;
        cout << "Height: " << heightVals.size() << endl;
        cout << "Vel: " << velocityVals.size() << endl;
        cout << "Accel: " << accelVals.size() << endl;
        cout << "Angle: " << alphaVals.size() << endl;
        return;
    }

    //setup code to retrieve date and time for the header
    time_t tt;
    struct tm * ti;
    time(&tt);
    ti = localtime(&tt);

    //space out generated data to reduce data volume
    vector<double> spacedTime, spacedHeight, spacedVelocity, spacedAccel, spacedAlpha;
    spacedTime.push_back(timeVals.at(0));
    spacedHeight.push_back(heightVals.at(0));
    spacedVelocity.push_back(velocityVals.at(0));
    spacedAccel.push_back(accelVals.at(0));
    spacedAlpha.push_back(alphaVals.at(0));

    double lastTime = timeVals.at(0);
    double timeInterval = 0.1;  //s
    for(int i = 0; i < timeVals.size(); i++)
    {
        if(timeVals.at(i) > lastTime+timeInterval)
        {
            spacedTime.push_back(timeVals.at(i));
            spacedHeight.push_back(heightVals.at(i));
            spacedVelocity.push_back(velocityVals.at(i));
            spacedAccel.push_back(accelVals.at(i));
            spacedAlpha.push_back(alphaVals.at(i));
            lastTime = timeVals.at(i);
        }

    }

    //write header and spaced information to output file
    writer << "Simulation created and run on: " << endl;
    writer << asctime(ti) << endl << endl;
    writer << "Time (s), Height (m), Velocity (m/s), Acceleration (m/s^2), Deployment Angle (degrees)" << endl;
    for(int i = 0; i < spacedTime.size(); i++)
    {
        writer << spacedTime.at(i) << ", " << spacedHeight.at(i) << ", " << spacedVelocity.at(i) << ", "
          << spacedAccel.at(i) << ", " << spacedAlpha.at(i) * (180/M_PI) << endl;
    }
}


// Returns height of the rocket at the end of the simulation
double Simulator::getApogee()
{
    return heightVals.at(heightVals.size()-1);
}


double Simulator::calcError(int refFileNum)
{
    double errorVal = 0;
    vector<double> refTimes, refHeights;
    ifstream reader(REF_DIRECTORY + REF_FILE_BASE + to_string(refFileNum) + ".txt");
    if(!reader.is_open()) cout << "File not opened in Simulator::calcError()" << endl;
    
    string line;
    for(int i = 0; i < REF_HEADER_SIZE; i++) getline(reader, line);  //skip header

    while(getline(reader, line))
    {
        vector<string> dataVals = split(line, ',');     //split line into individual values
        refTimes.push_back(stod(dataVals.at(0)));
        refHeights.push_back(stod(dataVals.at(1)));
    }

    double lastIndex = 0;
    for (int i = 0; i < refTimes.size(); i++)
    {
        for (int j = lastIndex; j < timeVals.size(); j++)
        {
            if (j != timeVals.size()-1 && timeVals.at(j+1) > refTimes.at(i))
            {
                lastIndex = j;
                break;
            }
        }
        errorVal += abs(refHeights.at(i) - heightVals.at(lastIndex)) / (refTimes.size() - i);
        if (lastIndex == timeVals.size()-1) break;
        
    }
    return errorVal;
}


std::vector<std::string> Simulator::split(const std::string& text, char delimiter)
{                                                                                                                                                                                             
   std::vector<std::string> splits;                                                                                                                                                           
   std::string split;                                                                                                                                                                         
   std::istringstream parser(text);                                                                                                                                                                  
   while (std::getline(parser, split, delimiter))
   {
      splits.push_back(split);
   }
   return splits;
}