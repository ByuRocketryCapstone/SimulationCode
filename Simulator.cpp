#include "Simulator.h"

// Constructor for Simulator objects. Takes in initial height (m), velocity (m/s), inclination angle (rad),
// and sets a constant step size to initialize the simulation. Also reads in characteristics of the rocket
// from the parameters file. The default parameter alpha0 represents a fixed paddle angle for use by the 
// Generator class to override the controller and specify a single paddle angle for the simulation.
// The initial velocity must be in the vertical direction only (inclination angle must be accounted for).
Simulator::Simulator(double h0, double V0, double alpha0)
{
    h = h0;     //m
    V = V0;     //m/s

    heightStep = 0.05;  //m
    currTime = t_c;
    fixedPaddleAngle = alpha0;

    // record data for the rocket at MECO
    timeVals.push_back(currTime);
    heightVals.push_back(h);
    velocityVals.push_back(V);
    accelVals.push_back(-g);
    alphaVals.push_back(0);
}


Simulator::~Simulator()
{
    //writeRecord();
}


void Simulator::simulate(Controller& controller)
{
    double currH, currV, currA, lastTime;
    double alpha, cmd_alpha;
    alpha = 0, cmd_alpha = 0, lastTime = currTime;
    do
    {
        calcNextStep(currH, currV, currA, currTime, alpha);
        
        if (fixedPaddleAngle == -1) cmd_alpha = controller.calcAngle(currTime, currH, currV, currA);
        else cmd_alpha = fixedPaddleAngle;
        
        // enforce actual paddle deployment limitations
        // using a constant rate defined by PADDLE_DEPLOYMENT_RATE to approximate actual non-linear rate
        if (cmd_alpha > alpha){
            //If the cmd angle is larger than the current angle ie the paddles need to open
            alpha += PADDLE_DEPLOYMENT_RATE * (currTime - lastTime);
        }
        else if (cmd_alpha < alpha){
            //If the cmd angle is less than the current angle ie the paddles need to close
            alpha += -(PADDLE_DEPLOYMENT_RATE * (currTime - lastTime));
        }
        lastTime = currTime;

        //Ensure that the commanded angle doesn't exceed the maximum possible angle or 0
        if (alpha >= MAX_PADDLE_ANGLE) alpha = MAX_PADDLE_ANGLE;
        else if (alpha <= 0) alpha = 0;
        else alpha = alpha;
        
    } while(currV > 0.1);
}


// Performs an energy balance for one height step. Output values are pass-by-reference parameters.
// Velocity and acceleration here are already corrected for inclination angle
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
    return 1.2 - 0.00012*(h+launchPadHeight); //kg/m^3
}


// Calculates frontal area and coefficient of drag of the paddles as a function of the 
// deployment angle. alpha is the paddle deployment angle in radians.
// The 0.8431 is the slope of the linear fit of the wind tunnel drag data from GEN-111
double Simulator::getPaddleDrag(double alpha)
{
    double Cd_p = alpha * 0.8431;
    double A_p = W_p * L_p * sin(alpha);

    return Cd_p * A_p;
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
        writer << spacedTime.at(i) << " " << spacedHeight.at(i) << " " << spacedVelocity.at(i) << " "
          << spacedAccel.at(i) << " " << spacedAlpha.at(i) * (180/M_PI) << endl;
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
        double t1, h1, V1, a1;
        stringstream parser(line);
        parser >> t1 >> h1 >> V1 >> a1;
        refTimes.push_back(t1);
        refHeights.push_back(h1);
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


// Splits a string of values according to the delimiter character
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


// Resets the simulation to the beginning so it can be run again
void Simulator::reset(double h0, double V0, double alpha0)
{
    h = h0;     //m
    V = V0;     //m/s

    heightStep = 0.05;  //m
    currTime = t_c;
    fixedPaddleAngle = alpha0;

    timeVals.clear();
    heightVals.clear();
    velocityVals.clear();
    accelVals.clear();
    alphaVals.clear();

    // record data for the rocket at MECO
    timeVals.push_back(currTime);
    heightVals.push_back(h);
    velocityVals.push_back(V);
    accelVals.push_back(-g);
    alphaVals.push_back(0);
}