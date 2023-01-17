#include "Controller.h" 

Controller::Controller(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    ref_alpha = 0;
    cmd_alpha = 0;

    loadData("SimRecords/datafile1.txt");   //FIXME: make file location not a string literal
}


void Controller::loadData(string datafile)
{
    ifstream reader(datafile);
    if(!reader.is_open())
    {
        cout << "File failed to open in Controller::loadData()." << endl;
    }

    string line;
    int headerSize = 5;
    for(int i = 0; i < headerSize; i++) getline(reader, line);  //skip header

    while(getline(reader, line))
    {
        vector<string> dataVals = split(line, ',');
        refTimes.push_back(stod(dataVals.at(0)));
        refHeights.push_back(stod(dataVals.at(1)));
        refVelocities.push_back(stod(dataVals.at(2)));
        refAccels.push_back(stod(dataVals.at(3)));
    }

    // for(int i = 0; i < 10; i++)
    // {
    //     cout << refTime.at(i) << ", ";
    //     cout << refHeight.at(i) << ", ";
    //     cout << refVelocity.at(i) << ", ";
    //     cout << refAccel.at(i) << endl;
    // }
}


double Controller::calcAngle(double currTime, double currHeight, double currVelocity, double currAccel)
{ 
    ref_alpha = getCurrentAngle();

    double error_h = currHeight - getRefHeight(currTime);
    double error_v = currVelocity - getRefVelocity(currTime);
    double error_a = currAccel - getRefAccel(currTime);

    //Actual PID Magic
    cmd_alpha = ref_alpha + (error_v * kp) + (error_h * ki) - (error_a * kd);

    //if it is being weird we will add anti windup scheme here

    //This is our saturation limits so we dont break things cause that would cause mucho problems
    if (cmd_alpha >= 70 * (M_PI/180)) cmd_alpha = 70 * (M_PI/180);
    else if (cmd_alpha <= 0) cmd_alpha = 0;
    else cmd_alpha = cmd_alpha;

    return cmd_alpha;
}


std::vector<std::string> Controller::split(const std::string& text, char delimiter)
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


double Controller::getCurrentAngle()
{
    return 0.0;
}


int Controller::findTimeIndex(double t)
{
    for (int i = 0; i < refTimes.size(); i++)
    {
        if(refTimes.at(i) > t) return t;
    }
    return -1;  //index was not found
}


double Controller::getRefHeight(double t)
{
    unsigned int upperBound = findTimeIndex(t);
    unsigned int lowerBound = upperBound - 1;
    double refHeight = refHeights.at(lowerBound) + 
        (t-refTimes.at(lowerBound)) * (refHeights.at(upperBound)-refHeights.at(lowerBound)) /
        (refTimes.at(upperBound)-refTimes.at(lowerBound));
    
    return refHeight;
}


double Controller::getRefVelocity(double t)
{
    unsigned int upperBound = findTimeIndex(t);
    unsigned int lowerBound = upperBound - 1;
    double refVelocity = refVelocities.at(lowerBound) + 
        (t-refTimes.at(lowerBound)) * (refVelocities.at(upperBound)-refVelocities.at(lowerBound)) /
        (refTimes.at(upperBound)-refTimes.at(lowerBound));
    
    return refVelocity;
}


double Controller::getRefAccel(double t)
{
    unsigned int upperBound = findTimeIndex(t);
    unsigned int lowerBound = upperBound - 1;
    double refAccel = refAccels.at(lowerBound) + 
        (t-refTimes.at(lowerBound)) * (refAccels.at(upperBound)-refAccels.at(lowerBound)) /
        (refTimes.at(upperBound)-refTimes.at(lowerBound));
    
    return refAccel;
}