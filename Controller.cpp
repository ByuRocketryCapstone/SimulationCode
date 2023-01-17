#include "Controller.h"

Controller::Controller(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
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
        refTime.push_back(stod(dataVals.at(0)));
        refHeight.push_back(stod(dataVals.at(1)));
        refVelocity.push_back(stod(dataVals.at(2)));
        refAccel.push_back(stod(dataVals.at(3)));
    }

    // for(int i = 0; i < 10; i++)
    // {
    //     cout << refTime.at(i) << ", ";
    //     cout << refHeight.at(i) << ", ";
    //     cout << refVelocity.at(i) << ", ";
    //     cout << refAccel.at(i) << endl;
    // }
}


double Controller::calcAngle()
{
    return 0.0;
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