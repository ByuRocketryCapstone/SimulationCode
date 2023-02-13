#ifndef CONSTS_H
#define CONSTS_H

#include <string>
#include <cmath>

const std::string REF_DIRECTORY = "SimRecords/References/";
const std::string REF_FILE_BASE = "refData";
const std::string INDEX_FILE_NAME = "index.txt";
const int REF_HEADER_SIZE = 5;

const double TARGET_APOGEE = 3048;      //m
const double PADDLE_DEPLOYMENT_RATE = 14*(M_PI/180);    //rad/s

#endif //CONSTS_H