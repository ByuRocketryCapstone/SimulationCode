#ifndef CONSTS_H
#define CONSTS_H

#include <string>
#include <cmath>

const std::string REF_DIRECTORY = "SimRecords/References/";
const std::string REF_FILE_BASE = "refData";
const std::string INDEX_FILE_NAME = "index.txt";
const int REF_HEADER_SIZE = 5;

const double TARGET_APOGEE = 3048;      //m
const double PADDLE_DEPLOYMENT_RATE = 14 * (M_PI/180);    //rad/s
const double MAX_PADDLE_ANGLE = 65 * (M_PI/180);    //rad

const double m_r = 15.522;            //rocket dry mass, kg
const double Cd_r = 0.3959;            //drag coefficient of rocket with no paddles
const double D_r = 0.156;              //rocket body diameter, meters
const double L_p = 0.1;                 //paddle length, meters
const double W_p = 0.1;                 //paddle width, meters
const double launchPadHeight = 1293;    //height of the launchpad above sealevel, meters
const double A_r = M_PI*(D_r/2)*(D_r/2);    //frontal area of the rocket, m^2
const double g = 9.80665;               //acceleration of gravity, m/s^2
const double t_c = 3.6;

const double mecoHeight = 679.84;     //height of the rocket at MECO obtained from OpenRocket, meters
const double mecoVelocity = 304.148;     //velocity of the rocket at MECO obtained from OpenRocket, m/s

#endif //CONSTS_H