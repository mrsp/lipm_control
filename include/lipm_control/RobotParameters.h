#ifndef ROBOTPARAMETERS_H_
#define ROBOTPARAMETERS_H_
#include <string>
#include <iostream>
#include <cmath>
#include <stdio.h>

#define MAX_TRAJECTORY_LENGTH 250

enum
{
    Ts = 0,  Td, ComZ,  g, mass, omega, StepXF, StepXH, StepYL, StepYR,
    Tc, Ta, Tn, Kc, Ka, Kn, ParametersSize
};

static const char * robotParameterNames[] =
{
    "Ts",  "Td", "ComZ",  "g", "mass", "omega", "StepXF", "StepXH", "StepYL", "StepYR",
    "Tc", "Ta", "Tn", "Kc", "Ka", "Kn", "ParametersSize"
};


static const char defaultFilenameForParameters[]="/home/nao/KWalkRobotParameters.ini";


class RobotParameters
{
    
private:
    float WalkParameters[ParametersSize];
    char robotName[256];
    
public:
    RobotParameters()
    {
        snprintf(robotName,256,"default");
        WalkParameters[Ts] = 0.005;
        WalkParameters[Td] = 0.05;
        WalkParameters[ComZ] = 1.14398;
        WalkParameters[g] = 9.80665;
        WalkParameters[mass]= 174.25; 
        WalkParameters[omega]=  sqrt(WalkParameters[g]/WalkParameters[ComZ]);
        WalkParameters[StepXF] = 0.172;
        WalkParameters[StepXH] = -0.090;
        WalkParameters[StepYL] = 0.04;
        WalkParameters[StepYR] = -0.04;
        WalkParameters[Tc] = 0.1; 
        WalkParameters[Ta] = 0.01;  
        WalkParameters[Tn] = 0.1; 
        WalkParameters[Kc] = 2.5;   
        WalkParameters[Ka] = 0.0005; 
        WalkParameters[Kn] = 0.005; 
    }
    
    float getWalkParameter(int);
    void  setWalkParameter(int p, float s);
    int writeWalkParametersFromFile(const char * filename);
    int readWalkParametersFromFile(const char * filename);
    void printWalkParameters();
};
#endif
