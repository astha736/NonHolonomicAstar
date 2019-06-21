#include <param.h>


using namespace std;


namespace ecn
{
float Param::xTolerance = 1; //cm
float Param::yTolerance = 1;
float Param::thetaTolerance = M_PI/36; // degree

// for future use
float Param::scaleFactor = 1;

// can set the default values here
float Param::rWheel =1; // wheel radius
float Param::tGauge =1;

float Param::wheelVelocityMax = 5; // max angular velocity for any wheel
float Param::wheelVelocityMin = -5; // min angular velocity for any wheel
float Param::wheelVelocityTolerance = 0.5; // angular velocity tolerance to be used in comparison
float Param::velocityIncrementLimit = 2; // limit of +-rightWheelVel and leftWheelVel from current state
float Param::velocityIncrementStep = 0.5;


// time related elements
float Param::bigTimeStep = 2; // largest time step
int   Param::intervalCount = 4; // number of smaller intervals into which big time step is broken
float Param::obstacleCheckInterval = 0.2; // del-time step to prevent leaping over obstacles and printing
float Param::goalRadiusMultiplier = 0.6; // threshold multplier which triggers the largest timestep
float Param::highestThreshold = 12;

//    static  std::vector<float> intervalVector; // vector containing these smaller intervals

// heuristic related elements
float Param::force = 1;
float Param::moment = 2;
float Param::mass = 1;
float Param::inertia = 1;

float Param::straightSpinWeight = 1;
float Param::thetaSpinWeight = 1;

float Param::childHealthLimit = 1.05;

// set the required params in a function like this
// call this in RobotPose::setGoalPose()
void Param::loadStaticVariables(){
//    rWheel = 1; // wheel radius
//    tGauge = 1;
}

void Param::tuneHeuristicElements(float _lowerLimit, float _upperLimit){

}

void Param::tuneTimeElements(){

}

void Param::tuneVelocityElements(){

}
























} // end of cpp file
