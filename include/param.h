#ifndef PARAM_H
#define PARAM_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <maze.h>

namespace ecn
{

class Param
{
public:

    // tolerances along x,y and theta to be ignored whiel some comparison
    static float xTolerance; //cm
    static float yTolerance;
    static float thetaTolerance; // degree

    // for future use
    static float scaleFactor; // assumption( 1 pixel = scaleFactor x 1 cm : minimum step of the robot)

    static float rWheel; // wheel radius
    static float tGauge;

    // wheel velocity terms (rad/sec)
    static  float wheelVelocityMax; // max angular velocity for any wheel
    static  float wheelVelocityMin; // min angular velocity for any wheel
    static  float wheelVelocityTolerance; // angular velocity tolerance to be used in comparison
    static  float velocityIncrementLimit; // limit of +-rightWheelVel and leftWheelVel from current state

    // tunable
    static  float velocityIncrementStep;

    // tunable time related elements
    static  float bigTimeStep; // largest time step
    static  int intervalCount; // number of smaller intervals into which big time step is broken
    static  float obstacleCheckInterval; // del-time step to prevent leaping over obstacles and printing
    static  float goalRadiusMultiplier; // threshold multplier which triggers the largest timestep
    static  float highestThreshold;

    //    static  std::vector<float> intervalVector; // vector containing these smaller intervals

    // tunable heuristic related elements
    static  float force;
    static  float moment;
    static  float mass;
    static  float inertia;

    // currently being tuned
    static  float thetaSpinWeight;
    static  float straightSpinWeight;
    static float childHealthLimit; // used to reduce the number of child nodes to be explored by removing the badly performing children before exploration


    // methods
    static void loadStaticVariables();

//    // tuning methods
//    static void tuneHeuristicElements(float _lowerLimit, float _upperLimit);
//    static void tuneTimeElements();
//    static void tuneVelocityElements();















};

}

#endif // PARAM_H
