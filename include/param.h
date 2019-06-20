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
    static  float velocityIncrementStep;
    static  float obstacleCheckInterval; // del-time step to prevent leaping over obstacles and printing


    // time related elements
    static  float bigTimeStep; // largest time step
    static  int intervalCount; // number of smaller intervals into which big time step is broken
//    static  std::vector<float> intervalVector; // vector containing these smaller intervals

    // heuristic related elements
    static  float force;
    static  float moment;
    static  float mass;
    static  float inertia;

    static void loadStaticVariables();

};

}

#endif // PARAM_H
