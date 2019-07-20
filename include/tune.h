#ifndef TUNE_H
#define TUNE_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <maze.h>
#include <param.h>

#include <robotPose.h>
namespace ecn
{

class Tune
{
public:
    friend class Param;
    // used for tuning
    static constexpr float weightRangeMax = 1.0;
    static constexpr float weightRangeMin = 0;
    static constexpr float weightStep = 0.10;

    static float resultTuneHeuristic;
    static bool  validResultSet;

    // tuning methods
    static void tuneHeuristicElements(float _lowerLimit, float _upperLimit);
    static void tuneTimeElements();
    static void tuneVelocityElements();


    static std::vector<pair<float, float>> generateTuningSet();

    template<class T>
    static void setTuningResults(T child, T start, T goal){
           resultTuneHeuristic = child.euclideanDist(goal);
           validResultSet = true;
         }

};

}

#endif // PARAM_H
