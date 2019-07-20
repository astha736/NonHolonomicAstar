#include <tune.h>
#include <robotPose.h>


using namespace std;


namespace ecn
{

float Tune::resultTuneHeuristic = FLT_MAX;
bool  Tune::validResultSet = false;

void Tune::tuneHeuristicElements(float _lowerLimit, float _upperLimit){

}

void Tune::tuneTimeElements(){

}

void Tune::tuneVelocityElements(){

}

std::vector<pair<float, float>> Tune::generateTuningSet(){
    std::vector<pair<float, float>> output;
    float thetaIndex = Tune::weightRangeMin*10;
    while(thetaIndex <= Tune::weightRangeMax*10){
        float straightIndex = Tune::weightRangeMin*10;
        while(straightIndex <=  Tune::weightRangeMax*10){
            output.push_back(make_pair(thetaIndex/10,straightIndex/10));
            straightIndex += Tune::weightStep*10;
        }
        thetaIndex += Tune::weightStep*10;
    }
    return output;
}

























} // end of cpp file
