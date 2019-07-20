#include <a_star.h>
#include <maze.h>


#include <robotPose.h>
#include <position.h>
#include <param.h>
#include <tune.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
// white -> free
// black -> blocked

int main( int argc, char **argv )
{

    float x_begin = 1, y_begin = 1, thetaBegin = 0;
    float x_end = 40,  y_end = 2,  thetaEnd = M_PI/2;

    float rightVelBegin = 0, rightVelEnd = 0;
    float leftVelBegin = 0, leftVelEnd = 0;

    // load file
    std::string filename = "maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Point::maze.load(filename);
//    cout<<Point::begin().x<<endl;
//    cout<<Point::end().x<<endl;


    Position beginPosition(x_begin, y_begin, thetaBegin);
    Position endPosition(x_end, y_end, thetaEnd);
    Position endPositionMaze(Point::end().x,Point::end().y,M_PI/2);

    RobotPose beginRobotPose(rightVelBegin,leftVelBegin, beginPosition);
    RobotPose endRobotPose(rightVelEnd,leftVelEnd, endPosition);
    RobotPose endRobotPoseMaze(rightVelEnd,leftVelEnd, endPositionMaze);


   endRobotPose.setGoalPose(endRobotPose);
    // endRobotPoseMaze.setGoalPose(endRobotPoseMaze);

    cout << "beginRobotPose: " << beginRobotPose << std::endl;
    cout << "endRobotPose: " << endRobotPose << std::endl;

    // call A* algorithm
    if(!beginRobotPose.isFree()){
        cout<<"wrong begin point"<<endl;
        return 0;
    }
    if(!endRobotPose.isFree()){
        cout<<"wrong end point"<<endl;
        return 0;
    }

///////////////////////// run normal A* ////////////////////////////
    ecn::Astar(beginRobotPose, endRobotPose);

    // save final image
    Point::maze.saveSolution("smalltest");
    cv::waitKey(0);

//////////////////////////////// Code to tune the parameters /////////////////////////////////////////
//        // step1. for a give start and end make a list of set parameters
//        std::vector<pair<float, float>> paramSetList =  Tune::generateTuningSet();
//        std::vector<float> heuristicSetList;
//        // step2. for each element in the set, run the Astar with true values
//        for(int i =0; i < paramSetList.size(); i++){
//            std::cout << "generateTuningset: " << i << " : " << paramSetList[i].first << " : " << paramSetList[i].second << std::endl;
//            Param::thetaSpinWeight = paramSetList[i].first;
//            Param::straightSpinWeight = paramSetList[i].second;
//            ecn::Astar(beginRobotPose, endRobotPose, true);
//            if(Tune::validResultSet == true){
//                heuristicSetList.push_back(Tune::resultTuneHeuristic);
//                Tune::validResultSet = false;
//            }
//            else{
//                std::cout << "Some error Param::resultTuneHeuristic = FLT_MAX" << std::endl;
//            }

//        }
//        // step3. based on the list of heuristic collected, print the best choices
//        float minHeuristic = FLT_MAX;
//        int minHIndex = heuristicSetList.size();
//        for(int i =0; i < heuristicSetList.size(); i++){
//            std::cout << "generateTuningset: " << paramSetList[i].first << " : " << paramSetList[i].second << std::endl;
//            std::cout << "EuclideanDist " << heuristicSetList[i] << std::endl;
//            if(minHeuristic > heuristicSetList[i]){
//                minHeuristic = heuristicSetList[i];
//                minHIndex = i;
//            }
//        }

////        std::cout << "********************* Finally ***************" << std::endl;
////        std::cout << "weight theta: " << paramSetList[minHIndex].first << std::endl;
////        std::cout << "weight straight " << paramSetList[minHIndex].second << std::endl;
////        std::cout << "minHeuristic " << heuristicSetList[minHIndex] << std::endl;

}
