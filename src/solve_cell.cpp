#include <a_star.h>
#include <maze.h>


#include <robotPose.h>
#include <position.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
// white -> free
// black -> blocked

int main( int argc, char **argv )
{

    float x_begin = 1, y_begin = 1, thetaBegin = 0;
    float x_end = 36,  y_end = 21,  thetaEnd = M_PI/2 ;

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

    //    cout << Point::maze.isFree(x_end,y_end) << std::endl;

    ecn::Astar(beginRobotPose, endRobotPose);
//    ecn::Astar(beginRobotPose, endRobotPoseMaze);

    // save final image
     Point::maze.saveSolution("maze_test");
    cv::waitKey(0);

}
