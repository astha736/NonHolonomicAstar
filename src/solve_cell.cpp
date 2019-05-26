#include <a_star.h>
#include <maze.h>


#include <robotPose.h>
#include <position.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time

int main( int argc, char **argv )
{
//    int x_begin = 0;
    int x_end = 10;

//    int y_begin = 0;
    int y_end = 0;

    // let theta be between 0 to 360
    int thetaBegin = 0;
    int thetaEnd = 0;

    int rightVelBegin = 0;
    int leftVelBegin = 0;

    int rightVelEnd = 0;
    int leftVelEnd = 0;

    // load file
    std::string filename = "maze_basic.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Point::maze.load(filename);

    Point beginMaze =  Point::begin();
    Point endMaze =  Point::end();
    cout << "begin Maze: " << beginMaze << std::endl;
    cout << "end Maze: " << endMaze << std::endl;
    // initial and end positions as Position's

    Position beginPosition(beginMaze,thetaBegin);
    Position endPosition(endMaze,thetaEnd);
//    Position endPosition(x_end, y_end, thetaEnd);

    //    Position beginPosition(x_begin,y_begin,thetaBegin);
    //    Position endPosition(x_end, y_end, thetaBegin);

    cout << "begin Position: " << beginPosition << std::endl;
    cout << "end Position: " << endPosition << std::endl;

    RobotPose beginRobotPose(rightVelBegin,leftVelBegin, beginPosition);
    RobotPose endRobotPose(rightVelEnd,leftVelEnd, endPosition);

    cout << "beginRobotPose: " << beginRobotPose << std::endl;
    cout << "endRobotPose: " << endRobotPose << std::endl;


//    RobotPose p1(1,1,10.2,0.2,0.1);
//    cout << "p1:  " << p1 << endl;
//    cout  << "end: " << endRobotPose << endl;
//    cout << p1.is(endRobotPose) << endl;

    // // call A* algorithm
    ecn::Astar(beginRobotPose, endRobotPose);

    // save final image
    // Point::maze.saveSolution("cell");
    cv::waitKey(0);

}
