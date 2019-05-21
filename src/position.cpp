#include <position.h>

namespace ecn
{

float Position::x_tol = 0; //cm
float Position::y_tol = 0;
float Position::theta_tol = 0; // degree
float Position::scale_factor = 1; // 1 pixel = 1 cm : minimum step of the robot

}
// TODO: write this print correctly
// int Position::distToParent()
// {
//     // in cell-based motion, the distance to the parent is always 1
//     return 1;
// }

// TODO: write this print correctly
// std::vector<Position::PositionPtr> Position::children()
// {
//     // this method should return  all positions reachable from this one
//     std::vector<Position::PositionPtr> generated;

//     return generated;
// }

// TODO: write this print correctly
//void Position::print(const Position &parent)
//{
//        int x_incr(0), y_incr(0);
//        // may need to change incase we assume anything other than 1 pix <=> 1 cm

//        if(x - parent.x)
//            x_incr = x - parent.x > 0 ? 1 : -1;
//        else
//            y_incr = y - parent.y > 0 ? 1 : -1;
//        int k = 1;
//        while(parent.x + k*x_incr != x || parent.y + k*y_incr != y)
//        {
//            maze.passThrough(parent.x + k*x_incr,
//                             parent.y + k*y_incr);
//            k++;
//        }

//    maze.passThrough(x, y);

//}

// used in the maze function, have to check and update 
// TODO: write this print correctly
// void Position::start()
// {
// }

// online print, color depends on closed / open set
// TODO: write this print correctly

//void Position::show(bool closed, const Position & parent)
//{
//}

// // TODO: these should set some static variables 
// Position Position::begin(){

// }

// // TODO: these should set some static variables 
// Position Position::end(){

// }
