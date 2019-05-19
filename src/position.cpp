#include <position.h>

namespace ecn
{
// TODO: write this print correctly
int Position::distToParent()
{
    // in cell-based motion, the distance to the parent is always 1
    return 1;
}

// TODO: write this print correctly
std::vector<Position::PositionPtr> Position::children()
{
    // this method should return  all positions reachable from this one
    std::vector<Position::PositionPtr> generated;

    return generated;
}

// TODO: write this print correctly
void Position::print(const Position &parent)
{
}

// used in the maze function, have to check and update 
// TODO: write this print correctly
void Position::start()
{
}

// online print, color depends on closed / open set
// TODO: write this print correctly
void Position::show(bool closed, const Position & parent)
{
}

// TODO: these should set some static variables 
Position Position::begin(){
}

// TODO: these should set some static variables 
Position Position::end()
{
}

}