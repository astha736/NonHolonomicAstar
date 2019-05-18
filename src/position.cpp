#include <position.h>

namespace ecn
{

int Position::distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return 1;
    }


std::vector<Position::PositionPtr> Position::children()
    {
        // this method should return  all positions reachable from this one
        std::vector<Position::PositionPtr> generated;

        return generated;
    }
}