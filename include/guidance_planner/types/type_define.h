
#ifndef TYPE_DEFINE_H
#define TYPE_DEFINE_H

namespace GuidancePlanner
{
    template <int T, int P> // T = total number of states minus time, P = total number of positions
    struct SpaceTimePointDim;
    typedef SpaceTimePointDim<2, 2> SpaceTimePoint; // 2D position
    // typedef SpaceTimePointDim<3, 2> SpaceTimePoint; // Orientation + 2D position
}

#endif // TYPE_DEFINE_H
