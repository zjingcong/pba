//
// Created by jingcoz on 12/3/17.
//

#ifndef PBA_ADVECTBYVOLUME_H
#define PBA_ADVECTBYVOLUME_H

# include "Types.h"
# include "DynamicalState.h"
# include <openvdb/tools/Interpolation.h>

namespace pba
{

    class AdvectByVolume
    {
    public:
        static void advect_by_volume(DynamicalState DS, Vec3fGrid::Ptr volume);

        static Vector& get_grid_value(const Vector& pos, Vec3fGrid::Ptr volume);
        static void loop_grid(Vec3fGrid::Ptr volume);
        static void loop_floatgrid(FloatGrid::Ptr volume);
    private:
    };

}

#endif //PBA_ADVECTBYVOLUME_H
