//
// Created by jingcoz on 12/3/17.
//

# include "AdvectByVolume.h"

using namespace pba;
using namespace std;

void AdvectByVolume::advect_by_volume(DynamicalState DS, Vec3fGrid::Ptr volume)
{
    for (size_t i = 0; i < DS->nb(); ++i)
    {
        Vector pos = DS->pos(i);
        Vector volume_vel = get_grid_value(pos, volume);
        DS->set_vel(i, volume_vel);
    }
}

Vector& AdvectByVolume::get_grid_value(const Vector& pos, Vec3fGrid::Ptr volume)
{
    Vec3s xyz(float(pos.X()), float(pos.Y()), float(pos.Z()));	// world space
    openvdb::tools::GridSampler<Vec3fGrid, openvdb::tools::BoxSampler> sampler(*volume);
    Vec3s gridValue;
    gridValue = sampler.wsSample(xyz);	// world space sample
    Vector vecValue(gridValue.x(), gridValue.y(), gridValue.z());

    return vecValue;
}

void AdvectByVolume::loop_grid(Vec3fGrid::Ptr volume)
{
    Vec3fGrid::Accessor accessor = volume->getAccessor();
    Transform::Ptr transform = volume->transformPtr();
    for(Vec3fGrid::ValueOnIter iter = volume->beginValueOn(); iter; ++iter)
    {
        Coord ijk = iter.getCoord();
        Vec3s world = transform->indexToWorld(ijk);
        std::cout << "Grid: " << ijk << " " << world << " = " << *iter << std::endl;
    }
}

void AdvectByVolume::loop_floatgrid(FloatGrid::Ptr volume)
{
    FloatGrid::Accessor accessor = volume->getAccessor();
    Transform::Ptr transform = volume->transformPtr();
    for(FloatGrid::ValueOnIter iter = volume->beginValueOn(); iter; ++iter)
    {
        Coord ijk = iter.getCoord();
        Vec3s world = transform->indexToWorld(ijk);
        std::cout << "Grid: " << ijk << " " << world << " = " << *iter << std::endl;
    }
}
