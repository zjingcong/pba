//
// Created by jingcoz on 11/22/17.
//

# include "SphereState.h"
# include <iostream>

using namespace pba;
using namespace std;

SphereStateData::SphereStateData(const std::string &nam)
{
    float_attributes["radius"] = DSAttribute<float>( "radius", 1.0 );
    int_attributes["isCollision"] = DSAttribute<int>("isCollision", 0);
    re_find_main_attrs();
}

const float& SphereStateData::radius(const size_t p) const
{
    return radiuses->second.get(p);
}

const int& SphereStateData::isCollision(const size_t p) const
{
    return collision_flags->second.get(p);
}

void SphereStateData::set_radius(const size_t p, const float &r)
{
    radiuses->second.set(p, r);
}

void SphereStateData::set_isCollision(const size_t p, const int &flag)
{
    collision_flags->second.set(p, flag);
}

void SphereStateData::clean_collision_flags()
{
    for (size_t i = 0; i < nb_items; ++i)
    {
        collision_flags->second.set(i, 0);
    }
}


void SphereStateData::re_find_main_attrs()
{
    DynamicalStateData::re_find_main_attrs();
    radiuses = float_attributes.find( "radius" );
    if( radiuses == float_attributes.end() )
    { std::cout << "ERROR could not find radiuses" << std::endl; }
    collision_flags = int_attributes.find("isCollision");
    if( collision_flags == int_attributes.end() )
    { std::cout << "ERROR could not find collision_flags" << std::endl; }
}


pba::SphereState pba::CreateSphereState(const std::string &nam)
{
    return pba::SphereState(new SphereStateData(nam));
}
