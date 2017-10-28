//
// Created by jingcoz on 10/28/17.
//

# include "SoftBodyState.h"

using namespace std;
using namespace pba;

SoftBodyStateData::SoftBodyStateData(const std::string &nam): DynamicalStateData(nam)
{
    vector_attributes["inner"] = DSAttribute<Vector>( "inner", Vector(0,0,0) );   // struct force
    parms = {{"Ks", 0.0},
             {"Kf", 0.0}};
}

void SoftBodyStateData::Init(const std::vector<Vector>& verts)
{
    add(verts.size());
    // init mass
    for (size_t i = 0; i < nb_items; ++i)
    {
        double mass = 1.0;
        set_mass(i, float(mass));
        set_pos(i, verts[i]);
    }
}

void SoftBodyStateData::Reset(const std::vector<Vector> &verts)
{
    for (size_t i = 0; i < nb_items; ++i)
    {
        double mass = 1.0;
        set_mass(i, float(mass));
        set_pos(i, verts[i]);
        set_vel(i, Vector(0.0, 0.0, 0.0));
        set_attr("inner", i, Vector(0.0, 0.0, 0.0));
    }
}

void SoftBodyStateData::set_softEdges(const std::vector<std::pair<size_t, size_t >> &pairs)
{
    connected_pairs.clear();
    for (auto& it: pairs)
    {
        SoftEdge e;
        e.i = it.first;
        e.j = it.second;
        e.L = (pos(e.i) - pos(e.j)).magnitude();
        connected_pairs.push_back(e);
    }
}

const Vector SoftBodyStateData::innerForce(const size_t& p)
{
    return get_vector_attr("inner", p);
}

void SoftBodyStateData::update_innerForce()
{
    for (auto& it: connected_pairs)
    {
        size_t i = it.i;
        size_t j = it.j;
        Vector structForce = get_structForce(i, j, it.L);
        set_attr("inner", i, innerForce(i) + structForce);
        set_attr("inner", j, innerForce(j) - structForce);
    }
}


Vector SoftBodyStateData::get_structForce(const size_t& i, const size_t& j, const double& L)
{
    Vector d_ij = (pos(j) - pos(i)).unitvector();
    // spring force
    Vector spring = parms.at("Ks") * ((pos(i) - pos(j)).magnitude() - L) * d_ij;
    // friction force
    Vector friction = parms.at("Kf") * d_ij * (d_ij * (vel(j) - vel(i)));

    return spring + friction;
}


pba::SoftBodyState pba::CreateSoftBodyState(const std::string &nam)
{
    return pba::SoftBodyState(new SoftBodyStateData(nam));
}
