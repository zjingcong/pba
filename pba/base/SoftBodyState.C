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
             {"Kf", 0.0},
             {"As", 0.0},
             {"Af", 0.0}};
}

void SoftBodyStateData::Init(const std::vector<Vector>& vertices)
{
    add(vertices.size());
    // init mass
    for (size_t i = 0; i < nb_items; ++i)
    {
        double mass = 1.0;
        set_mass(i, float(mass));
        set_pos(i, vertices[i]);
        verts.push_back(vertices[i]);
    }
}

void SoftBodyStateData::Reset()
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

void SoftBodyStateData::add_softEdges(size_t i, size_t j)
{
    SoftEdge e;
    e.i = i;
    e.j = j;
    e.L = (pos(i) - pos(j)).magnitude();
    connected_pairs.push_back(e);
}

void SoftBodyStateData::add_softTriangles(size_t i, size_t j, size_t k)
{
    SoftTriangle t;
    t.i = i;
    t.j = j;
    t.k = k;
    t.Area = 0.5 * ((pos(j) - pos(i)) ^ (pos(k) - pos(i))).magnitude();
    triangle_areas.push_back(t);
}

const Vector SoftBodyStateData::innerForce(const size_t& p)
{
    return get_vector_attr("inner", p);
}

// calculate inner force for all the particles at one time
void SoftBodyStateData::Update()
{
    /// reset inner force
    for (size_t l = 0; l < nb_items; ++l) { set_attr("inner", l, Vector(0.0, 0.0, 0.0)); }
    /// calculate inner force
    // struct force
    for (auto& softE: connected_pairs)
    {
        size_t i = softE.i;
        size_t j = softE.j;
        Vector structForce = get_structForce(softE);
        set_attr("inner", i, innerForce(i) + structForce);
        set_attr("inner", j, innerForce(j) - structForce);
    }
    // area force
    for (auto& softT: triangle_areas)
    {
        size_t i = softT.i;
        size_t j = softT.j;
        size_t k = softT.k;
        Vector f0, f1, f2;
        get_areaForce(softT, f0, f1, f2);
        set_attr("inner", i, innerForce(i) + f0);
        set_attr("inner", j, innerForce(j) + f1);
        set_attr("inner", k, innerForce(k) + f2);
    }
    // bending force
}


Vector SoftBodyStateData::get_structForce(const SoftEdge& e)
{
    size_t i = e.i;
    size_t j = e.j;
    double L = e.L;
    Vector d_ij = (pos(j) - pos(i)).unitvector();
    // spring force
    Vector spring = parms.at("Ks") * ((pos(i) - pos(j)).magnitude() - L) * d_ij;
    // friction force
    Vector friction = parms.at("Kf") * d_ij * (d_ij * (vel(j) - vel(i)));

    return spring + friction;
}

void SoftBodyStateData::get_areaForce(const SoftTriangle& t, Vector& f0, Vector& f1, Vector& f2)
{
    size_t i = t.i;
    size_t j = t.j;
    size_t k = t.k;
    double A0 = t.Area;
    Vector x0 = pos(i);
    Vector x1 = pos(j);
    Vector x2 = pos(k);
    Vector d0 = (x0 - 0.5 * (x1 + x2)).unitvector();
    Vector d1 = (x1 - 0.5 * (x0 + x2)).unitvector();
    Vector d2 = (x2 - 0.5 * (x0 + x1)).unitvector();
    double A = 0.5 * ((x1 - x0) ^ (x2 - x0)).magnitude();
    Vector v0 = vel(i);
    Vector v1 = vel(j);
    Vector v2 = vel(k);
    Vector V0 = v0 - 0.5 * (v1 + v2);
    Vector V1 = v1 - 0.5 * (v0 + v2);
    Vector V2 = v2 - 0.5 * (v0 + v1);
    float As = parms.at("As");
    float Af = parms.at("Af");

    f0 = -As * (A - A0) * d0 - Af * d0 * (d0 * V0);
    f1 = -As * (A - A0) * d1 - Af * d1 * (d1 * V1);
    f2 = -As * (A - A0) * d2 - Af * d2 * (d2 * V2);
}


pba::SoftBodyState pba::CreateSoftBodyState(const std::string &nam)
{
    return pba::SoftBodyState(new SoftBodyStateData(nam));
}
