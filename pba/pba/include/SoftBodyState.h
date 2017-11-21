//
// Created by jingcoz on 10/28/17.
//

#ifndef PBA_SOFTBODYSTATE_H
#define PBA_SOFTBODYSTATE_H

# include "DynamicalState.h"

namespace pba
{
    struct SoftEdge
    {
        size_t i;
        size_t j;
        double L;
    };

    struct SoftTriangle
    {
        size_t i;   // P0
        size_t j;   // P1
        size_t k;   // P2
        double Area;
    };

    class SoftBodyStateData: public DynamicalStateData
    {
    public:
        SoftBodyStateData(const std::string& nam = "SoftBodyDataNoName");
        ~SoftBodyStateData()    {}

        void Init(const std::vector<Vector>& vertices);
        void Reset();
        void Update();

        void add_softEdges(size_t i, size_t j);
        void add_softTriangles(size_t i, size_t j, size_t k);

        void update_parms(const std::string &key, const float &value)  {parms.at(key) = value;}

        const Vector innerForce(const size_t& p);
        const float get_parms(const std::string &key) const {return parms.at(key);}
        const std::vector<SoftEdge>& get_connectedPairs() { return connected_pairs;}
        const std::vector<SoftTriangle>& get_triangleAreas()  { return triangle_areas;}

    private:
        std::vector<Vector> verts;  // init vertices
        std::vector<SoftEdge> connected_pairs;
        std::vector<SoftTriangle> triangle_areas;
        std::map<std::string, float> parms;

        Vector get_structForce(const SoftEdge& e);
        void get_areaForce(const SoftTriangle& t, Vector& f0, Vector& f1, Vector& f2);

    };

    typedef std::shared_ptr<SoftBodyStateData> SoftBodyState;
    SoftBodyState CreateSoftBodyState(const std::string& nam = "SoftBodyDataNoName");

}

#endif //PBA_SOFTBODYSTATE_H
