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

    class SoftBodyStateData: public DynamicalStateData
    {
    public:
        SoftBodyStateData(const std::string& nam = "SoftBodyDataNoName");
        ~SoftBodyStateData()    {}

        void Init(const std::vector<Vector>& verts);
        void set_softEdges(const std::vector<std::pair<size_t, size_t>>& pairs);

        std::vector<SoftEdge>& get_connectedPairs() { return connected_pairs;}
        const Vector innerForce(const size_t& p);

        void update_structForce();

    private:
        std::vector<SoftEdge> connected_pairs;
        double Ks;
        double Kf;

        Vector get_structForce(const size_t& i, const size_t& j, const double& L);

    };

    typedef std::shared_ptr<SoftBodyStateData> SoftBodyState;
    SoftBodyState CreateSoftBodyState(const std::string& nam = "SoftBodyDataNoName");

}

#endif //PBA_SOFTBODYSTATE_H
