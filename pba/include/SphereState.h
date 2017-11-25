//
// Created by jingcoz on 11/22/17.
//

#ifndef PBA_SPHERESTATE_H
#define PBA_SPHERESTATE_H

# include "DynamicalState.h"

namespace pba
{

    class SphereStateData: public DynamicalStateData
    {
    public:
        SphereStateData(const std::string& nam = "SphereDataNoName");
        ~SphereStateData()  {}

        const float& radius(const size_t p) const;
        const int& isCollision(const size_t p) const;
        void set_radius(const size_t p, const float& r);
        void set_isCollision(const size_t p, const int& flag);

        void clean_collision_flags();

    protected:
        std::map< std::string, DSAttribute<float> >::iterator radiuses;
        std::map< std::string, DSAttribute<int> >::iterator collision_flags;

        void re_find_main_attrs();
    };

    typedef std::shared_ptr<SphereStateData> SphereState;
    SphereState CreateSphereState(const std::string& nam = "SphereDataNoName");

}

#endif //PBA_SPHERESTATE_H
