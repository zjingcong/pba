//
// Created by jingcoz on 10/28/17.
//

#ifndef PBA_SBD_H
#define PBA_SBD_H

# include "Force.h"
# include "SoftBodyState.h"

namespace pba
{

    class SoftBodyInnerForce: public ForceBase
    {
    public:
        SoftBodyInnerForce(SoftBodyState& sb): SB(sb)    {name = "softBodyInnerForce";}
        const Vector getForce(const size_t& p);

    private:
        SoftBodyState SB;
    };

    ForcePtr CreateSoftBodyInnerForce(SoftBodyState& sb);

}

#endif //PBA_SBD_H
