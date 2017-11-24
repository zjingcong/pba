//
// Created by jingcoz on 10/28/17.
//

# include "SBD.h"

using namespace pba;
using namespace std;

const Vector SoftBodyInnerForce::getForce(const size_t& p)
{
    return SB->innerForce(p);
}


pba::ForcePtr pba::CreateSoftBodyInnerForce(SoftBodyState &sb)
{
    return ForcePtr(new SoftBodyInnerForce(sb));
}
