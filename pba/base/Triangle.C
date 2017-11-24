//
// Created by jingcoz on 9/18/17.
//

# include "Triangle.h"

using namespace std;
using namespace pba;

Triangle::Triangle(const pba::Vector& p0, const pba::Vector& p1, const pba::Vector& p2):
        P0(p0), P1(p1), P2(p2), isCollision(false), isVisible(true)
{
    e1 = P1 - P0;
    e2 = P2 - P0;
    e3 = P2 - P1;
    norm = (e2^e1).unitvector();
}
