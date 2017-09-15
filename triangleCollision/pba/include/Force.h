//
// Created by jingcoz on 9/13/17.
//

#ifndef PBA_FORCE_H
#define PBA_FORCE_H

# include <string>
# include "Vector.h"
# include "DynamicalState.h"

namespace pba
{
    class ForceBase
    {
    public:
        ForceBase(): name("unknown")    {}
        virtual ~ForceBase()    {}

        const std::string& Name() const { return name; }
        virtual void updateForce(DynamicalState ds, const size_t p) {};
        const Vector getForce(DynamicalState DS, const size_t p)
        {
            updateForce(DS, p);
            return force;
        }

    protected:
        Vector force;
        std::string name;
    };

    typedef ForceBase* ForcePtr;


    class Gravity: public ForceBase
    {
    public:
        Gravity(float gconstant): g(gconstant)
            {name="gravity";}
        ~Gravity()  {}

        void updateForce(DynamicalState DS, const size_t p);
        void setGConstant(float gconstant)   {g = gconstant;}
        const float getGConstant() const {return g;}

    private:
        float g;
    };
}

#endif //PBA_FORCE_H
