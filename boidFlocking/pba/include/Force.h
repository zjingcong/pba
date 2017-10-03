//
// Created by jingcoz on 9/13/17.
//

# ifndef PBA_FORCE_H
# define PBA_FORCE_H

# include <string>
# include "Vector.h"
# include "DynamicalState.h"
# include "Boid.h"

namespace pba
{
    class ForceBase
    {
    public:
        ForceBase(): name("unknown")    {}
        virtual ~ForceBase()    {}

        const std::string& Name() const { return name; }
        const float getParms(const std::string& key) const {return parms.at(key);}
        //! update force
        void updateParms(const std::string& key, const float& value)  {parms.at(key) = value;}

        virtual const Vector getForce(DynamicalState DS, const size_t p) {return force;}

    protected:
        Vector force;
        std::string name;
        std::map<std::string, float> parms; // force parms
    };

    typedef ForceBase* ForcePtr;
    typedef std::vector<ForcePtr> ForcePtrContainer;


    //! -------------------------------------------------------------------------------------------------


    class Gravity: public ForceBase
    {
    public:
        Gravity(const float& gconstant)
        {
            name = "gravity";
            parms = {{"g", gconstant}}; // gravity parms: gravity constant
        }
        ~Gravity()  {}

        const Vector getForce(DynamicalState DS, const size_t p);
    };


    class BoidInnerForce: public ForceBase
    {
    public:
        BoidInnerForce(BoidPtr b): boid(b)
        {
            name = "boid_inner_force";
        }
        ~BoidInnerForce()   {}

        const Vector getForce(DynamicalState DS, const size_t p);

    private:
        BoidPtr boid;
    };
}

# endif //PBA_FORCE_H
