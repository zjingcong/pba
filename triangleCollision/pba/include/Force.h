//
// Created by jingcoz on 9/13/17.
//

# ifndef PBA_FORCE_H
# define PBA_FORCE_H

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


    class Gravity: public ForceBase
    {
    public:
        Gravity(float gconstant)
        {
            name="gravity";
            parms = {{"g", gconstant}}; // gravity parms: gravity constant
        }
        ~Gravity()  {}

        const Vector getForce(DynamicalState DS, const size_t p);
    };
}

# endif //PBA_FORCE_H
