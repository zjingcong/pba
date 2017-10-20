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
    class Boid;
    typedef std::shared_ptr<Boid> BoidPtr;
}

namespace pba
{
    class ForceBase
    {
    public:
        ForceBase(): name("unknown")    {}
        virtual ~ForceBase()    {}

        const std::string& Name() const { return name; }
        const float get_floatParms(const std::string &key) const {return floatParms.at(key);}
        const Vector get_vectorParms(const std::string &key) const { return vectorParms.at(key);}
        //! update force
        void update_parms(const std::string &key, const float &value)  {floatParms.at(key) = value;}
        void update_parms(const std::string &key, const Vector &value)  {vectorParms.at(key) = value;}

        virtual const Vector getForce(DynamicalState DS, const size_t p) {return force;}

    protected:
        Vector force;
        std::string name;
        std::map<std::string, float> floatParms; // force floatParms
        std::map<std::string, Vector> vectorParms; // force floatParms
    };

    typedef std::shared_ptr<ForceBase> ForcePtr;
    typedef std::vector<ForcePtr> ForcePtrContainer;


    //! -------------------------------------------------------------------------------------------------


    class Gravity: public ForceBase
    {
    public:
        Gravity(const float& gconstant)
        {
            name = "gravity";
            floatParms = {{"g", gconstant}}; // gravity floatParms: gravity constant
        }
        ~Gravity()  {}

        const Vector getForce(DynamicalState DS, const size_t p);
    };


    class BoidInnerForce: public ForceBase
    {
    public:
        BoidInnerForce(BoidPtr& b): boid(b) { name = "boid_inner_force"; }
        ~BoidInnerForce()   {}

        const Vector getForce(DynamicalState DS, const size_t p);

    private:
        BoidPtr& boid;
    };


    class Spring: public ForceBase
    {
    public:
        Spring(const Vector& x0, const float& kconstant)
        {
            name = "spring";
            floatParms = {{"k", kconstant}};
            vectorParms = {{"x0", x0}};
        }
        ~Spring()   {}

        const Vector getForce(DynamicalState DS, const size_t p);
    };


    class MagneticForce: public ForceBase
    {
    public:
        MagneticForce(const Vector& xm, const float& b)
        {
            name = "magnetic_force";
            floatParms = {{"B", b}};
            vectorParms = {{"xm", xm}};
        }
        ~MagneticForce()    {}

        const Vector getForce(DynamicalState DS, const size_t p);
    };


    /// set up forces shared ptrs
    ForcePtr CreateGravity(const float& gconstant);
    ForcePtr CreateBoidInnerForce(BoidPtr& b);
    ForcePtr CreateSpring(const Vector& x0, const float& kconstant);
    ForcePtr CreateMagneticForce(const Vector& xm, const float& b);
}

# include "Boid.h"
# endif //PBA_FORCE_H
