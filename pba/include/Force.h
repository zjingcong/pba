//
// Created by jingcoz on 9/13/17.
//

# ifndef PBA_FORCE_H
# define PBA_FORCE_H

# include <string>
# include "Vector.h"
# include "DynamicalState.h"
# include "Noise.h"
# include "PerlinNoise.h"

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
        ForceBase(): name("unknown")    {force = Vector(0.0, 0.0, 0.0);}
        virtual ~ForceBase()    {}

        const std::string& Name() const { return name; }
        const float get_floatParms(const std::string &key) const {return floatParms.at(key);}
        const Vector get_vectorParms(const std::string &key) const { return vectorParms.at(key);}
        //! update force
        void update_parms(const std::string &key, const float &value)  {floatParms.at(key) = value;}
        void update_parms(const std::string &key, const Vector &value)  {vectorParms.at(key) = value;}

        virtual const Vector getForce(const size_t& p) {return force;}

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
        Gravity(DynamicalState& ds, const float& gconstant): DS(ds)
        {
            name = "gravity";
            floatParms = {{"g", gconstant}}; // gravity floatParms: gravity constant
        }
        ~Gravity()  {}

        const Vector getForce(const size_t& p);

    private:
        DynamicalState DS;
    };


    class BoidInnerForce: public ForceBase
    {
    public:
        BoidInnerForce(DynamicalState& ds, BoidPtr& b): DS(ds), boid(b) { name = "boid_inner_force"; }
        ~BoidInnerForce()   {}

        const Vector getForce(const size_t& p);

    private:
        DynamicalState DS;
        BoidPtr& boid;
    };


    class Spring: public ForceBase
    {
    public:
        Spring(DynamicalState& ds, const Vector& x0, const float& kconstant): DS(ds)
        {
            name = "spring";
            floatParms = {{"k", kconstant}};
            vectorParms = {{"x0", x0}};
        }
        ~Spring()   {}

        const Vector getForce(const size_t& p);

    private:
        DynamicalState DS;
    };


    class MagneticForce: public ForceBase
    {
    public:
        MagneticForce(DynamicalState& ds, const Vector& xm, const float& b): DS(ds)
        {
            name = "magnetic_force";
            floatParms = {{"B", b}};
            vectorParms = {{"xm", xm}};
        }
        ~MagneticForce()    {}

        const Vector getForce(const size_t& p);

    private:
        DynamicalState DS;
    };


    class NoiseForce: public ForceBase
    {
    public:
        NoiseForce(DynamicalState& ds, Noise* n, const float& scale, const float& offset): DS(ds), noise(n)
        {
            name = "noise_force";
            floatParms = {{"scale", scale}, {"offset", offset}};
        }
        ~NoiseForce()   {}

        const Vector getForce(const size_t& p);

    private:
        DynamicalState DS;
        Noise* noise;
    };


    /// set up forces shared ptrs
    ForcePtr CreateGravity(DynamicalState ds, const float& gconstant);
    ForcePtr CreateBoidInnerForce(DynamicalState ds, BoidPtr& b);
    ForcePtr CreateSpring(DynamicalState ds, const Vector& x0, const float& kconstant);
    ForcePtr CreateMagneticForce(DynamicalState ds, const Vector& xm, const float& b);
    ForcePtr CreateNoiseForce(DynamicalState ds, Noise* n, const float& scale, const float& offset);
}

# include "Boid.h"
# endif //PBA_FORCE_H
