
#ifndef ____PBA_HOU_H____
#define ____PBA_HOU_H____

# include <vector>
# include <string>
# include "assert.h"
# include "DynamicalState.h"
# include "Solver.h"
# include "Noise.h"
# include "PerlinNoise.h"

using namespace std;

namespace pba
{
class PbaHouParticles
{
  public:
    static  PbaHouParticles* Instance()
    {
       if(pPbaHouParticles==nullptr)
       {
          pPbaHouParticles = new PbaHouParticles();
       }
       return pPbaHouParticles;
    }

    ~PbaHouParticles();
    void init(const std::vector<std::string>& args);

    //! leap frog solver
    void solve();
    //! add and init particles
    void add_DS(int num, float life, float variance, float pscale);
    //! clean "dead" particles
    void clean();
    //! set other attribs
    void update_status(double vel_max);
    //! create perlin noise based on given parms
    void create_noise(float gamma, float freq, float f_jump, float octaves, float offset);
    //! advect particles by 3d-noise field
    void advect_by_noise(double scale);
    //! add particles vel by 3d-noise field
    void add_by_noise(double scale);

    //! set values
    void set_pos(int p, double x, double y, double z);
    void set_vel(int p, double x, double y, double z);
    void set_dt(double delta_t);
    void set_gravity(float g);
    void set_current_frame(int f);

    //! get values
    double* get_pos(int p);
    double* get_vel(int p);
    double* get_cd(int p);
    int get_id(int p);
    int get_nb();
    float get_age(int p);
    float get_life(int p);
    float get_pscale(int p);
    int get_dead(int p);

    //! reset
    void reset();

  private:
    static PbaHouParticles* pPbaHouParticles;
    DynamicalState DS;
    SolverPtr solver;
    float life;
    float variance;
    ForcePtrContainer forces;
    ForcePtr gravity;
    double dt;
    int current_frame;
    Noise* perlin_noise;
    float noise_offset;

    double clamp(double value, double min, double max);
    Vector noise_field(const Vector& pos);

    // dont allow any of these
    PbaHouParticles();
    PbaHouParticles( const PbaHouParticles& );
    PbaHouParticles& operator= (const PbaHouParticles&);
};

    PbaHouParticles* CreatePbaHouParticles();

}

#endif
