
#ifndef ____PBA_HOU_H____
#define ____PBA_HOU_H____

# include <vector>
# include <string>
# include "assert.h"
# include "DynamicalState.h"
# include "Solver.h"

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
    void solve();
    //! clean "dead" particles
    void clean();

    void add_DS(int num, float life, float variance);
    void set_pos(int p, double x, double y, double z);
    void set_vel(int p, double x, double y, double z);
    void set_dt(double delta_t);
    void set_gravity(float g);
    void set_current_frame(int f);

    double* get_pos(int p);
    double* get_vel(int p);
    int get_id(int p);
    int get_nb();
    float get_age(int p);
    float get_life(int p);
    float get_pscale(int p);
    int get_dead(int p);

    void reset();

  private:
    static PbaHouParticles* pPbaHouParticles;
    DynamicalState DS;
    SolverPtr solver;
    ForcePtrContainer forces;
    ForcePtr gravity;
    double dt;
    int current_frame;

    // dont allow any of these
    PbaHouParticles();
    PbaHouParticles( const PbaHouParticles& );
    PbaHouParticles& operator= (const PbaHouParticles&);
};

    PbaHouParticles* CreatePbaHouParticles();

}

#endif
