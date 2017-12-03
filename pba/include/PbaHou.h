
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
    void add_DS(int num);
    void set_pos(int p, double x, double y, double z);
    void set_vel(int p, double x, double y, double z);
    void set_dt(double delta_t);
    void set_gravity(float g);

    double* get_pos(int p);
    int get_nb();

    void reset();

  private:
    static PbaHouParticles* pPbaHouParticles;
    DynamicalState DS;
    SolverPtr solver;
    ForcePtrContainer forces;
    ForcePtr gravity;
    double dt;

    // dont allow any of these
    PbaHouParticles();
    PbaHouParticles( const PbaHouParticles& );
    PbaHouParticles& operator= (const PbaHouParticles&);
};

    PbaHouParticles* CreatePbaHouParticles();

}

#endif
