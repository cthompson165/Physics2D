using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.Integrator
{


/** Represents the interface for ordinary differential equation solvers. These
 * solvers take the current state of the system (position, velocity, and external forces) 
 * and solve for the next state. 
 */
public interface ODESolver 
    {
    void solve(double stepSize);
    }

}
