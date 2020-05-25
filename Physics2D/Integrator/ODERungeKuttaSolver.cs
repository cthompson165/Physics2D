using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D;
using Physics2D.ForceGenerators;
using Physics2D.Util.Matrix;
using Physics2D.Constraint;

namespace Physics2D.Integrator
{

/** Implements a Runge-Kutta ordinary differential equation solver. The runge-kutta solver
 * reduces errors over the euler integrator by adding terms in the taylor series expansion. 
 */
public class ODERungeKuttaSolver : ODESolver
    {
    private ConstraintEngine objCE;
    private ForceEngine objFE;
        
    private PhysicsState physicsState;
        
    public ODERungeKuttaSolver()
        {
        physicsState = PhysicsState.getInstance();
        this.objCE = ConstraintEngine.getInstance();
        this.objFE = ForceEngine.getInstance();
        }

    private Vector ODEFunction(Vector state)
        {
        int stateRowDimension = state.m;
        int halfStateRowDimension = stateRowDimension / 2;
                
        // Update the physics engine so it thinks this is the current state
        physicsState.setStateVector(state);
        objFE.addForces();
        Vector externalForcesVector = physicsState.getExternalForcesVector();
        Vector totalForces;
                
        // this is for resting contact
        /*
          if (physicsState.lcp.contacts.numObjs == 0)
          {
          matrix.Vector constraintForces = objPE.calculateConstraintForces(externalForcesMatrix);                 
          matrix.Vector constraintForces = objPE.calculateConstraintForces(externalForcesVector);
                        
          // Solve for total forces and accelerations
          totalForces = constraintForces.plus(externalForcesVector);
          }
          else
          totalForces = externalForcesVector;
        */
                
        Vector constraintForces = objCE.calculateConstraintForces(externalForcesVector);
        totalForces = constraintForces.plus(externalForcesVector);
        Vector acc = physicsState.getMassInverseMatrix().times(totalForces);
                
        // Set the first half of the stateDot matrix to velocity
        // (second half of state matrix) and the second half of the
        // stateDot matrix to the accelerations just calculated
        Vector stateDot = new Vector(stateRowDimension);
        for (int i = 0; i < halfStateRowDimension; i++)
            {
            stateDot.vals[i] = state.vals[i + halfStateRowDimension];
            stateDot.vals[i + halfStateRowDimension] = acc.vals[i];
            }
                
        return stateDot;
        }

    public void solve(double stepSize)
        {
        // Make a copy of the state at the start of the current step
        Vector stepState = physicsState.getStateVectorCopy();
                
        Vector f1 = ODEFunction(stepState).times(stepSize);
        Vector f2 = ODEFunction(stepState.plus(f1.times(0.5))).times(stepSize);
        Vector f3 = ODEFunction(stepState.plus(f2.times(0.5))).times(stepSize);
        Vector f4 = ODEFunction(stepState.plus(f3)).times(stepSize);
                
        // Update the step to be the new state at the end of this step
        physicsState.setStateVector(stepState.plus(f1.plus(f2.times(2)).plus(f3.times(2)).plus(f4).times((double)1/6)));
        }
    }

}
