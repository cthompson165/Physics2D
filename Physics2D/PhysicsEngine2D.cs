using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.PhysicalObject;
using Physics2D.ForceGenerators;
using Physics2D.Integrator;
using Physics2D.CollisionDetection;
using Physics2D.Util;
using Physics2D.Constraint;

namespace Physics2D
{
   
/** PhysicsEngine2D coordinates all the activities of the physics engine.
 */
public class PhysicsEngine2D
    {
    private const double ZERO_VELOCITY = 0.000001;
    private const double STICKY_THRESHOLD = 0.02;
        
    private ODESolver objODE;
    private CollisionDetectionEngine objCDE;
    private PhysicsState physicsState;
    private ConstraintEngine objCE;
    private ForceEngine objFE;

    private List<object> m_ObjectsToUnregister = new List<object>();
                
    public PhysicsEngine2D()
        {
        physicsState = PhysicsState.reset();
        objCE = ConstraintEngine.reset();
        objFE = ForceEngine.reset();
                
        objCDE = new CollisionDetectionEngine();
        objODE = new ODERungeKuttaSolver();
        }
        
    /** Replace the default runge-kutta ODE integrator with a new one.
     */
    public void setODESolver(ODESolver solver)
        {
        this.objODE = solver;
        }

    public void step()
    {
        physicsState.backupCurrentPosition();

        // Handle collisions
        Bag contactList = objCDE.getCollisions();
        Bag collidingList = new Bag();

        // Notify each object involved in collisions of the collision
        for (int i = 0; i < contactList.size(); i++)
        {
            CollisionPair pair = (CollisionPair)contactList.objs[i];

            int colType1 = pair.c1.handleCollision(pair.c2, pair.getColPoint1());
            int colType2 = pair.c2.handleCollision(pair.c1, pair.getColPoint2());

            if (colType1 != 0 && colType2 != 0)
            {
                if (colType1 == 2 || colType2 == 2)
                    pair.setSticky();
                collidingList.add(pair);
            }
        }
        objCE.addCollisionResponses(collidingList);

        // Handle resting contacts and other forces/constraints
        physicsState.saveLastState();
        objODE.solve(1);

        if (m_ObjectsToUnregister.Count > 0)
        {
            physicsState.CacheState();
            foreach (object obj in m_ObjectsToUnregister)
            {
                doUnRegister(obj);
            }

            physicsState.reinit();
            objCDE.clear();

            for (int i = 0; i < physicsState.physObjs.numObjs; i++)
            {
                var po = physicsState.physObjs.objs[i] as PhysicalObject2D;
                if (po != null)
                {
                    objCDE.register(po);
                }
            }

            m_ObjectsToUnregister.Clear();
        }
    }

    private void doUnRegister(Object obj)
    {
        if (obj is ImpulseConstraint)
            objCE.unRegisterImpulseConstraint((ImpulseConstraint)obj);

        if (obj is ForceConstraint)
            objCE.unRegisterForceConstraint((ForceConstraint)obj);

        if (obj is ForceGenerator)
            objFE.unRegisterForceGenerator((ForceGenerator)obj);

        if (obj is MobileObject2D)
            objFE.unRegisterMobileObject((MobileObject2D)obj);

        var po = obj as PhysicalObject2D;
        if (po != null)
        {
            physicsState.removeBody(po);
        }        
    }
                
    /** Registers a physical object, force generator, or constraint
     * with the physics engine.
     */
    public void register(Object obj)
        {
            var po = obj as PhysicalObject2D;
            if (po != null)
            {
                physicsState.addBody(po);
                po.Registered();
                objCDE.register(po);
            }

        if (obj is MobileObject2D)
            objFE.registerMobileObject((MobileObject2D)obj);
                
        if (obj is ForceGenerator)
            objFE.registerForceGenerator((ForceGenerator)obj);
                
        if (obj is ForceConstraint)
            objCE.registerForceConstraint((ForceConstraint)obj);
                
        if (obj is ImpulseConstraint)
            objCE.registerImpulseConstraint((ImpulseConstraint)obj);
        }

    /** Turns off collision response for a pair of objects 
     */
    public void setNoCollisions(PhysicalObject2D c1, PhysicalObject2D c2)
        {
            objCE.setNoCollisions(c1, c2);
        }
        
    /** Removes a constraint
     */
    public void unRegister(Object obj)
        {
            m_ObjectsToUnregister.Add(obj);
             
        }
    }

}
