using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.PhysicalObject;
using Physics2D.Util;

namespace Physics2D.CollisionDetection
{
    
   /** The CollisionDetectionEngine coordinates and abstracts the collision 
 * detection logic.
 */
    public class CollisionDetectionEngine 
    {
    private BroadPhaseCollision2D objBPCollision; 
    private Collision2D objCollision;
        
    public CollisionDetectionEngine()
        {
        objCollision = new Collision2D();
        objBPCollision = new BroadPhaseCollision2D();
        }
        
    /** Returns a list of the pairs of objects currently colliding.
     */
    public Bag getCollisions()
        {
        objBPCollision.testCollisions();
        return objCollision.testCollisions(objBPCollision.getActiveList());
        }
        
    /** Registers an object with the collision detection engine.
     */
    public void register(PhysicalObject2D objCol)
        {
        objBPCollision.register(objCol);
        }
    }

}
