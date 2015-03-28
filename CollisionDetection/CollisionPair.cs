using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.PhysicalObject;
using Physics2D.Util;

namespace Physics2D.CollisionDetection
{
    /** CollisionPair represents a pair of objects that are currently colliding
 */
public class CollisionPair : PhysicalObjectPair
    {
    public CollisionPair(PhysicalObject2D c1, PhysicalObject2D c2) : base(c1, c2)
        {
        }
        
    // Used in Narrow phase for closest Feature Tracking
    public int? closestFeature1 = null;
    public int? closestFeature2 = null;

    public Double2D colPoint1 = null;
    public Double2D colPoint2 = null;
    public Double2D normal = null;
        
    public double relVel;

    public bool stickyCol = false;
        
    // If this is set to true, the narrow phase processor will
    // never schedule the pair to collide. This is used when 
    // objects are colliding and it can't be determined when
    // they collided (e.g. they started on top of each other)
    public bool noCollision = false;
        
    /** Returns a vector pointing from the center of object 1 to
     * the collision point
     */
    public Double2D getColPoint1()
        {
        return colPoint1;
        }
        
    /** Returns a vector pointing from the center of object 2 to
     * the collision point
     */
    public Double2D getColPoint2()
        {
        return colPoint2;
        }
        
    /** Returns the collision normal. The collision normal points out
     * from object 2.
     */
    public Double2D getNormal()
        {
        return normal;
        }
        
    /** Returns the relative velocity of the objects along the collision normal
     */
    public double getRelativeVelocity()
        {
        return relVel;
        }
        
    /** Indicates that this collision should be perfectly inelastic (so the objects
     * lose all energy along the collision normal).
     */
    public void setSticky()
        {
        stickyCol = true;
        }
        
    /** Returns a value indicated whether this is a sticky collision or not.
     */
    public bool getSticky()
        {
        return stickyCol;
        }
        
    public void clear()
        {
        this.closestFeature1 = null;
        this.closestFeature2 = null;
        this.colPoint1 = null;
        this.colPoint2 = null;
        this.relVel = 0;
        }
    }
}
