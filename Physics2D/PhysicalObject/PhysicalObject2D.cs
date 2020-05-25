using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util;
using Physics2D.ObjectShape;

namespace Physics2D.PhysicalObject
{
    public abstract class PhysicalObject2D
    {
    // Data members common to all physical objects
    protected Shape shape;
    protected PhysicalObjectState m_State = new PhysicalObjectState();
    protected double coefficientOfRestitution; // elasticity
 
    /** Returns the object's index, which uniquely identifies the object
     * and determines where its state variables are kept in the state
     * vectors and matrices.
     */
    public int getIndex()
        {
            return m_State.index;
        }
        
    public void setIndex(int index)
        {
            this.shape.setIndex(index);
            m_State.index = index;
        }

    public void Registered()
    {
        m_State.Registered();
    }

    public void Unregistered()
    {
        m_State.Unregistered();
    }

    public void RestoreState()
    {
        m_State.RestoreState();
    }

    public void CacheState()
    {
        m_State.CacheState();
    }
        
    public PhysicalObject2D()
        {
        
        }
        
    /** Returns an object's associated shape
     */
    public Shape getShape()
        {
        return shape;
        }

    /** Returns an object's current position
     */
    public Double2D getPosition()
        {
            return m_State.getPosition();
        }
        
    /** Returns an object's current orientation 
     */
    public Angle getOrientation()
        {
            return m_State.getOrientation();
        }
        
    /** Represents the elasticity of an object
     * 1 is perfectly elastic and 0 is perfectly inelastic. 
     * Determines how much momentum is conserved when objects collide
     */
    public double getCoefficientOfRestitution()
        {
        return coefficientOfRestitution;
        }
        
    /** Represents the elasticity of an object
     * 1 is perfectly elastic and 0 is perfectly inelastic. 
     * Determines how much momentum is conserved when objects collide
     */
    public void setCoefficientOfRestitution(double coefficientOfRestitution)
        {
        this.coefficientOfRestitution = coefficientOfRestitution;
        }
     
   
        /*
    /// Display the object
    public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
        {
        shape.draw(object, graphics, info);
        }*/
        
    /** Set the pose of the object */
    public void setPose(Double2D position, Angle orientation)
        {
            m_State.setPose(position, orientation);
        }
        
    // Member functions that vary between mobile and stationary objects
    abstract public Double2D getVelocity();
        
    /** How fast the object is rotating in radians per second.
     * A positive angular velocity means the object is rotating
     * counter clockwise
     */
    abstract public double getAngularVelocity();
        
    /** Provides a default implementation for the function used by the collision 
     * detection engine to notify an object when it has collided with another object. 
     */
    public virtual int handleCollision(PhysicalObject2D other, Double2D colPoint)
        {
        return 1; //regular collision
        }
        
    /////////////////////////////////////////////////////
    // Abstract functions used by collision detection
    /////////////////////////////////////////////////////
    abstract public void resetLastPose();
    abstract public void updatePose(double percent);
    abstract public void restorePose();
    abstract public double getMassInverse();
    abstract public double getMassMomentOfInertiaInverse();
    }
}
