using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util;
using Physics2D.ObjectShape;

namespace Physics2D.PhysicalObject
{
    
/**
 * MobileObject2D represents a physical object that can move. 
 */
public abstract class MobileObject2D : PhysicalObject2D
    {
    protected double coefficientOfFriction; 
    protected double coefficientOfStaticFriction;

    protected double mass = 0;
    protected double massMomentOfInertia = 0;
        
    private const double zeroVelocity = 0.01;
    private double normalForce;
    private const double gravity = .1;

    public MobileObject2D()
        {
        this.setPose(new Double2D(0,0), new Angle(0));
        this.setVelocity(new Double2D(0,0));
        this.setAngularVelocity(0);
        }
        
    /** Returns the object's mass
     */
    public double getMass()
        {
        return mass;
        }

    /** Sets an object's mass. The mass moment of inertia is calculated
     * by the object's associated shape
     */
    public void setMass(double mass)
        {       
        massMomentOfInertia = shape.getMassMomentOfInertia(mass);
                
        double massMomentOfInertiaInverse;
        if (massMomentOfInertia > 0)
            massMomentOfInertiaInverse = 1 / this.massMomentOfInertia;
        else
            massMomentOfInertiaInverse = 0;
                
        // Precompute inverses since we need them a lot in collision response
        double massInverse = 1 / mass;
        m_State.setMassInverse(massInverse, massMomentOfInertiaInverse);
                
        this.mass = mass;
        normalForce = mass * gravity;
        }

    /** Apply a force to the MobileObject */
    public virtual void addForce(Double2D force)
        {
            m_State.addForce(force);
        }
        
    /** Apply a torque to the MobileObject */
    public virtual void addTorque(double torque)
        {
            m_State.addTorque(torque);
        }
        
    /** Positive value representing the coefficient of friction of the
     * object with the background surface. 0 is no friction
     */
    public double getCoefficientOfFriction()
        {
        return coefficientOfFriction;
        }
        
    /** Positive value representing the coefficient of friction of the
     * object with the background surface. 0 is no friction
     */
    public void setCoefficientOfFriction(double coefficientOfFriction)
        {
        this.coefficientOfFriction = coefficientOfFriction;
        }
        
    /** Positive value representing the coefficient of static friction of the
     * object with the background surface. 0 is no static friction
     */
    public double getCoefficientOfStaticFriction()
        {
        return coefficientOfStaticFriction;
        }

    /** Positive value representing the coefficient of static friction of the
     * object with the background surface. 0 is no static friction
     */
    public void setCoefficientOfStaticFriction(double coefficientOfStaticFriction)
        {
        this.coefficientOfStaticFriction = coefficientOfStaticFriction;
        }

    /** Set the shape of the object which determines how it is displayed, 
     * when it is colliding with another object, and how its mass moment of 
     * inertia is calculated
     */
    public void setShape(Shape shape, double mass)
        {
        this.shape = shape;
        this.shape.setIndex(this.getIndex());
        this.shape.calcMaxDistances(true);
        setMass(mass);
        }
        
    /** Updates the pose to where the object would be in only a percentage
     * of a time step. Useful for searching for exact moment of collision.
     */
    public override void updatePose(double percent)
        {
            m_State.updatePose(percent);
        }
        
    /** Move the object back to its previous location */
    public override void resetLastPose()
        {
            m_State.resetLastPose();
        }
        
    /** Restores an object to its current location */
    public override void restorePose()
        {
            m_State.restorePose();
        }

    /** Returns the object's velocity */
    public override Double2D getVelocity()
        {
            return m_State.getVelocity();
        }
        
    /** Updates the object's velocity */
    public virtual void setVelocity(Double2D velocity)
        {
            m_State.setVelocity(velocity);
            
        }
        
    /** How fast the object is rotating in radians per second.
     * A positive angular velocity means the object is rotating
     * counter clockwise
     */
    public override double getAngularVelocity()
        {
        return m_State.getAngularVelocity();
        }
        
    /** How fast the object is rotating in radians per second.
     * A positive angular velocity means the object is rotating
     * counter clockwise
     */
    public void setAngularVelocity(double angularVelocity)
    {
        m_State.setAngularVelocity(angularVelocity);
    }
        
    /** Returns a vector that represents a combination of 
     * all the forces applied to it
     */
    public Double2D getForceAccumulator()
        {
            return m_State.getForceAccumulator();
        }
        
    /** Returns a number that represents a combination of 
     * all the torques applied to it
     */
    public double getTorqueAccumulator()
        {
            return m_State.getTorqueAccumulator();
        }

    /** 1 / mass. Used in collision response */
    public override double getMassInverse()
        {
            return m_State.getMassInverse();
        }
        
    /** 1 / massMomentOfInertia. Used in collision response */
    public override double getMassMomentOfInertiaInverse()
        {
        return m_State.getMassMomentOfInertiaInverse();
        }
        
    /** Calculates and adds the static and dynamic friction forces on the object
     * based on the coefficients of friction. 
     */
    public void addFrictionForce()
        {
        if (this.coefficientOfFriction > 0 || this.coefficientOfStaticFriction > 0)
            {
            Double2D velocity = this.getVelocity(); 
            double velLength = velocity.length();
            if (velLength < zeroVelocity)
                {
                // static friction
                Double2D externalForce = this.getForceAccumulator();
                if (normalForce * this.getCoefficientOfStaticFriction() > externalForce.length())
                    this.addForce(new Double2D(-externalForce.x, -externalForce.y));
                }
                        
            // add dynamic friction
            if (velLength > 0)
                {
                Double2D velRot = new Double2D(-velocity.x, -velocity.y);
                this.addForce(velRot.multiply(this.getCoefficientOfFriction() * normalForce));
                }
            }
        }
    }

}
