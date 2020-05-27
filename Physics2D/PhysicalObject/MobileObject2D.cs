using Physics2D.Util;
using Physics2D.ObjectShape;

namespace Physics2D.PhysicalObject
{

    /**
     * MobileObject2D represents a physical object that can move. 
     */
    public abstract class MobileObject2D : PhysicalObject2D
    {
        protected double _coefficientOfFriction;
        protected double _coefficientOfStaticFriction;

        protected double _mass = 0;
        protected double _massMomentOfInertia = 0;

        private const double ZERO_VELOCITY = 0.01;
        private double NORMAL_FORCE;
        private const double GRAVITY = .1;

        public MobileObject2D()
        {
            this.setPose(new Double2D(0, 0), new Angle(0));
            this.SetVelocity(new Double2D(0, 0));
            this.SetAngularVelocity(0);
        }

        /** Returns the object's mass
         */
        public double GetMass()
        {
            return _mass;
        }

        /** Sets an object's mass. The mass moment of inertia is calculated
         * by the object's associated shape
         */
        public void SetMass(double mass)
        {
            _massMomentOfInertia = shape.GetMassMomentOfInertia(mass);

            double massMomentOfInertiaInverse;
            if (_massMomentOfInertia > 0)
                massMomentOfInertiaInverse = 1 / this._massMomentOfInertia;
            else
                massMomentOfInertiaInverse = 0;

            // Precompute inverses since we need them a lot in collision response
            double massInverse = 1 / mass;
            m_State.setMassInverse(massInverse, massMomentOfInertiaInverse);

            this._mass = mass;
            NORMAL_FORCE = mass * GRAVITY;
        }

        /** Apply a force to the MobileObject */
        public virtual void AddForce(Double2D force)
        {
            m_State.AddForce(force);
        }

        /** Apply a torque to the MobileObject */
        public virtual void addTorque(double torque)
        {
            m_State.AddTorque(torque);
        }

        /** Positive value representing the coefficient of friction of the
         * object with the background surface. 0 is no friction
         */
        public double GetCoefficientOfFriction()
        {
            return _coefficientOfFriction;
        }

        /** Positive value representing the coefficient of friction of the
         * object with the background surface. 0 is no friction
         */
        public void SetCoefficientOfFriction(double coefficientOfFriction)
        {
            this._coefficientOfFriction = coefficientOfFriction;
        }

        /** Positive value representing the coefficient of static friction of the
         * object with the background surface. 0 is no static friction
         */
        public double GetCoefficientOfStaticFriction()
        {
            return _coefficientOfStaticFriction;
        }

        /** Positive value representing the coefficient of static friction of the
         * object with the background surface. 0 is no static friction
         */
        public void SetCoefficientOfStaticFriction(double coefficientOfStaticFriction)
        {
            this._coefficientOfStaticFriction = coefficientOfStaticFriction;
        }

        /** Set the shape of the object which determines how it is displayed, 
         * when it is colliding with another object, and how its mass moment of 
         * inertia is calculated
         */
        public void SetShape(Shape shape, double mass)
        {
            this.shape = shape;
            this.shape.setIndex(this.getIndex());
            this.shape.CalcMaxDistances(true);
            SetMass(mass);
        }

        /** Updates the pose to where the object would be in only a percentage
         * of a time step. Useful for searching for exact moment of collision.
         */
        public override void UpdatePose(double percent)
        {
            m_State.updatePose(percent);
        }

        /** Move the object back to its previous location */
        public override void ResetLastPose()
        {
            m_State.ResetLastPose();
        }

        /** Restores an object to its current location */
        public override void restorePose()
        {
            m_State.RestorePose();
        }

        /** Returns the object's velocity */
        public override Double2D GetVelocity()
        {
            return m_State.GetVelocity();
        }

        /** Updates the object's velocity */
        public virtual void SetVelocity(Double2D velocity)
        {
            m_State.SetVelocity(velocity);
        }

        /** How fast the object is rotating in radians per second.
         * A positive angular velocity means the object is rotating
         * counter clockwise
         */
        public override double GetAngularVelocity()
        {
            return m_State.GetAngularVelocity();
        }

        /** How fast the object is rotating in radians per second.
         * A positive angular velocity means the object is rotating
         * counter clockwise
         */
        public void SetAngularVelocity(double angularVelocity)
        {
            m_State.SetAngularVelocity(angularVelocity);
        }

        /** Returns a vector that represents a combination of 
         * all the forces applied to it
         */
        public Double2D GetForceAccumulator()
        {
            return m_State.GetForceAccumulator();
        }

        /** Returns a number that represents a combination of 
         * all the torques applied to it
         */
        public double GetTorqueAccumulator()
        {
            return m_State.getTorqueAccumulator();
        }

        /** 1 / mass. Used in collision response */
        public override double GetMassInverse()
        {
            return m_State.getMassInverse();
        }

        /** 1 / massMomentOfInertia. Used in collision response */
        public override double GetMassMomentOfInertiaInverse()
        {
            return m_State.GetMassMomentOfInertiaInverse();
        }

        /** Calculates and adds the static and dynamic friction forces on the object
         * based on the coefficients of friction. 
         */
        public void AddFrictionForce()
        {
            if (this._coefficientOfFriction > 0 || this._coefficientOfStaticFriction > 0)
            {
                Double2D velocity = this.GetVelocity();
                double velLength = velocity.length();
                if (velLength < ZERO_VELOCITY)
                {
                    // static friction
                    Double2D externalForce = this.GetForceAccumulator();
                    if (NORMAL_FORCE * this.GetCoefficientOfStaticFriction() > externalForce.length())
                        this.AddForce(new Double2D(-externalForce.x, -externalForce.y));
                }

                // add dynamic friction
                if (velLength > 0)
                {
                    Double2D velRot = new Double2D(-velocity.x, -velocity.y);
                    this.AddForce(velRot.multiply(this.GetCoefficientOfFriction() * NORMAL_FORCE));
                }
            }
        }
    }

}
