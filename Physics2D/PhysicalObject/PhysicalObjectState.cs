using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util;

namespace Physics2D.PhysicalObject
{
    public class PhysicalObjectState
    {
        public int index;
        protected bool isRegistered;
        private Double2D m_Position = new Double2D();
        private Angle m_Orientation;
        private Double2D m_Velocity = new Double2D();
        private double m_AngularVelocity;
        private Double2D m_ExternalForce = new Double2D();
        private double m_ExternalTorque;

        private double m_MassInverse;
        private double m_MassMomentOfInertiaInverse;

        protected PhysicsState physicsState = PhysicsState.getInstance();

        public void Registered()
        {
            isRegistered = true;

            RestoreState();
        }

        public void Unregistered()
        {
            isRegistered = false;
            index = -1;
        }

        public void RestoreState()
        {
            physicsState.setPosition(m_Position, index);
            physicsState.setOrientation(m_Orientation, index);
            physicsState.addExternalTorque(m_ExternalTorque, index);
            physicsState.addExternalForce(m_ExternalForce, index);
            physicsState.setVelocity(m_Velocity, index);
            physicsState.setMassInverse(m_MassInverse, m_MassMomentOfInertiaInverse, index);
            physicsState.setAngularVelocity(m_AngularVelocity, index);
        }

        public void CacheState()
        {
            m_Position = physicsState.getPosition(index);
            m_Orientation = physicsState.getOrientation(index);
            m_ExternalTorque = physicsState.getExternalTorque(index);
            m_ExternalForce = physicsState.getExternalForce(index);
            m_Velocity = physicsState.getVelocity(index);
            m_MassInverse = physicsState.getMassInverse(index);
            m_MassMomentOfInertiaInverse = physicsState.getMassMomentOfInertiaInverse(index);
            m_AngularVelocity = physicsState.getAngularVelocity(index);
        }

        public Double2D getPosition()
        {
            if (isRegistered)
                return physicsState.getPosition(index);
            else
                return m_Position;
        }

        /** Returns an object's current orientation 
         */
        public Angle getOrientation()
        {
            if (isRegistered)
                return physicsState.getOrientation(index);
            else
                return m_Orientation;
        }

        
        public void setPose(Double2D position, Angle orientation)
        {
            if (isRegistered)
            {
                physicsState.setPosition(position, index);
                physicsState.setOrientation(orientation, index);
            }
            else
            {
                m_Position = position;
                m_Orientation = orientation;
            }
        }

        /** Apply a torque to the MobileObject */
        public virtual void AddTorque(double torque)
        {
            physicsState.addExternalTorque(torque, this.index);
        }

        public virtual void AddForce(Double2D force)
        {
            physicsState.addExternalForce(force, index);
        }

        public Double2D GetVelocity()
        {
            if (isRegistered)
            {
                return physicsState.getVelocity(index);
            }
            else
            {
                return m_Velocity;
            }
        }

        /** Updates the object's velocity */
        public void SetVelocity(Double2D velocity)
        {
            if (isRegistered)
            {
                physicsState.setVelocity(velocity, index);
            }
            else
            {
                m_Velocity = velocity;
            }

        }

        /** How fast the object is rotating in radians per second.
         * A positive angular velocity means the object is rotating
         * counter clockwise
         */
        public double GetAngularVelocity()
        {
            if (isRegistered)
            {
                return physicsState.getAngularVelocity(index);
            }
            else
            {
                return m_AngularVelocity;
            }
        }

        /** How fast the object is rotating in radians per second.
         * A positive angular velocity means the object is rotating
         * counter clockwise
         */
        public void SetAngularVelocity(double angularVelocity)
        {

            if (isRegistered)
            {
                physicsState.setAngularVelocity(angularVelocity, index);
            }
            else
            {
                m_AngularVelocity = angularVelocity;
            }
        }

        /** Returns a vector that represents a combination of 
         * all the forces applied to it
         */
        public Double2D GetForceAccumulator()
        {
            if (isRegistered)
                return physicsState.getExternalForce(index);
            else
                return m_ExternalForce;
        }

        /** Returns a number that represents a combination of 
         * all the torques applied to it
         */
        public double getTorqueAccumulator()
        {
            if (isRegistered)
                return physicsState.getExternalTorque(index);
            else
                return m_ExternalTorque;
        }

        public void setMassInverse(double massInverse, double massMomentOfInertiaInverse)
        {
            if (isRegistered)
                physicsState.setMassInverse(massInverse, massMomentOfInertiaInverse, index);
            else
            {
                m_MassInverse = massInverse;
                m_MassMomentOfInertiaInverse = massMomentOfInertiaInverse;
            }
        }

        
        /** 1 / mass. Used in collision response */
        public double getMassInverse()
        {
            if (isRegistered)
                return physicsState.getMassInverse(index);
            else
                return m_MassInverse;
        }

        /** 1 / massMomentOfInertia. Used in collision response */
        public double GetMassMomentOfInertiaInverse()
        {
            if (isRegistered)
                return physicsState.getMassMomentOfInertiaInverse(index);
            else
                return m_MassMomentOfInertiaInverse;
        }

        public void updatePose(double percent)
        {
            if (!isRegistered)
                throw new Exception("Object must be registered");
            this.setPose(this.getPosition().add(physicsState.getLastVelocity(this.index).multiply(percent)), this.getOrientation().Add(physicsState.getLastAngularVelocity(this.index)));
        }

        /** Move the object back to its previous location */
        public void ResetLastPose()
        {
            if (!isRegistered)
                throw new Exception("Object must be registered");
            
            this.setPose(physicsState.getLastPosition(this.index), physicsState.getLastOrientation(this.index));
        }

        /** Restores an object to its current location */
        public void RestorePose()
        {
            if (!isRegistered)
                throw new Exception("Object must be registered");
            
            this.setPose(physicsState.getSavedPosition(this.index), physicsState.getSavedOrientation(this.index));
        }
    }
}
