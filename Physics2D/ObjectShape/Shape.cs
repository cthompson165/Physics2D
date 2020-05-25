using Physics2D.Util;

namespace Physics2D.ObjectShape
{

    /** Each physical object has an associated shape. The type of shape associated
     * with the object determines how it is displayed, when it is colliding with another
     * object, and how its mass moment of inertia is calculated.
     * 
     * Shape is an abstract class representing any shape that can be associated with a 
     * physical object
     */
    public abstract class Shape
    {
        private PhysicsState _physicsState = PhysicsState.getInstance();
        protected bool _stationary;
        protected int _index;

        public Shape(bool stationary)
        {
            _stationary = stationary;
        }

        public Shape()
        {
            _stationary = false;
        }

        protected Double2D getPosition()
        {
            return _physicsState.getPosition(_index);
        }

        /** Tells the shape the index of its associated physical object. Used by shapes
         * to get the object's pose from the state vector.
         */
        public void setIndex(int index)
        {
            _index = index;
        }

        protected Angle getOrientation()
        {
            return _physicsState.getOrientation(_index);
        }

        /** Return the mass moment of inertia of this shape */
        public abstract double GetMassMomentOfInertia(double mass);

        /////////////////////////////////////////////////////////////////
        // These functions are used by the broad phase Collision detection 
        // logic
        /////////////////////////////////////////////////////////////////

        /** Calculate the max distance a point can be from the center of the object.
            For polygons, this can be different if the object is moving (rotating).
            For circles, this is alway the same. */
        public abstract void CalcMaxDistances(bool mobile);
        public abstract double GetMaxXDistanceFromCenter();
        public abstract double GetMaxYDistanceFromCenter();
    }
}
