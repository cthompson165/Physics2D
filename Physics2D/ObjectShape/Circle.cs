namespace Physics2D.ObjectShape
{
    /** The Circle class is used by a circular physical object to store the attributes
     * of its appearance and size
     */
    public class Circle : Shape
    {
        private double _radius;

        public Circle(double radius)
        {
            _radius = radius;
        }

        public double getRadius()
        {
            return _radius;
        }

        /** Calculates the mass moment of inertia for a circle based on its mass */
        public override double GetMassMomentOfInertia(double mass)
        {
            return mass * _radius * _radius / 2;
        }

        /////////////////////////////////////////////////////////////////
        // These functions are used by the broad phase Collision detection 
        // logic
        /////////////////////////////////////////////////////////////////

        /** Calculate the max distance a point can be from the center of the object.
            For polygons, this can be different if the object is moving (rotating).
            For circles, this is alway the same. */
        public override void CalcMaxDistances(bool mobile)
        {
            // stationary and mobile circles have the same max distances
        }

        public override double GetMaxXDistanceFromCenter()
        {
            return _radius;
        }

        public override double GetMaxYDistanceFromCenter()
        {
            return _radius;
        }
    }
}
