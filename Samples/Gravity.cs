using Physics2D.ForceGenerators;
using Physics2D.PhysicalObject;
using Physics2D.Util;
using System.Collections.Generic;

namespace Samples
{
    public class Gravity : ForceGenerator
    {
        private List<MobileObject2D> _objects = new List<MobileObject2D>();
        private Double2D _gravity = new Double2D(0, 5);

        public void Affects(MobileObject2D mobileObject)
        {
            _objects.Add(mobileObject);
        }

        void ForceGenerator.addForce()
        {
            foreach (MobileObject2D mobileObject in _objects)
            {
                mobileObject.AddForce(_gravity);
            }
        }
    }
}
