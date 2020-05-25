using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.Util
{
    /** Angle handles adding angles and ensuring that the result never gets
     * above 2PI or below 0. 
     */
    public class Angle
    {
        public double _radians;

        // Pre-compute some often used constants
        public const double TWO_PI = 2 * Math.PI;
        public const double HALF_PI = Math.PI / 2;

        public Angle()
        {
        }

        public Angle(double radians)
        {
            _radians = GetNormalizedRadians(radians);
        }

        public double Degrees
        {
            get { return _radians * 180 / Math.PI; }
        }

        /** Adds two angles and makes sure the result doesn't
         * falls below 0 or above 2 P 
         */
        public Angle Add(Angle other)
        {
            return Add(other._radians);
        }

        /** Adds two angles and makes sure the result doesn't
         * falls below 0 or above 2 P 
         */
        public Angle Add(double radians)
        {
            double newVal = GetNormalizedRadians(radians + _radians);
            return new Angle(newVal);
        }

        private double GetNormalizedRadians(double radians)
        {
            while (radians > TWO_PI)
                radians -= TWO_PI;

            while (radians < 0)
                radians += TWO_PI;

            return radians;
        }
    }
}
