using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.Util
{
    public class Double2D
    {
        public Double2D(double x, double y)
        {
            this.x = x;
            this.y = y;
        }

        public double x;
        public double y;

        public Double2D add(Double2D other)
        {
            return new Double2D(x + other.x, y + other.y);
        }

        public Double2D subtract(Double2D other)
        {
            return new Double2D(x - other.x, y - other.y);
        }

        public Double2D normalize()
        {
            double magnitude = length();
            return new Double2D(x / magnitude, y / magnitude);
        }

        public double length()
        {
            return Math.Sqrt(x * x + y * y);
        }

        public double dot(Double2D other)
        {
            return x * other.x + y * other.y;
        }

        public Double2D multiply(double value)
        {
            return new Double2D(x * value, y * value);
        }

        public Double2D rotate(double angle)
        {
            double sinTheta = Math.Sin(angle);
            double cosTheta = Math.Cos(angle);
            double x = this.x;
            double y = this.y;
            return new Double2D(cosTheta * x + -sinTheta * y, sinTheta * x + cosTheta * y);
        }

        public double perpDot(Double2D other)
        {
            return (-this.y) * other.x + this.x * other.y;  
        }
    }
}
