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
    public double radians;
        
    // Pre-compute some often used constants
    public const double twoPI = 2 * Math.PI;
    public const double halfPI = Math.PI / 2;

    public Angle(double radians)
        {
        this.radians = GetNormalizedRadians(radians);
        }

    /** Adds two angles and makes sure the result doesn't
     * falls below 0 or above 2 P 
     */
    public Angle add(Angle other)
        {
        return add(other.radians);
        }

    /** Adds two angles and makes sure the result doesn't
     * falls below 0 or above 2 P 
     */
    public Angle add(double radians)
        {
            double newVal = GetNormalizedRadians(radians + this.radians);
            return new Angle(newVal);
        }

    private double GetNormalizedRadians(double radians)
    {
        while (radians > twoPI)
            radians -= twoPI;

        while (radians < 0)
            radians += twoPI;

        return radians;
    }

    }
}
