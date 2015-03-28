using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.ObjectShape
{

/** The Circle class is used by a circular physical object to store the attributes
 * of its appearance and size
 */
public class Circle : Shape
    {
    private double radius;
        
    public Circle(double radius)
        {
        this.radius = radius;
        // this.paint = paint;
        }
        
    public double getRadius()
        {
        return radius;
        }
        
    /** Calculates the mass moment of inertia for a circle based on its mass */
    public override double getMassMomentOfInertia(double mass)
        {
        return mass * radius * radius / 2;
        }

    /*
    /// Display the circle
    public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
        {
        final double width = info.draw.width * radius * 2;
        final double height = info.draw.height  * radius * 2;

        graphics.setPaint(paint);
        // we are doing a simple draw, so we ignore the info.clip
        final int x = (int)(info.draw.x - width / 2.0);
        final int y = (int)(info.draw.y - height / 2.0);
        final int w = (int)(width);
        final int h = (int)(height);

        // draw centered on the origin
        graphics.fillOval(x,y,w, h);
        }
    */

    /////////////////////////////////////////////////////////////////
    // These functions are used by the broad phase Collision detection 
    // logic
    /////////////////////////////////////////////////////////////////
        
    /** Calculate the max distance a point can be from the center of the object.
        For polygons, this can be different if the object is moving (rotating).
        For circles, this is alway the same. */
    public override void calcMaxDistances(bool mobile)
        {
        // stationary and mobile circles have the same max distances
        }

    public override double getMaxXDistanceFromCenter()
        {
        return this.radius;
        }

    public override double getMaxYDistanceFromCenter()
        {
        return this.radius;
        }
    }

}
