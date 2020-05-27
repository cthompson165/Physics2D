using Physics2D.PhysicalObject;
using Physics2D.Util;
using System.Drawing;
using System.Windows.Forms;

namespace Samples
{
    public class Rod : MobileObject2D
    {
        public Rod(int width, int height, Double2D position, double orientation, Double2D velocity)
        {
            // initial position and velocity
            SetVelocity(velocity);
            setPose(position, new Angle(orientation));

            // shape - determines how the object will look and how collision detection is done
            SetShape(new Physics2D.ObjectShape.Rectangle(width, height), width * height);

            // physical properties
            SetCoefficientOfFriction(0);
            setCoefficientOfRestitution(1);
        }

        public void Draw(PaintEventArgs e)
        {
            Double2D[] vertices = ((Physics2D.ObjectShape.Polygon)shape).GetVertices();
            var points = new Point[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                Double2D vertex = vertices[i];
                points[i] = new Point((int)vertex.x, (int)vertex.y);
            }

            using (var brush = new SolidBrush(Color.Red))
            {
                e.Graphics.FillPolygon(brush, points);
            }
        }
    }
}
