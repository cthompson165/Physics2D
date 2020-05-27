using Physics2D.ObjectShape;
using Physics2D.PhysicalObject;
using Physics2D.Util;
using System.Drawing;
using System.Windows.Forms;

namespace Samples
{
    public class Ball : MobileObject2D
    {
        private float _radius;
        private int _twoRadius;

        public Ball(int radius, double x, double y)
        {
            _radius = radius + .5f; // centers the drawing
            _twoRadius = radius * 2;
            Circle circle = new Circle(radius);
            SetShape(circle, 5);
            SetMass(5);
            setPose(new Double2D(x, y), new Angle(0));
            setCoefficientOfRestitution(1);
        }

        public void Draw(PaintEventArgs e)
        {
            Double2D position = this.GetPosition();

            using (var brush = new SolidBrush(Color.Red))
            {
                e.Graphics.FillEllipse(brush, 
                    (float)(position.x - _radius), (float)(position.y - _radius), 
                    _twoRadius, _twoRadius);
            }
        }
    }
}
