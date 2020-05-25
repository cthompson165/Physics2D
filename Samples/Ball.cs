using Physics2D.ObjectShape;
using Physics2D.PhysicalObject;
using Physics2D.Util;
using System.Drawing;
using System.Windows.Forms;

namespace PhysicsVisualization
{
    public class Ball : MobileObject2D
    {
        private int _radius;
        private int _twoRadius;
        public Ball(int radius, double x, double y)
        {
            _radius = radius;
            _twoRadius = radius * 2;
            shape = new Circle(radius);
            setMass(5);
            setPose(new Double2D(x, y), new Angle(0));
            setCoefficientOfRestitution(1);
        }

        public void Draw(PaintEventArgs e)
        {
            Double2D position = this.getPosition();

            using (var brush = new SolidBrush(Color.Red))
            {
                e.Graphics.FillEllipse(brush, 
                    (float)(position.x - _radius), (float)(position.y - _radius), 
                    _twoRadius, _twoRadius);
            }
        }
    }
}
