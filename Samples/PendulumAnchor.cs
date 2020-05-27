using Physics2D.ObjectShape;
using Physics2D.PhysicalObject;
using Physics2D.Util;
using System.Drawing;
using System.Windows.Forms;

namespace Samples
{
    public class PendulumAnchor : StationaryObject2D
    {
        private double _radius;
        private double _twoRadius;

        public PendulumAnchor(Double2D pos, double radius)
        {
            _radius = radius;
            _twoRadius = radius * 2;

            setPose(pos, new Angle(0));
            setShape(new Circle(radius));
            setCoefficientOfRestitution(1);
        }

        public void Draw(PaintEventArgs e)
        {
            Double2D position = this.GetPosition();

            using (var brush = new SolidBrush(Color.Black))
            {
                e.Graphics.FillEllipse(brush,
                    (float)(position.x - _radius), (float)(position.y - _radius),
                    (float)_twoRadius, (float)_twoRadius);
            }
        }
    }
}
