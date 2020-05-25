using Physics2D.PhysicalObject;
using Physics2D.Util;
using System.Drawing;
using System.Windows.Forms;

namespace PhysicsVisualization
{
    public class Wall : StationaryObject2D
    {
        Physics2D.ObjectShape.Rectangle _rectangle;
        Double2D _topLeft;
        int _width;
        int _height;

        public Wall(int width, int height, int x, int y)
        {
            _topLeft = new Double2D(x - width / 2, y - height / 2);

            setPose(new Double2D(x, y), new Angle(0));
            _rectangle = new Physics2D.ObjectShape.Rectangle(width, height, true);
            setShape(_rectangle);

            _width = width;
            _height = height;
            setCoefficientOfRestitution(1);
        }

        public void Draw(PaintEventArgs e)
        {
            using (var brush = new SolidBrush(Color.Black))
            {
                e.Graphics.FillRectangle(brush, (float)_topLeft.x, (float)_topLeft.y, _width, _height);
            }
        }
    }
}
