using Physics2D.PhysicalObject;
using Physics2D.Util;
using System.Drawing;
using System.Windows.Forms;

namespace Samples
{
    public class Wall : StationaryObject2D
    {
        public Wall(Double2D position, int width, int height)
        {
            setCoefficientOfRestitution(1);
            setPose(position, new Angle(0));
            setShape(new Physics2D.ObjectShape.Rectangle(width, height, true));
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

            using (var brush = new SolidBrush(Color.Black))
            {
                e.Graphics.FillPolygon(brush, points);
            }
        }
    }
}
