using Physics2D.Util;
using System.Collections.Generic;
using System.Windows.Forms;

namespace Samples
{
    public class Room
    {
        public List<Wall> Walls { get; set; }

        public Room(int width, int height, int wallThickness)
        {
            int halfThickness = wallThickness / 2;
            int halfHeight = height / 2;
            int halfWidth = width / 2;
            int doubleThickness = wallThickness * 2;

            Walls = new List<Wall>();
            Walls.Add(new Wall(new Double2D(halfThickness, halfHeight), wallThickness, height));
            Walls.Add(new Wall(new Double2D(halfWidth, halfThickness), width - doubleThickness, wallThickness));
            Walls.Add(new Wall(new Double2D(width - halfThickness, halfHeight), wallThickness, height));
            Walls.Add(new Wall(new Double2D(halfWidth, height - halfThickness), width - doubleThickness, wallThickness));
        }

        public void Draw(PaintEventArgs e)
        {
            foreach (Wall wall in Walls)
                wall.Draw(e);
        }
    }
}
