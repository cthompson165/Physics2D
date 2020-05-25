using System.Collections.Generic;
using System.Windows.Forms;

namespace PhysicsVisualization
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
            Walls.Add(new Wall(wallThickness, height, halfThickness, halfHeight));
            Walls.Add(new Wall(width - doubleThickness, wallThickness, halfWidth, halfThickness));
            Walls.Add(new Wall(wallThickness, height, width - halfThickness, halfHeight));
            Walls.Add(new Wall(width - doubleThickness, wallThickness, halfWidth, height - halfThickness));
        }

        public void Draw(PaintEventArgs e)
        {
            foreach (Wall wall in Walls)
                wall.Draw(e);
        }
    }
}
