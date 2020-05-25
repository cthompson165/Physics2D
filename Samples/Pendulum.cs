using Physics2D;
using Physics2D.Util;
using System;
using System.Windows.Forms;

namespace Samples
{
    public partial class Pendulum : Form
    {
        private PhysicsEngine2D _engine;
        private Rod _rod;
        private Room _room;
        
        public Pendulum()
        {
            InitializeComponent();
            DoubleBuffered = true;

            // important - create the engine first. it sets up stuff
            // needed by the objects
            _engine = new PhysicsEngine2D();

            _rod = new Rod(10, 20, new Double2D(100, 100), .78, new Double2D(1, .8));
            _engine.register(_rod);

            _room = new Room(200, 200, 6);
            foreach (Wall wall in _room.Walls)
                _engine.register(wall);

            FormTimer.Interval = 30;
            FormTimer.Start();
        }

        private void Pendulum_Paint(object sender, PaintEventArgs e)
        {
            _rod.Draw(e);
            _room.Draw(e);
        }

        private void FormTimer_Tick(object sender, EventArgs e)
        {
            _engine.step();
            Refresh();
        }
    }
}
