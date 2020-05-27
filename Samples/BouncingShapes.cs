using Physics2D;
using Physics2D.Util;
using System.Windows.Forms;

namespace Samples
{
    public partial class BouncingShapes : Form
    {
        private PhysicsEngine2D _engine;
        private Ball _ball;
        private Rod _rod;
        private Room _room;
        
        public BouncingShapes()
        {
            InitializeComponent();
            DoubleBuffered = true;

            // important - create the engine first. it sets up stuff
            // needed by the objects
            _engine = new PhysicsEngine2D();

            _rod = new Rod(10, 30, new Double2D(100, 100), .78, new Double2D(1, .8));
            _engine.register(_rod);

            _ball = new Ball(10, 70, 30);
            _ball.SetVelocity(new Double2D(-1, .8));
            _engine.register(_ball);

            _room = new Room(200, 200, 6);
            foreach(Wall wall in _room.Walls)
                _engine.register(wall);

            FormTimer.Interval = 30;
            FormTimer.Start();
        }

        private void BouncingBall_Paint(object sender, PaintEventArgs e)
        {
            _ball.Draw(e);
            _room.Draw(e);
            _rod.Draw(e);
        }

        private void FormTimer_Tick(object sender, System.EventArgs e)
        {
            _engine.step();
            Refresh();
        }
    }
}
