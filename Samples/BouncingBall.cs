using Physics2D;
using Physics2D.Util;
using System.Windows.Forms;

namespace PhysicsVisualization
{
    public partial class BouncingBall : Form
    {
        private PhysicsEngine2D _engine;
        private Ball _ball;
        private Room _room;
        private Wall _wall;
        
        public BouncingBall()
        {
            InitializeComponent();
            DoubleBuffered = true;

            // important - create the engine first. it sets up stuff
            // needed by the objects
            _engine = new PhysicsEngine2D();

            _ball = new Ball(5, 40, 10);
            _ball.setVelocity(new Double2D(-1, .8));
            _engine.register(_ball);

            _room = new Room(100, 100, 6);
            foreach(Wall wall in _room.Walls)
                _engine.register(wall);

            FormTimer.Interval = 30;
            FormTimer.Start();
        }

        private void BouncingBall_Paint(object sender, PaintEventArgs e)
        {
            _ball.Draw(e);
            _room.Draw(e);
        }

        private void FormTimer_Tick(object sender, System.EventArgs e)
        {
            _engine.step();
            Refresh();
        }
    }
}
