using Physics2D;
using Physics2D.Constraint;
using Physics2D.Util;
using System;
using System.Windows.Forms;

namespace Samples
{
    public partial class Pendulum : Form
    {
        private PhysicsEngine2D _engine;
        private Rod _rod;
        private PendulumAnchor _anchor;
        
        public Pendulum()
        {
            InitializeComponent();
            DoubleBuffered = true;

            // important - create the engine first. it sets up stuff
            // needed by the objects
            _engine = new PhysicsEngine2D();

            _anchor = new PendulumAnchor(new Double2D(100, 50), 5);
            _engine.register(_anchor);

            _rod = new Rod(20, 5, new Double2D(80, 50), 0, new Double2D(0, 0));
            _engine.register(_rod);

            var gravity = new Gravity();
            gravity.Affects(_rod);
            _engine.register(gravity);
            
            PinJoint pj = new PinJoint(new Double2D(100, 50), _anchor, _rod);
            _engine.register(pj);

            FormTimer.Interval = 30;
            FormTimer.Start();
        }

        private void Pendulum_Paint(object sender, PaintEventArgs e)
        {
            _rod.Draw(e);
            _anchor.Draw(e);
        }

        private void FormTimer_Tick(object sender, EventArgs e)
        {
            _engine.step();
            Refresh();
        }
    }
}
