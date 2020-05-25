using System;
using Physics2D.Util;
using Physics2D.Util.Matrix;

namespace Physics2D.ObjectShape
{
    /** Rectangle implementation of Polygon
     */
    public class Rectangle : Polygon 
    {
        private double _width;
        private double _height;
        
        public Rectangle(double width, double height) : this(width, height, false)
        {
        }
        
        public Rectangle(double width, double height, bool stationary) : base (stationary)
        {
            this._width = width;
            this._height = height;
                
            InitVertices();
            InitEdges();
            InitNormals();
        }
        
        /** Set up the vertices matrix in homogenous coordinates */
        public override void InitVertices()
        {
            // going clockwise starting from the bottom left
            double[][] verts = 
                Array2D.GetArray(new double[,] {{-_width / 2, -_width / 2, _width / 2, _width / 2},
                     {-_height / 2, _height / 2, _height / 2, -_height / 2},
                     {1, 1, 1, 1}});
            _vertices = new DenseMatrix(verts);
        }

        /** Set up the edges matrix */
        public override void InitEdges()
        {
            _edges = new DenseMatrix(3, 4);
            Double2D curEdge;
            for (int i = 0; i < 3; i++)
            {
                // edges must be normalized
                curEdge = new Double2D(_vertices.vals[0][i+1], _vertices.vals[1][i+1]).subtract(new Double2D(_vertices.vals[0][i], _vertices.vals[1][i])).normalize();
                _edges.vals[0][i] = curEdge.x;
                _edges.vals[1][i] = curEdge.y;
                _edges.vals[2][i] = 1;
            }
                
            // Get the last one
            curEdge = new Double2D(_vertices.vals[0][0], _vertices.vals[1][0]).subtract(new Double2D(_vertices.vals[0][3], _vertices.vals[1][3])).normalize();
            _edges.vals[0][3] = curEdge.x;
            _edges.vals[1][3] = curEdge.y;
            _edges.vals[2][3] = 1;
        }
                
        /** Set up the normals matrix */
        public override void InitNormals()
        {
            // Just rotate the edges 90 degrees
            _normals = new DenseMatrix(3,4);
            _normals.vals[0][0] = -_edges.vals[1][0];
            _normals.vals[1][0] = _edges.vals[0][0];
            _normals.vals[2][0] = 1;
                
            _normals.vals[0][1] = -_edges.vals[1][1];
            _normals.vals[1][1] = _edges.vals[0][1];
            _normals.vals[2][1] = 1;
                
            _normals.vals[0][2] = -_edges.vals[1][2];
            _normals.vals[1][2] = _edges.vals[0][2];
            _normals.vals[2][2] = 1;
                
            _normals.vals[0][3] = -_edges.vals[1][3];
            _normals.vals[1][3] = _edges.vals[0][3];
            _normals.vals[2][3] = 1;
        }
        
        public double GetWidth()
        {
            return _width;
        }
        public double GetHeight()
        {
            return _height;
        }

        /////////////////////////////////////////////////////////////////
        // These functions are used by the broad phase Collision detection 
        // logic
        /////////////////////////////////////////////////////////////////
        
        /** Calculate the max distance a point can be from the center of the object.
            If the object is stationary, we can give more exact values. Remember
            that stationary objects can have orientation, though, so it can be more
            complicated than width / 2 and height / 2 */
        public override void CalcMaxDistances(bool mobile)
        {
            if (mobile)
            {
                _maxXDistanceFromCenter = Math.Sqrt(this._width * this._width + this._height * this._height) / 2;
                _maxYDistanceFromCenter = _maxXDistanceFromCenter;
            }
            else
            {
                // Get the max distances from the center for a stationary object
                // taking into account its orientation
                DenseMatrix rotated = Polygon.RotationTranslationMatrix2D(this.getOrientation().radians, new Double2D(0, 0)).times(this._vertices);
                        
                _maxXDistanceFromCenter = 0;
                _maxYDistanceFromCenter = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (rotated.vals[0][i] > _maxXDistanceFromCenter)
                        _maxXDistanceFromCenter = rotated.vals[0][i];
                    if (rotated.vals[1][i] > _maxYDistanceFromCenter)
                        _maxYDistanceFromCenter = rotated.vals[1][i];
                }
            }
                
            // Adding padding will help to give the narrow phase
            // collision logic a chance to run before objects penetrate.
            // TODO: if we keep the dimension reduction strategy for broad phase,
            // make this customizable so users can tune their own applications.
            _maxXDistanceFromCenter += .1;
            _maxYDistanceFromCenter += .1;
        }
        
        /** Calculate the mass moment of intertia of the object.
         * This can be done through integration, or by finding a precomputed
         * equation for the polygon being defined 
         */
        public override double GetMassMomentOfInertia(double mass)
        {
            return (mass / 12) * (_width * _width + _height * _height);
        }
    }
}
