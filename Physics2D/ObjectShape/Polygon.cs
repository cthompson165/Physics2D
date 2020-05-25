using System;
using Physics2D.Util;
using Physics2D.Util.Matrix;

namespace Physics2D.ObjectShape
{

    /** Polygons represents any convex multi-sided object. Convex means
     * that every angle measured between the insides of two edges must be
     * less than 180 degrees. To create a new polygon, create a new class
     * that inherits from polygon and implement the abstract functions based
     * on the descriptions given below. Use rectangle as a reference.
     */
    public abstract class Polygon : Shape
    {
        protected double _maxXDistanceFromCenter;
        protected double _maxYDistanceFromCenter;

        protected DenseMatrix _vertices;
        protected DenseMatrix _edges;
        protected DenseMatrix _normals;

        private Double2D[] verticesCache;
        private Double2D[] edgesCache;
        private Double2D[] normalsCache;

        private bool _vertCacheValid;
        private bool _edgesCacheValid;
        private bool _normalsCacheValid;

        public Polygon(bool stationary) : base(stationary)
        {
            _vertCacheValid = false;
            _edgesCacheValid = false;
            _normalsCacheValid = false;
        }

        public Polygon() : this(false)
        {
        }

        protected DenseMatrix _scale = new DenseMatrix(
            Array2D.GetArray(new double[,] {{0, 0, 0},
             {0, 0, 0},
             {0, 0, 1}}));

        /** Returns a list of the vertexes in a clockwise direction with
         * positive Y pointing up (vs. pointing down as on a computer screen) */
        public Double2D[] GetVertices()
        {
            if (_vertCacheValid)
                return verticesCache;
            else
            {
                DenseMatrix rotTranDenseMatrix = Polygon.RotationTranslationMatrix2D(this.getOrientation().radians, this.getPosition());
                DenseMatrix rotVertices = rotTranDenseMatrix.times(this._vertices);
                Double2D[] verts = new Double2D[rotVertices.n];
                for (int i = 0; i < rotVertices.n; i++)
                    verts[i] = new Double2D(rotVertices.vals[0][i], rotVertices.vals[1][i]);
                verticesCache = verts;

                if (_stationary)
                    _vertCacheValid = true;

                return verticesCache;
            }
        }

        /** Returns a list of the normalized edges in a clockwise direction. The
         * starting vertex of the edge must be the vertex with the corresponding index.
         * For example, edge 0 goes from vertex 0 to vertex 1 */
        public Double2D[] GetEdges()
        {
            if (_edgesCacheValid)
                return edgesCache;
            else
            {
                DenseMatrix rotDenseMatrix = Polygon.RotationTranslationMatrix2D(this.getOrientation().radians, new Double2D(0, 0));
                DenseMatrix rotEdges = rotDenseMatrix.times(this._edges);
                Double2D[] result = new Double2D[rotEdges.n];
                for (int i = 0; i < rotEdges.n; i++)
                    result[i] = new Double2D(rotEdges.vals[0][i], rotEdges.vals[1][i]);

                edgesCache = result;
                if (_stationary) _edgesCacheValid = true;

                return edgesCache;
            }
        }

        /** Returns a list of the unit normals in a clockwise direction. Each 
         * normal's index must correspond to its edge's index. For example,
         * the normal to edge 0 must have index 0 */
        public Double2D[] GetNormals()
        {
            if (_normalsCacheValid)
                return normalsCache;
            else
            {
                DenseMatrix rotDenseMatrix = Polygon.RotationTranslationMatrix2D(this.getOrientation().radians, new Double2D(0, 0));
                DenseMatrix rotNormals = rotDenseMatrix.times(this._normals);
                Double2D[] result = new Double2D[rotNormals.n];
                for (int i = 0; i < rotNormals.n; i++)
                    result[i] = new Double2D(rotNormals.vals[0][i], rotNormals.vals[1][i]);

                if (_stationary)
                {
                    normalsCache = result;
                    _normalsCacheValid = true;
                }
                return result;
            }
        }

        // force polygons to set stuff up correctly
        /** Set up the vertices DenseMatrix. The vertices DenseMatrix gives the
         * homogenous coordinates of the vertices centered around 0,0.
         * They must be defined clockwise. */
        abstract public void InitVertices();

        /** Set up the edges DenseMatrix. The edges DenseMatrix gives the edge 
         * vectors in homogenous coordinates between the 
         * vertices. The number of the edge should
         * correspond to the number of the vertex from which the vector
         * points (i.e. edge 0 points from vertex 0 to vertex 1). Edges
         * should point clockwise and must be normalized. */
        abstract public void InitEdges();

        /** Set up the normals DenseMatrix. The normals DenseMatrix gives 
         * the normal vectors in homogenous coordinates. The numbers of
         * the normals must correspond to the edge to which the normal
         * is perpendicular to (i.e. normal 0 points out from edge 0). 
         * The normals must be of unit length */
        abstract public void InitNormals();

        public override double GetMaxXDistanceFromCenter()
        {
            return this._maxXDistanceFromCenter;
        }

        public override double GetMaxYDistanceFromCenter()
        {
            return this._maxYDistanceFromCenter;
        }

        /** Returns a DenseMatrix in homogenous coordinates to rotate a 2 dimensional 
         * rigid body given the angle theta (in radians)
         */
        public static DenseMatrix RotationMatrix2D(double theta)
        {
            double cosTheta = Math.Cos(theta);
            double sinTheta = Math.Sin(theta);

            double[][] vals =
                Array2D.GetArray(new double[,] {{cosTheta, -sinTheta},
                 {sinTheta, cosTheta},
                 {0, 1}});

            return new DenseMatrix(vals);
        }

        /** Returns a DenseMatrix in homogenous coordinates to rotate and translate 
         * a 2 dimensional rigid body given the angle theta (in radians) and a translation vector
         */
        public static DenseMatrix RotationTranslationMatrix2D(double theta, Double2D translation)
        {
            double cosTheta = Math.Cos(theta);
            double sinTheta = Math.Sin(theta);

            double[][] vals =
                Array2D.GetArray(new double[,] {{cosTheta, -sinTheta, translation.x},
                 {sinTheta, cosTheta, translation.y},
                 {0, 0, 1}});

            return new DenseMatrix(vals);
        }

        /** Returns a row of the DenseMatrix rounded to integers */
        public static int[] GetRow(int row, DenseMatrix mat)
        {
            int cols = mat.n;
            int[] result = new int[cols];
            for (int i = 0; i < cols; i++)
                result[i] = (int)Math.Round(mat.vals[row][i]);
            return result;
        }
    }
}
