using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.Util.Matrix
{
    public class Block
    {
        public int m;
        public int n;
        public int rowoffset;
        public int coloffset;
        public double[][] vals;

        public Block(int m, int n, int rowoffset, int coloffset, double[][] vals)
        {
            this.m = m;
            this.n = n;
            this.rowoffset = rowoffset;
            this.vals = vals;
            this.coloffset = coloffset;
        }
    }
}
