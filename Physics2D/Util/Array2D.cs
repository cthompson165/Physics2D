using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.Util
{
    public class Array2D
    {
        public static double[][] GetArray(double[,] array)
        {
            int rows = array.GetLength(0);
            int cols = array.GetLength(1);
            double[][] jaggedArray = new double[rows][];

            for (int row = 0; row < rows; row++)
            {
                jaggedArray[row] = new double[cols];
                for (int col = 0; col < cols; col++)
                {
                    jaggedArray[row][col] = array[row, col];
                }
            }
            return jaggedArray;
        }
    }
}
