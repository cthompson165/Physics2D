using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util.Matrix;

namespace Physics2D.Constraint
{

    /** Represents a constraint on objects' accelerations. Force constraints assume
     * legal positions and velocities and solve for legal accelerations.
     */
    public interface ForceConstraint
    {
        int GetConstraintRows();
        void setConstraintMatrices(int curConstraintRow, BlockSparseMatrix jacobianMatrix, BlockSparseMatrix jacobianDotMatrix, Vector constraintVector, Vector constraintDotVector);
        void addHolonomicConstraints();
    }
}
