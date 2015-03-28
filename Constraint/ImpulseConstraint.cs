using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util.Matrix;

namespace Physics2D.Constraint
{
    /** Represents a constraint on objects' velocities. Impulse constraints
 * are used to solve for legal velocities after the impulses are applied to 
 * the objects.
 */
    public interface ImpulseConstraint
    {
        int GetCollisionResponseRows();
        void setCollisionMatrices(int curConstraintRow, BorderedDiagonalIdentityMatrix collisionMatrix, Vector answerVector);
        void applyImpulses(int curAnswerRow, Vector answers);
    }
}
