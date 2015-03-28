using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util.Matrix;
using Physics2D;
using Physics2D.PhysicalObject;
using Physics2D.CollisionDetection;
using Physics2D.Util;

namespace Physics2D.Constraint
{


/** The ConstraintEngine solves for constraint forces and impulses */
public class ConstraintEngine 
    {
    // Force constraint vectors
    private Vector constraintVector;
    private Vector constraintDotVector;
    private BlockSparseMatrix jacobianMatrix;
    private BlockSparseMatrix jacobianDotMatrix;
    private Vector qDotVector;
        
    // Impulse constraint vectors
    private BorderedDiagonalIdentityMatrix collisionResponseMatrix;
    private Vector collisionResponseAnswersVector;
    private int constraintRows;
    private int collisionRows;
    private int collisionResponseRows;

    private HashSet<PhysicalObjectPair> noCollisions;
        
    private Bag constraints;
    private Bag collisions;
        
    private static int debugCounter = 0;
        
    private PhysicsState physicsState = null;
        
    private const double ZERO_VELOCITY = 0.000001;
        
    private static ConstraintEngine instance = null;
        
    public static ConstraintEngine getInstance()
        {
        if (instance == null)
            instance = new ConstraintEngine();
        return instance;
        }
        
    public static ConstraintEngine reset()
        {
        instance = new ConstraintEngine();
        return instance;
        }
        
    private ConstraintEngine()
        {
        physicsState = PhysicsState.getInstance();
        constraintRows = 0;
        constraints = new Bag();
                
        collisionRows = 0;
        collisions = new Bag();

        noCollisions = new HashSet<PhysicalObjectPair>();
        }

    /** Turns off collisions for a pair of objects
     */
    public void setNoCollisions(PhysicalObject2D c1, PhysicalObject2D c2)
        {
        PhysicalObjectPair pair = new PhysicalObjectPair(c1, c2);
        noCollisions.Add(pair);
        }
        
    /** Turns collisions for a pair of objects back on
     */
    public void removeNoCollisions(PhysicalObject2D c1, PhysicalObject2D c2)
        {
        PhysicalObjectPair pair = new PhysicalObjectPair(c1, c2);
        noCollisions.Remove(pair);
        }
        
    /** Tests whether collisions between a pair of objects is currently turned off 
     */
    public bool testNoCollisions(PhysicalObject2D c1, PhysicalObject2D c2)
        {
        PhysicalObjectPair pair = new PhysicalObjectPair(c1, c2);
        return noCollisions.Contains(pair);
        }
        
    /** Registers a force constraint with the constraint engine
     */
    public void registerForceConstraint(ForceConstraint constraint)
        {
        constraintRows += constraint.GetConstraintRows();
        constraints.add(constraint);
        // set up the resting contact constraint
        constraint.addHolonomicConstraints();
        }
        
    /** Registers an impulse constraint with the constraint engine */
    public void registerImpulseConstraint(ImpulseConstraint collision)
        {
        collisionResponseRows += collision.GetCollisionResponseRows();
        collisions.add(collision);
        }
        
    /** Un-registers a force constraint with the constraint engine */
    // TODO - need to remove holonomic constraints
    public void unRegisterForceConstraint(ForceConstraint con)
        {
        constraints.remove(con);
        constraintRows -= con.GetConstraintRows();
        }
        
    /** Un-registers an impulse constraint with the constraint engine */
    public void unRegisterImpulseConstraint(ImpulseConstraint con)
        {
        collisionResponseRows -= ((ImpulseConstraint)con).GetCollisionResponseRows();
        collisions.remove(con);
        }
        
    /** Calculates the constraint forces based on the constraints and external forces
     * currently in the system
     */
    public Vector calculateConstraintForces(Vector externalForcesVector)
        {
        setMatrices();
                
        double ks = .3;
        double kd = .3;

        DiagonalMatrix W = physicsState.getMassInverseMatrix();
        Vector feedback = constraintVector.times(ks).plus(constraintDotVector.times(kd));
        Vector b = jacobianDotMatrix.times(qDotVector.times(-1)).minus(jacobianMatrix.times(W.times(externalForcesVector))).minus(feedback);
                        
        Vector lambda = new Vector(b.m);
        DiagonalMatrix A_t = new DiagonalMatrix(b.m);
        for (int i = 0; i < b.m; i++)
            A_t.vals[i] = 1;
                
        lambda = BlockSparseMatrix.solveBiConjugateGradient(jacobianMatrix, W, A_t, b, lambda, W.m * 2, 1E-10);
                
        Vector Qhat = jacobianMatrix.transposeTimes(lambda);
        return Qhat;
        }
        
    /** Solves for and adds collision responses to the colliding objects */
    public void addCollisionResponses(Bag collidingList)
        {
        //physicsState.lcp.contacts.clear();
        for (int i = 0; i < collidingList.numObjs; i++)
            {
            CollisionPair pair = (CollisionPair)collidingList.objs[i];
                
            Collision col = new Collision();
            PhysicalObject2D collidePoly1 = (PhysicalObject2D)pair.c1;
            PhysicalObject2D collidePoly2 = (PhysicalObject2D)pair.c2;
                        
            col.AddPhysicalObject(collidePoly1, pair.getColPoint1());
            col.AddPhysicalObject(collidePoly2, pair.getColPoint2());
            col.setColNormal(pair.getNormal());
            col.setRelVel(pair.getRelativeVelocity());
                        
            if (pair.getSticky())
                col.setSticky();
                        
            //bool sticky = false;
            //if (pair.relVel.dotProduct(pair.normal) > -STICKY_THRESHOLD)
            //{
            //      sticky = true;
            //      col.setSticky();
            //}
                        
            this.registerImpulseConstraint(col);
            this.setCollisionMatrices();
                
            Vector answerCT = new Vector(collisionResponseAnswersVector.m);
                        
            try
                {
                // First try with the id matrix as the preconditioner
                answerCT = BorderedDiagonalIdentityMatrix.solveBiConjugateGradient(collisionResponseMatrix, collisionResponseAnswersVector, answerCT, collisionResponseMatrix.m * 2, 1E-5, false);
                }
            catch(Exception e)
                {
                try
                    {
                    // If that fails, try again with ILU decomp
                    answerCT = BorderedDiagonalIdentityMatrix.solveBiConjugateGradient(collisionResponseMatrix, collisionResponseAnswersVector, answerCT, collisionResponseMatrix.m * 2, 1E-5, true);
                    }
                                
                catch(Exception e2)
                    {
                    // In the worst case, solve it using dense matrices
                    answerCT = new Vector(collisionResponseMatrix.getDenseMatrix().solve(collisionResponseAnswersVector.getDenseMatrix()));
                    }
                }
                        
            addCalculatedResponses(answerCT);
            this.unRegisterImpulseConstraint(col);
                        
            // Add the pair to the resting list if they are stuck. Otherwise,
            // (separating) clear the features
            double relVelNorm = pair.getRelativeVelocity(); 
            if (relVelNorm > -ZERO_VELOCITY && relVelNorm < ZERO_VELOCITY)
                {
                //physicsState.lcp.addContact(pair.c1, pair.c2, pair.normal, new Double2D(pair.relVel.x, pair.relVel.y), pair.getColPoint1(), pair.colPoint2);
                }
                        
            // Clear this pair's features
            pair.clear();
                        
            }
        }
        
    private void setMatrices()
        {
        Vector stateVector = physicsState.getStateVector();
        int DOF = stateVector.m / 2;
                
        int conRows = this.constraintRows;
                
        constraintVector = new Vector(conRows);
        constraintDotVector = new Vector(conRows);
        jacobianMatrix = new BlockSparseMatrix(conRows, DOF);
        jacobianDotMatrix = new BlockSparseMatrix(conRows, DOF);
                
        qDotVector = new Vector(DOF);
        for (int i = 0; i < DOF; i++)
            qDotVector.vals[i] = stateVector.vals[i + DOF];
                
        int curConstraintRow = 0;
                
        // Fill in matrices based on the individual constraint matrixes
        Bag constraints = this.constraints;
        for (int i = 0; i < constraints.numObjs; i++)
            {       
            ForceConstraint con = (ForceConstraint)constraints.objs[i];
            con.setConstraintMatrices(curConstraintRow, jacobianMatrix, jacobianDotMatrix, constraintVector, constraintDotVector);
            curConstraintRow += con.GetConstraintRows();
            }
        }
        
    private void setCollisionMatrices()
        {
        Vector stateVector = physicsState.getStateVector();
        int DOF = stateVector.m / 2;
        int colResponseMatrixSize = DOF + this.collisionResponseRows;
                
        collisionResponseMatrix = new BorderedDiagonalIdentityMatrix(colResponseMatrixSize, colResponseMatrixSize - DOF);
        collisionResponseAnswersVector = new Vector(colResponseMatrixSize);
                
        // The first half of the answers vector should be the velocities of the objects
        for (int i = 0; i < DOF; i++)
            collisionResponseAnswersVector.vals[i] = stateVector.vals[i + DOF];
                
        int curCollisionResponseRow = DOF;
                
        // Fill in matrices based on the individual constraint matrixes
        Bag collisions = this.collisions;
        for (int i = 0; i < collisions.numObjs; i++)
            {
            ImpulseConstraint col = (ImpulseConstraint)collisions.objs[i];
            col.setCollisionMatrices(curCollisionResponseRow, collisionResponseMatrix, collisionResponseAnswersVector);
            curCollisionResponseRow += col.GetCollisionResponseRows();
            }
        }
        
    private void addCalculatedResponses(Vector answers)
        {
        Vector stateVector = physicsState.getStateVector();
                
        int DOF = stateVector.m / 2;
        int colResponseMatrixSize = DOF + this.collisionResponseRows;
                
        int curAnswerRow = DOF;
                
        Bag collisions = this.collisions;
        for (int i = 0; i < collisions.numObjs; i++)
            {
            ImpulseConstraint col = (ImpulseConstraint)collisions.objs[i];
            col.applyImpulses(curAnswerRow, answers);
            curAnswerRow += 2;
            }
        }
    }

}
