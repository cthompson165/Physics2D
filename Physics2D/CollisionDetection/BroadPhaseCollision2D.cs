using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Physics2D.Util;
using Physics2D.PhysicalObject;

namespace Physics2D.CollisionDetection
{

/** BroadPhaseCollision2D performs "broad phase" collision detection.
 * It's goal is to quickly determine which objects are near enough together
 * to justify the cost of more exact "narrow phase" collision detection. 
 * It uses a dimension reduction strategy that determines if objects 
 * are simulateously overlapping on the X and Y axes. If so, the pair
 * is added to the "ActiveList" for later processing by the narrow phase
 * logic - Collision2D 
 */

// See http://www.cs.jhu.edu/~cohen/Publications/icollide.pdf for more information
// about dimension reduction for broad phase collision detection.
class BroadPhaseCollision2D
    {
    private const int X_DIM = 0;
    private const int Y_DIM = 1;
        
    private const double ENDPOINT_PADDING = .5;
        
    private Bag[] arDimEPBags = new Bag[2];
    private Bag arOS; // Keeps track of the overlaps
    private HashSet<CollisionPair> activeList; // Objects which could currently be colliding

    /** OverlapStatus tracks whether or not an object is overlapping in the
     * X and Y dimensions.
     */
    private class OverlapStatus
        {
        public bool[] dimension = new bool[2];
        }

    /** EndPoints are used in the dimension lists to represent the start and end
     * of each object. Each object has 2 EndPoints in both dimensions. 
     */
    private class EndPoint
    {
        public bool start; // false if this is EndPoint further from the origen
        public PhysicalObject2D objCol; // The object this EndPoint represents
        public double offset; // How far this EndPoint is from the center of the object
        public int dimension; // What dimension list this EndPoint is in

        public EndPoint(PhysicalObject2D objCol, bool start, int dimension)
        {
            this.start = start;
            this.objCol = objCol;
            if (start)
                this.offset = -1 * (getMaxDistanceFromCenter(objCol, dimension)) - ENDPOINT_PADDING;
            else
                this.offset = getMaxDistanceFromCenter(objCol, dimension) + ENDPOINT_PADDING;

            this.dimension = dimension;
        }

        private double getMaxDistanceFromCenter(PhysicalObject2D objCol, int dimension)
        {
            if (dimension == X_DIM)
                return objCol.getShape().GetMaxXDistanceFromCenter();
            else
                return objCol.getShape().GetMaxYDistanceFromCenter();
        }

        // An EndPoint's position is its object's position plus its offset
        public double getPos()
        {
            if (dimension == X_DIM)
                return objCol.getPosition().x + offset;
            else
                return objCol.getPosition().y + offset;
        }

        public override string ToString()
        {
            return string.Format("{0} {1}: {2} - {3}", objCol.GetType(),
                dimension == X_DIM ? "X" : "Y", getPos(), start ? "Start" : "End");
        }
    }

    public BroadPhaseCollision2D()
    {
        arOS = new Bag();

        arDimEPBags[0] = new Bag();
        arDimEPBags[1] = new Bag();

        activeList = new HashSet<CollisionPair>();
    }

    /** Returns a HashSet containing a list of the object pairs that
     * could possibly be colliding.
     */
    public HashSet<CollisionPair> getActiveList()
        {
        return activeList;
        }

    /** Register an object for collision detection. */ 
    public void register(PhysicalObject2D objCol)
        {
        // Create and add the end points
        EndPoint epStart = new EndPoint(objCol, true, 0);
        EndPoint epEnd = new EndPoint(objCol, false, 0);

        // X endpoint list
        arDimEPBags[0].add(epStart);
        arDimEPBags[0].add(epEnd);

        epStart = new EndPoint(objCol, true, 1);
        epEnd = new EndPoint(objCol, false, 1);

        // Y endpoint list
        arDimEPBags[1].add(epStart);
        arDimEPBags[1].add(epEnd);

        // Keep an lower diagonal matrix of overlap status
        // objects. The column number is the current object 
        // and the row number is the object with which this one
        // is being compared. 
        int index = objCol.getIndex();
        if (index == 0)
            arOS.add(null);
        else
            {
            arOS.add(new OverlapStatus[index]);

            // initialize the overlap status array
            for (int i = 0; i < index; i++)
                {
                OverlapStatus[] arTmp = (OverlapStatus[])arOS.objs[index];
                arTmp[i] = new OverlapStatus();
                arTmp[i].dimension[0] = false;
                arTmp[i].dimension[1] = false;
                }
            }

        // Sort the arrays (which initializes the overlap statuses)
        insertionSort(0);
        insertionSort(1);
        }

    // TODO - we can probably do this without wiping out the whole thing.
    public void clear()
    {
        arOS = new Bag();

        arDimEPBags[0] = new Bag();
        arDimEPBags[1] = new Bag();

        activeList = new HashSet<CollisionPair>();
    }

    /** Run through all objects to see if they are colliding */
    public void testCollisions()
        {
        // Sort the arrays. This should be pretty efficient
        // since the lists should be almost sorted
        insertionSort(0);
        insertionSort(1);
        }

    ////////////////////////////////////////////////////
    // INSERTION SORT
    ////////////////////////////////////////////////////
        
    // Loop through the list of endpoints starting at 0.
    // If the current endpoint's position is less than the 
    // previous endpoint's position, move the current endpoint
    // back in the list until it is sorted. While moving back,
    // update the overlap status arrays appropriately.
    private void insertionSort(int dimension)
        {               
        Bag arList = arDimEPBags[dimension];
        int curEPIndex = 0;
                
        // loop through the list
        while (curEPIndex < arList.numObjs)
            {
            if (curEPIndex > 0)
                {
                int prevEPIndex = curEPIndex - 1;
                                
                EndPoint curEP = (EndPoint)arList.objs[curEPIndex];
                EndPoint prevEP = (EndPoint)arList.objs[prevEPIndex];
                if (prevEP.getPos() > curEP.getPos())
                    orderedInsert(arList, curEPIndex - 1, dimension);
                }
                                
            curEPIndex += 1;
            }
        }
        
    // Swaps the EndPoint down the list until it is ordered correctly
    private void orderedInsert(Bag arList, int seekEPIndex, int dimension)
        {
        // go back until we find an EP less than this one       
        while (seekEPIndex >= 0 
            && ((EndPoint)arList.objs[seekEPIndex]).getPos() > ((EndPoint)arList.objs[seekEPIndex + 1]).getPos())
            {               
            // Check if overlap status should change based on the
            // end point that is being reordered and the end point 
            // that it just passed
            checkOverlaps((EndPoint)arList.objs[seekEPIndex + 1], (EndPoint)arList.objs[seekEPIndex], dimension);
                        
            // Swap down
            Object temp = arList.objs[seekEPIndex + 1];
            arList.objs[seekEPIndex + 1] = arList.objs[seekEPIndex];
            arList.objs[seekEPIndex] = temp;
                
            seekEPIndex--;
            }
        }
        
    // Update the overlap statuses the EndPoint and the one it is being
    // swapped with.
    private void checkOverlaps(EndPoint curEP, EndPoint prevEP, int dimension)
        {
        // Higher index must be first since we have an upper triangular matrix
        OverlapStatus objOS;

        if (curEP.objCol.getIndex() > prevEP.objCol.getIndex())
            {
            OverlapStatus[] arTmp = (OverlapStatus[])arOS.objs[curEP.objCol.getIndex()];
            objOS = arTmp[prevEP.objCol.getIndex()];
            }
        else
            {
            OverlapStatus[] arTmp = (OverlapStatus[])arOS.objs[prevEP.objCol.getIndex()];
            objOS = arTmp[curEP.objCol.getIndex()];
            }

        // See if we create overlaps
        if (curEP.start == true)
            {
            // curEP is a start point, since we have moved it behind something, we are
            // now overlapping in this dimension
            objOS.dimension[dimension] = true;

            // see if both dimensions are true
            if (objOS.dimension[0] && objOS.dimension[1])
                {
                if (!(curEP.objCol is StationaryObject2D && prevEP.objCol is StationaryObject2D))
                    {
                    CollisionPair pair = new CollisionPair(curEP.objCol, prevEP.objCol);
                    activeList.Add(pair);
                    }
                }
            }
        else
            {
            // This is the end point, set the overlap status to false if we move behind the
            // start of something else. Since we are never setting to true, don't ever test
            // for collisions
            if (prevEP.start == true)
                {
                objOS.dimension[dimension] = false;
                                
                // Remove from the active list (does nothing if not already on)
                CollisionPair pair = new CollisionPair(curEP.objCol, prevEP.objCol);
                bool removed = activeList.Remove(pair);
                }
            }
        }
    }

}
