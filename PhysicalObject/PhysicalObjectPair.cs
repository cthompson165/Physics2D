using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.PhysicalObject
{

/** PhysicalObjectPair holds two physical objects that can
 * be put into a hashtable.
 */
public class PhysicalObjectPair 
    {
    public PhysicalObject2D c1;
    public PhysicalObject2D c2;
    public int hashcode;
        
    public PhysicalObjectPair(PhysicalObject2D c1, PhysicalObject2D c2)
        {
        this.c1 = c1;
        this.c2 = c2;
                
        hashcode = calcHashCode();
        }
        
    private int calcHashCode()
        {
        int index1 = c1.getIndex();
        int index2 = c2.getIndex();
                
        if (index2 > index1)
            {
            int temp = index2;
            index2 = index1;
            index1 = temp;
            }
                
        index1 += ~(index1 << 9);
        index1 ^=  (int)((uint)index1 >> 14);
        index1 +=  (index1 << 4);
        index1 ^=  (int)((uint)index1 >> 10);
        return index2 ^ index1;
        }
        
    // Overload hashCode and equals so we can put PhysicalObjectPairs 
    // hashtables
    public override int GetHashCode()
    {
        // TODO - need to override anything else?
        return hashcode;
    }

    public override bool Equals(object obj)
    {
        PhysicalObjectPair other = obj as PhysicalObjectPair;
        if (other == null)
            return false;
        else
            return hashcode == other.hashcode;
    }
    
    public bool equals(Object obj)
        {
        PhysicalObjectPair ap = (PhysicalObjectPair)obj;
        if ((ap.c1.getIndex() == this.c1.getIndex() && ap.c2.getIndex() == this.c2.getIndex())
            || (ap.c1.getIndex() == this.c2.getIndex() && ap.c2.getIndex() == this.c1.getIndex()))
            return true;
        else
            return false;
        }
    }

}
