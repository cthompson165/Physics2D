using System;
using System.Collections;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics2D.Util
{
    
    public class Bag
    {
        private const int sizeIncrement = 100;

        public object[] objs;
        public int numObjs;

        public Bag()
        {
            objs = new object[sizeIncrement];
        }

        public void add(object value)
        {
            objs[numObjs++] = value;
            if (numObjs >= objs.Length)
            {
                var temp = new object[objs.Length + sizeIncrement];
                Array.Copy(objs, temp, objs.Length);
                objs = temp;
            }
        }

        public void clear()
        {
            objs = new object[100];
            numObjs = 0;
        }
       
        public int size()
        {
            return numObjs;
        }

        public void remove(object value)
        {
            int index = -1;
            for (int i = 0; i < numObjs; i++)
            {
                if (objs[i] == value)
                {
                    index = i; 
                    break;
                }
            }
            if (index > -1)
            {
                removeAt(index);
            }
        }

        public void removeAt(int index)
        {
            for (int i = index; i < numObjs - 1; i++)
            {
                objs[i] = objs[i + 1];
            }

            numObjs--;
        }
    }
}
