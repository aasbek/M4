//import java.util.Vector;
import java.util.Comparator;

public class UnprocessedComparator implements Comparator<Label> {
	
	//PriorityQueue<Label> pQueue = new PriorityQueue<Label>(10, new Comparator<Label>();
//@Override
    public int compare(Label label1, Label label2)
    {
        // Assume neither string is null. Real code should
        // probably be more robust
        // You could also just return x.length() - y.length(),
        // which would be more efficient.
        if (label1.time < label2.time)
        {
            return -1;
        }
        else if (label1.time > label2.time)
        {
            return 1;
        }
        return 0;
    }	
}
