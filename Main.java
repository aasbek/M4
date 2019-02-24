import java.util.ArrayList;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Vector;


public class Main {

	public static void main(String[] args) throws FileNotFoundException {
		long startTime = System.nanoTime();
		// TODO Auto-generated method stub
		String datafile = "7T7R1V_java.txt";
		
		File file = new File ("7R1V_java_results.txt");
		
		if (!file.exists()) {
			try { file.createNewFile(); 
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		PrintWriter pw = new PrintWriter(file);	
		
		//ArrayList<PickupNode> pickupnodes = new ArrayList<PickupNode>();
		//ArrayList<DeliveryNode> deliverynodes = new ArrayList<DeliveryNode>();
		Vector<Node> nodes = new Vector<Node>();
		Vector<Node> depot = new Vector<Node>();
		Vector<Node> pickupNodes = new Vector<Node>();
		Vector<Node> deliveryNodes = new Vector<Node>();
		
		InstanceData inputdata = new InstanceData(datafile);

		InputReader.inputReader(datafile, nodes, inputdata, depot, pickupNodes, deliveryNodes) ;

		//InstanceData.getDistance(nodes.get(1), nodes.get(5), inputdata);
		//InstanceData.getTime(nodes.get(4), nodes.get(5), inputdata);
		//System.out.println(nodes.get(0).location);
		//System.out.println(nodes.get(0).locationName);
		PathBuilder builder;
		builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot,inputdata, pw);
		builder.BuildPaths();
		//System.out.println(Node.getCorrespondingNode(nodes.get(2),nodes).number);
		
		
		//code
		long endTime = System.nanoTime();
		System.out.println("Took "+(endTime - startTime)/1000000 + " milli seconds"); 
		pw.println ("Took "+(endTime - startTime)/1000000 + " milli seconds");
		
		pw.close();
	}
}
