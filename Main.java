import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Vector;


public class Main {

	public static void main(String[] args) throws FileNotFoundException {
		long startTime = System.nanoTime();
		
		String datafile = "10P5R1V_java.txt";
		
		File file = new File ("8P5R1V_java_results.txt");
		
		if (!file.exists()) {
			try { file.createNewFile(); 
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		PrintWriter pw = new PrintWriter(file);	
		Vector<Node> nodes = new Vector<Node>();
		Vector<Node> depot = new Vector<Node>();
		Vector<Node> pickupNodes = new Vector<Node>();
		Vector<Node> deliveryNodes = new Vector<Node>();
		Vector<Node> startDepots = new Vector<Node>();
		Vector<Vehicle> vehicles = new Vector<Vehicle>();
		Vector<Route> routes = new Vector<Route>();
		Vehicle vehicle = new Vehicle();
		
		
		InstanceData inputdata = new InstanceData(datafile);

		InputReader.inputReader(datafile, nodes, inputdata, depot, pickupNodes, deliveryNodes, startDepots, vehicles);

		PathBuilder builder;
		builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot,inputdata, pw, routes, vehicles);
		builder.BuildPaths(vehicle);
		
		//time
		long endTime = System.nanoTime();
		System.out.println("Took "+(endTime - startTime)/1000000 + " milli seconds"); 
		pw.println ("Took "+(endTime - startTime)/1000000 + " milli seconds");
		
		pw.close();
	}
}
