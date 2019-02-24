import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
// import java.util.ArrayList;
// import java.util.Arrays;
import java.util.Vector;


public class InputReader {
	public static void inputReader(String datafile,  Vector<Node> nodes, InstanceData inputdata, Vector<Node> depot, Vector<Node> pickupNodes, Vector<Node> deliveryNodes) {
		try {
			File file = new File(datafile);
			//File file2 = file.getAbsoluteFile();
			FileReader reader = new FileReader(file);
			BufferedReader fr = new BufferedReader(reader);
			
			//Given each node a number, corresponding to the place in the vector in the datafile
			//Assigning the type of node to each node
			String line = fr.readLine();
			//System.out.println(line);
			String[] list1 =line.split(",");
			for (int i = 1; i < list1.length; i++) {
				int number = Integer.parseInt(list1[i].trim());
				//System.out.println(number);
				Node hello = new Node(number);
				nodes.add(hello);
				if(number == 0 || number == 1) {
					hello.type = "Depot";
					depot.add(hello);
				}
				else if((number%2)==0) {
					hello.type = "PickupNode";
					pickupNodes.add(hello);
				}
				else {
					hello.type = "DeliveryNode";
					deliveryNodes.add(hello);
				}
			}
			
			//for(int i=0; i <nodes.size(); i++){
			//	System.out.println(nodes.get(i).number);
			//}
			
			//Volume cap
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.volumeCap = Integer.parseInt(list1[1].trim());
			
			
			//Weight Cap
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.weightCap = Integer.parseInt(list1[1].trim());	
			
			//Early Time Window
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				float number = Float.parseFloat(list1[i].trim());
				nodes.get(i-1).earlyTimeWindow = number;
				//System.out.println(number);
			}
			//System.out.println(nodes.get(2).earlyTimeWindow);
			
			//Late time window
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				float number = Float.parseFloat(list1[i].trim());
				nodes.get(i-1).lateTimeWindow = number;
			}
			
			//Assigning weight to each node
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				int number = Integer.parseInt(list1[i].trim());
				pickupNodes.get(i-1).weight = number;
				deliveryNodes.get(i-1).weight = number;
				//System.out.println(number);
			}
			
			//Assigning volume to each node
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				int number = Integer.parseInt(list1[i].trim());
				pickupNodes.get(i-1).volume = number;
				deliveryNodes.get(i-1).volume = number;
			}
		
			
			//Assigning locations to each pickupnode
			line = fr.readLine();
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				int number = Integer.parseInt(list1[i].trim());
				pickupNodes.get(i-1).location = number;
				pickupNodes.get(i-1).getLocation(number);
			}
			
			//Assigning location to each delivery node
			line = fr.readLine();
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				int number = Integer.parseInt(list1[i].trim());
				deliveryNodes.get(i-1).location = number;
				deliveryNodes.get(i-1).getLocation(number);
			}
			
			// Assigning location to the startdepot
			line = fr.readLine();
			list1 = line.split(",");
			int number = Integer.parseInt(list1[1].trim());
			depot.get(0).location = number;
			depot.get(0).getLocation(number);
			
			// Assigning location to the end depot (zero time and distance to every other node)
			line = fr.readLine();
			list1 = line.split(",");
			number = Integer.parseInt(list1[1].trim());
			depot.get(1).location = number;
			depot.get(1).getLocation(number);
			
			// Counting the number of cities
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.numberOfCities = Integer.parseInt(list1[1].trim());
			
			inputdata.times = new float[inputdata.numberOfCities][inputdata.numberOfCities];
			inputdata.distances = new int[inputdata.numberOfCities][inputdata.numberOfCities];
			
			fr.readLine();
			
			// Making the time matrix
			for(int i = 0; i < inputdata.numberOfCities; i++) {
				line = fr.readLine();
				for(int j = 0; j < inputdata.numberOfCities; j++){
				list1 = line.split(",");
				inputdata.times[i][j] = Float.parseFloat(list1[j].trim());
				}
			}
			
			fr.readLine();
			
			//Making the distance matrix
			for(int i = 0; i < inputdata.numberOfCities; i++) {
				line = fr.readLine();
				for(int j = 0; j < inputdata.numberOfCities; j++){
				list1 = line.split(",");
				inputdata.distances[i][j] = Integer.parseInt(list1[j].trim());
				}
			}
			//System.out.println(inputdata.distances[0][1]);
			
			
			// Assigning industry specific variables
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.fuelPrice = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.fuelConsumptionEmptyTruckPerKm = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.fuelConsumptionPerTonKm = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.laborCostperHour = Integer.parseInt(list1[1].trim());
			
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.otherDistanceDependentCostsPerKm = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.otherTimeDependentCostsPerKm = Integer.parseInt(list1[1].trim());
			
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.timeTonService = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			//System.out.println(line);
			list1 = line.split(",");
			inputdata.revenue = Integer.parseInt(list1[1].trim());
			
			fr.close();
			
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}
	
		


}
