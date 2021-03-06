import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
// import java.util.ArrayList;
// import java.util.Arrays;
import java.util.Vector;


public class InputReader {
	public static void inputReader(String datafile,  Vector<Node> nodes, InstanceData inputdata, Vector<Node> depot, Vector<Node> pickupNodes, Vector<Node> deliveryNodes, Vector<Node>startDepots, Vector<Vehicle>vehicles) {
		try {
			File file = new File(datafile);
			FileReader reader = new FileReader(file);
			BufferedReader fr = new BufferedReader(reader);
			
			//Giving each node a number, corresponding to the location in the vector in the data file
			String line = fr.readLine();
			String[] list1 =line.split(",");
			for (int i = 1; i < list1.length; i++) {
				int number = Integer.parseInt(list1[i].trim());
				Node hello = new Node(number);
				nodes.add(hello);
				//if(number == 1) {
				//	hello.type = "Depot";
				//	depot.add(hello);
				//}
				if((number%2)==0 && number != 0) {
					hello.type = "PickupNode";
					pickupNodes.add(hello);
				}
				else if (number > 2  ) {
					hello.type = "DeliveryNode";
					deliveryNodes.add(hello);
				}
			}
			
			// Volume capacity
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.volumeCap = Integer.parseInt(list1[1].trim());
			
			
			//Weight capacity
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.weightCap = Integer.parseInt(list1[1].trim());	
			
			//Early time window
			line = fr.readLine();
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				float number = Float.parseFloat(list1[i].trim());
				nodes.get(i-1).earlyTimeWindow = number;
			}
			
			//Late time window
			line = fr.readLine();
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				float number = Float.parseFloat(list1[i].trim());
				nodes.get(i-1).lateTimeWindow = number;
			}
			
			//Assigning a weight to each node
			line = fr.readLine();
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				int number = Integer.parseInt(list1[i].trim());
				pickupNodes.get(i-1).weight = number;
				deliveryNodes.get(i-1).weight = number;
			}
			
			//Assigning a volume to each node
			line = fr.readLine();
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				int number = Integer.parseInt(list1[i].trim());
				pickupNodes.get(i-1).volume = number;
				deliveryNodes.get(i-1).volume = number;
			}
		
			//Assigning locations to each pickup node
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
			
			// Assigning location to the start depot of each vehicle
			line = fr.readLine();
			list1 = line.split(",");
			for(int i = 1; i < list1.length; i++){
				int number2 = Integer.parseInt(list1[i].trim());
				Vehicle v = new Vehicle ();
				v.number = i-1;
				Node startDepot = new Node (0);
				startDepot.location = number2;
				startDepot.type = "Depot";
				startDepot.getLocation(number2);
				vehicles.add(v);
				v.startDepot = startDepot;
				startDepot.number = 0;
				depot.add(startDepot);
				
			//	depot.get(i).location = number2;
			//	depot.get(i).getLocation(number2);
			}
			
			// Finding the number of vehicles in the data file
			inputdata.numberOfVehicles = vehicles.size();
			
			// Assigning location to the end depot (zero time and distance to every other node)
			line = fr.readLine();
			list1 = line.split(",");
			int number = Integer.parseInt(list1[1].trim());
			Node endDepot = new Node(1);
			endDepot.type = "Depot";
			endDepot.location = number;
			endDepot.getLocation(number);
			depot.add(endDepot);
			nodes.set(1, endDepot);
		//	depot.get(0).location = number;
		//	depot.get(0).getLocation(number);
			//System.out.println(depot.get(0).location);
			
			for (int i = 0; i < depot.size(); i++) {
				System.out.println(depot.get(i).location);
			}
			
			// Counting the number of cities
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.numberOfCities = Integer.parseInt(list1[1].trim());
			
			// Creating empty time and distance matrices
			inputdata.times = new float[inputdata.numberOfCities][inputdata.numberOfCities];
			inputdata.distances = new float[inputdata.numberOfCities][inputdata.numberOfCities];
			
			fr.readLine();
			
			// Filling the time matrix
			for(int i = 0; i < inputdata.numberOfCities; i++) {
				line = fr.readLine();
				for(int j = 0; j < inputdata.numberOfCities; j++){
				list1 = line.split(",");
				inputdata.times[i][j] = Float.parseFloat(list1[j].trim());
				}
			}
			
			fr.readLine();
			
			// Filling the distance matrix
			for(int i = 0; i < inputdata.numberOfCities; i++) {
				line = fr.readLine();
				for(int j = 0; j < inputdata.numberOfCities; j++){
				list1 = line.split(",");
				inputdata.distances[i][j] = Float.parseFloat(list1[j].trim());
				}
			}
			
			// Assigning industry specific parameters
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.fuelPrice = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.fuelConsumptionEmptyTruckPerKm = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.fuelConsumptionPerTonKm = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.laborCostperHour = Integer.parseInt(list1[1].trim());
			
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.otherDistanceDependentCostsPerKm = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.otherTimeDependentCostsPerKm = Integer.parseInt(list1[1].trim());
			
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.timeTonService = Float.parseFloat(list1[1].trim());
			
			line = fr.readLine();
			list1 = line.split(",");
			inputdata.revenue = Integer.parseInt(list1[1].trim());
			
			fr.close();
		}
		catch(Exception e) {
			e.printStackTrace();
		}
	}
}
