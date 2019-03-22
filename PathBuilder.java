import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Vector;

public class PathBuilder {
	public Vector<Node> nodes;
	public Vector<Node> pickupNodes;
	public Vector<Node> deliveryNodes;
	public Vector<Node> depot;
	public InstanceData inputdata;
	public PrintWriter pw;
	private double zeroTol = 0.001;
	private int numberOfDominatedLabels;
	public Preprocessing preprocess;
	
	
	public PathBuilder(Vector<Node> pickupNodes, Vector<Node> deliveryNodes, Vector<Node> nodes, Vector<Node> depot, InstanceData inputdata, PrintWriter pw) {
		this.pickupNodes = pickupNodes;
		this.nodes = nodes;
		this.deliveryNodes = deliveryNodes;
		this.depot = depot;
		this.inputdata = inputdata;
		this.pw = pw;
		numberOfDominatedLabels = 0;
		
		preprocess = new Preprocessing(pickupNodes, deliveryNodes, nodes, depot, inputdata);
		preprocess.unreachableNodeCombination();
		preprocess.unreachableDeliveryNode();
		preprocess.unreachableDeliveryPairs();
	}
	
	
	
	
	public Label LabelExtension(Node node, Label L) {
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Computing total daily driving time
		float dailyDrivingTime = L.dailyDrivingTime + inputdata.getTime(L.node, node);
		float startTimeDailyRest = L.startTimeDailyRest;
		int numberDailyRests = L.numberDailyRests;
		float consecutiveDrivingTime = L.consecutiveDrivingTime + inputdata.getTime(L.node, node);
		int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		float startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		float consecutiveWorkingTime = L.consecutiveWorkingTime + inputdata.getTime(L.node, node) + L.node.weight*inputdata.timeTonService;
		
	
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
	
		
		//4,5 hour driving rule
		if (consecutiveDrivingTime > 4.5 || consecutiveWorkingTime > 6) { // 
			return null;
		}
		
		
		// If the restrictions on daily driving time (9 hours) or the limit of 24 hours without a daily rest are not met, do not extend the label
		if(dailyDrivingTime > 9 || arrivalTime >13 + 24*(numberDailyRests-1) ) {  //arrivalTime - 11 - startTimeDailyRest > 24
			return null;
		}
		
		if(numberDailyRests == 1 && arrivalTime > 13) {
			return null;
		}
		
		//if(arrivalTime - startTimeDailyRest > 24) {
		//	return null;
		//}	
		
		// If the time is greater than the late time window of a node, return null
		if(arrivalTime> node.lateTimeWindow){
			return null;
		}
	
		// Run preprocessing on the nodes in the open nodes set
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//				System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//				System.out.println(node.number+" "+(i+1));
//				System.exit(0);
				return null;
			}
		}
		
		
		
		if(node.type == "Depot") {
			// Cannot arrive at end depot without delivering every pickup that is picked up
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
	
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Calculating profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			return L2;
		}
		
	
		
		
		else if(node.type == "PickupNode"){
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.time =arrivalTime;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// // Adding the volume corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachable nodes to this label's unreachable nodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the set of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);

			// Running preprocessing on the label and checking whether the node in unreachable due to time windows
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			return L2;
		}
	
		
		
		else if(node.type == "DeliveryNode") {
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
			}
			
			// Removing the weight corresponding to a delivery node when the delivery node is visited
			L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
			
			// Removing the volume corresponding to a delivery node when the delivery node is visited
			L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
			
			
			// Calculating the profit when a pickup node is visited (visiting a delivery node only creates costs)
			L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
			// Running preprocessing on the label and checking whether the node in unreachable due to time windows
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			//System.out.println(L2.toString());
			return L2;
		}
				
	
		
//		L2.path.add(node.number);
		return null;
	}
	
	
	
public Label LabelExtensionWithDailyRest(Node node, Label L) {
		
		// Returns null if the node is already visited
//		if(L.path.contains(node.number)) {
//			return null;
//		}		
		
		
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
	
		//float dailyDrivingTime = L.drivingTime + inputdata.getTime(L.node, node);
		int dailyRestTime = 11; 
		float startTimeDailyRest = L.startTimeDailyRest;
		//float startTimeDailyRestDay = 0;
		//float dailyDrivingTimeDay = 0;
		//float consecutiveDrivingTime = 0;
		//float consecutiveWorkingTime = 0;
		//float dailyDrivingTimeDriving = 0;
		//float timeLeftDailyRest = 0;
		float maxDrivingTime = Float.parseFloat("4.5");
		float consecutiveDrivingTime = L.consecutiveDrivingTime;
		float startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		float consecutiveWorkingTime = L.consecutiveWorkingTime;
		float dailyDrivingTime = L.dailyDrivingTime ;
		
		int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		

		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		//float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		
	
		float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime, node.earlyTimeWindow); 
		float arcDrivingTime = inputdata.getTime(L.node, node);
		float arrivalTimeNoWait = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime; // arrivalTime without the max check (without earlyTimeWindow)
		float waitingTime = node.earlyTimeWindow - (L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime); //computing total waitingTime
		float timeLeftDailyDriving = 9 - L.dailyDrivingTime;
		float timeLeftDriving = maxDrivingTime - L.consecutiveDrivingTime;
		float timeLeftWorking = 6 - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
		float timeToBreak = Math.min(timeLeftDailyDriving, timeLeftDriving);
		timeToBreak = Math.min(timeToBreak, timeLeftWorking);
		startTimeDailyRest = L.time + L.node.weight*inputdata.timeTonService + timeToBreak;
		consecutiveDrivingTime = arcDrivingTime - timeToBreak;
		consecutiveWorkingTime = arcDrivingTime - timeToBreak ;
		dailyDrivingTime = arcDrivingTime - timeToBreak;   
		if (timeLeftWorking <= 0) {  // if daily rest must be started inside the loading time
			startTimeDailyRest = L.time + (6 - L.consecutiveWorkingTime);  // the time inside the loading time where the daily rest is started 
			consecutiveWorkingTime = arcDrivingTime + L.node.weight*inputdata.timeTonService - (6 - L.consecutiveWorkingTime); // the remaining loading time on the node plus the entire driving time on the arc
			consecutiveDrivingTime = arcDrivingTime;
			dailyDrivingTime = arcDrivingTime; 
		}
		
		if (startTimeDailyRest > L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { // if no daily rest is needed, place the daily rest at the end of the arc
			startTimeDailyRest = arrivalTime - dailyRestTime;
			consecutiveWorkingTime = 0;
			consecutiveDrivingTime = 0;
			dailyDrivingTime = 0;
		}
		

	
		int numberDailyRests = L.numberDailyRests;
		
		if(numberDailyRests == 1 && startTimeDailyRest > 13 ) { // if startTimeDailyRest is set to more than 13 and its the first daily rest
			startTimeDailyRest = 13; 
			float timeDriven = startTimeDailyRest - L.time - L.node.weight*inputdata.timeTonService;
			dailyDrivingTime = arcDrivingTime - timeDriven; //how long driven since last break
			consecutiveDrivingTime = arcDrivingTime - timeDriven; //how long driven since last break
			consecutiveWorkingTime = arcDrivingTime - timeDriven;// - L.node.weight*inputdata.timeTonService;
			if(timeDriven < 0) { // if daily rest must be taken inside the working time 
				dailyDrivingTime = arcDrivingTime;
				consecutiveDrivingTime = arcDrivingTime;
				float remainingWorkingTime = L.time + L.node.weight*inputdata.timeTonService - startTimeDailyRest; //the time left of the workingTime after 13 hour rule on daily rest is reached
				consecutiveWorkingTime = arcDrivingTime + remainingWorkingTime; // workingTime after daily rest  
			}
			if (arrivalTimeNoWait < node.earlyTimeWindow && timeDriven > arcDrivingTime) { // secures that the daily rest is always taken on the current arc
				dailyDrivingTime = 0;
				consecutiveWorkingTime = 0;
				consecutiveDrivingTime = 0;	
				startTimeDailyRest = arrivalTime - dailyRestTime;
			} 
		}
		
		else if(numberDailyRests > 1 && startTimeDailyRest  > 24 * (numberDailyRests -1) + 13) { // if startTimeDailyRest is too high according to the 24*numberDailyRests + 13 rule 
			startTimeDailyRest = 13 + 24* (numberDailyRests -1);
			float timeDriven = startTimeDailyRest - L.time - L.node.weight*inputdata.timeTonService;
			dailyDrivingTime = arcDrivingTime - timeDriven; //how long driven since last break
			consecutiveDrivingTime = arcDrivingTime - timeDriven; //how long driven since last break
			consecutiveWorkingTime =  arcDrivingTime - timeDriven;// - L.node.weight*inputdata.timeTonService;
			if(timeDriven < 0) { // if daily rest must be taken inside the working time 
				dailyDrivingTime = arcDrivingTime;
				consecutiveDrivingTime = arcDrivingTime;
				float remainingWorkingTime = L.time + L.node.weight*inputdata.timeTonService - startTimeDailyRest; //the time left of the workingTime after 24*numberDailyRests + 13 rule on daily rest is reached
				consecutiveWorkingTime = arcDrivingTime + remainingWorkingTime; // workingTime after daily rest  
			}
			if (arrivalTimeNoWait < node.earlyTimeWindow && timeDriven > arcDrivingTime) { // secures that the daily rest is always taken on the current arc 
				dailyDrivingTime = 0;
				consecutiveWorkingTime = 0;
				consecutiveDrivingTime = 0;	
				startTimeDailyRest = arrivalTime - dailyRestTime;
			} 	
		}
		
		// If the time is greater than the late time window of a node, return null
		if(arrivalTime> node.lateTimeWindow){
			return null;
		}
			
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//				System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//				System.out.println(node.number+" "+(i+1));
//				System.exit(0);
				return null;
			}
		}
		
		if (consecutiveDrivingTime > maxDrivingTime || consecutiveWorkingTime > 6) {
			return null;
		}
	
		if(node.type == "Depot") {
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;	
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			
			
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			return L2;
		}
		
		if(node.type == "PickupNode"){
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			L2.unreachablePickupNodes = new Vector<Integer>();
			L2.time =arrivalTime;
			
			// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);

			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
//					System.out.println("addind unreach node");
				}
			}
			// Calculating the profit (revenue - costs) when a pickup node is visited 
			
				L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
				return L2;
		}
	
		
		else if(node.type == "DeliveryNode") {
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
	//	
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
			L2.time = arrivalTime; 	
			
			// If the time is greater than the late time window of a node, return null
			
			if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
			}
			// Removing the weight corresponding to a delivery node when the delivery node is visited
			L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
			// Removing the volume corresponding to a delivery node when the delivery node is visited
			L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
			
			L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			

			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
//					System.out.println("addind unreach node");
				}
			}
			return L2;
		}
	
		return null;
	}
	

public Label LabelExtensionWithIntermediateBreak(Node node, Label L) {
		
	// Cannot return to start depot
	if(node.number == 0){
		return null;
	}
	
	// Cannot leave end depot
	if (L.node.number == 1){
		return null;
	}
	
	float dailyDrivingTime = L.dailyDrivingTime + inputdata.getTime(L.node, node); 
	float startTimeDailyRest = L.startTimeDailyRest;
	float startTimeIntermediateBreak = L.startTimeIntermediateBreak;
	float intermediateBreakTime = Float.parseFloat("0.75");
	float maxDrivingTime = Float.parseFloat("4.5");
	float maxWorkingTime = Float.parseFloat("6");
	int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
	

	float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime, node.earlyTimeWindow); 
	float arrivalTimeNoWait = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime;
	float waitingTime = node.earlyTimeWindow -arrivalTimeNoWait;
	float arcDrivingTime = inputdata.getTime(L.node, node);
	float timeLeftDriving = maxDrivingTime - L.consecutiveDrivingTime;
	float timeLeftWorking = maxWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
	float timeLeftDailyDriving = 9 - L.dailyDrivingTime;
	if(timeLeftDailyDriving < arcDrivingTime) {
		return null; // cannot extend with intermediate break if a daily rest is necessary 
	}
	float timeToBreak = Math.min(timeLeftDriving, timeLeftWorking); 
	float timeDrivenBeforeFirstBreak = timeToBreak;
	startTimeIntermediateBreak = L.time  + L.node.weight*inputdata.timeTonService + timeToBreak;
	float consecutiveDrivingTime = arcDrivingTime - timeToBreak;
	float consecutiveWorkingTime = arcDrivingTime - timeToBreak; 
	if (timeLeftWorking <= 0) { // if startTimeIntermediate break is inside the working time
		startTimeIntermediateBreak = L.time + (6 - L.consecutiveWorkingTime); 
		consecutiveWorkingTime = arcDrivingTime + L.node.weight*inputdata.timeTonService - (6 - L.consecutiveWorkingTime);
		consecutiveDrivingTime = arcDrivingTime;
		dailyDrivingTime = arcDrivingTime + L.dailyDrivingTime; 
		timeDrivenBeforeFirstBreak = 0; // if break is taken inside working time, no driving has been done
	}
	if (startTimeIntermediateBreak > L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { // if no intermediate break is necessary, put the break at the end of the arc
		startTimeIntermediateBreak = arrivalTime - intermediateBreakTime;
		consecutiveWorkingTime = 0;
		consecutiveDrivingTime = 0;
	}
	
	int numberDailyRests = L.numberDailyRests;
	
	
	// If the limit of 24 hours*numberofBreaks+13 without a daily rest are not met, do not extend the label
	if(arrivalTime > 13 + 24*(numberDailyRests-1)) {  
		return null;
	}
	
	// If the limit of 13 hours driving time without a daily rest is met, do not extend the label
	if(numberDailyRests == 1 && arrivalTime > 13) {
		return null;
	}
	
	if (waitingTime > 1.5) { // adds a second intermediate break at the end of the arc if waiting time is larger than 90 minutes
		startTimeIntermediateBreak = Math.min(arrivalTime - intermediateBreakTime, startTimeIntermediateBreak + intermediateBreakTime + maxDrivingTime);
		consecutiveWorkingTime = 0;
		consecutiveDrivingTime = 0;
		if(startTimeIntermediateBreak < arrivalTime - intermediateBreakTime) { // if the second intermediate break must be taken before the end of the arc 
			consecutiveWorkingTime = arcDrivingTime - maxDrivingTime - timeDrivenBeforeFirstBreak; // working time (only driving) left after both breaks
			consecutiveDrivingTime = arcDrivingTime - maxDrivingTime - timeDrivenBeforeFirstBreak; // driving time left after both breaks
		}
		arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + 2*intermediateBreakTime, node.earlyTimeWindow); 
	}
	
	
	if (consecutiveDrivingTime > maxDrivingTime) {
		return null;
	}
	
	if (consecutiveWorkingTime > maxWorkingTime) {
		return null;
	}

	// If the time is greater than the late time window of a node, return null
	if(arrivalTime > node.lateTimeWindow){
		return null;
	}

	
	for(int i : L.openNodes) {
		if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//			System.out.println(node.number+" "+(i+1));
//			System.exit(0);
			return null;
		}
	}	

	if(node.type == "Depot") {
		if(!L.openNodes.isEmpty()){
			return null;	
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
		L2.unreachablePickupNodes = new Vector<Integer>();
		
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
		//System.out.println (L2.toString());  
		return L2;
	}
	

	// Deciding whether a pickup node is unreachable: if it is visited or if it is unreachable due to its time windows 	
	 if(node.type == "PickupNode"){
		// Returns null if the node is unreachable 
		if(L.unreachablePickupNodes.contains(node.number)) {
			return null;
		}
		
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
		L2.unreachablePickupNodes = new Vector<Integer>();
		L2.time =arrivalTime;
		
		// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
		if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
			L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
		}
		else{
			return null;
		}
		if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
			L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
		}
		else{
			return null;
		}
		
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		L2.unreachablePickupNodes.add(node.number); 
		L2.openNodes.add(node.number);

		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}
		
		// Calculating the profit (revenue - costs) when a pickup node is visited 
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			return L2;
	}

	
	else if(node.type == "DeliveryNode") {
		// Cannot visit a delivery node whose pickup node has not been visited 
		if (!L.openNodes.contains((node.number-1))){	
			return null;
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
//	
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		L2.time = arrivalTime; 	
		
		// If the time is greater than the late time window of a node, return null
		
		if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
			L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
		}
		// Removing the weight corresponding to a delivery node when the delivery node is visited
		L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
		// Removing the volume corresponding to a delivery node when the delivery node is visited
		L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
		
		L2.profit = L.profit 
				- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
				- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
				- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
				- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
		

		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}

		
		return L2;
		
		
	}

	return null;
}




public Label LabelExtensionWithTwoBreaks(Node node, Label L) { //intermediate break before daily rest
	
	
	// Cannot return to start depot
	if(node.number == 0){
		return null;
	}
	
	// Cannot leave end depot
	if (L.node.number == 1){
		return null;
	}
	
	float dailyDrivingTime = L.dailyDrivingTime; 
	float startTimeDailyRest = L.startTimeDailyRest;
	float startTimeIntermediateBreak = L.startTimeIntermediateBreak;
	float intermediateBreakTime = Float.parseFloat("0.75");
	float maxDrivingTime = Float.parseFloat("4.5");
	float maxWorkingTime = Float.parseFloat("6");
	int dailyRestTime = 11;
	float consecutiveWorkingTime = L.consecutiveWorkingTime;
	float consecutiveDrivingTime = L.consecutiveDrivingTime;	
	int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
	int numberDailyRests = L.numberDailyRests;
	float remainingLoadingTime = 0;
	int extraBreak = 0;
	
	
	float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow); 
	float waitingTime = node.earlyTimeWindow -  L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService  ;
	//float arrivalTimeNoWait = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreak;
	float arcDrivingTime = inputdata.getTime(L.node, node);
	float timeLeftDriving = maxDrivingTime - L.consecutiveDrivingTime;
	float timeLeftWorking = maxWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
	float timeTo24HourRule = (13 + 24* (numberDailyRests -1) - L.time - L.node.weight*inputdata.timeTonService );
	float timeLeftDailyDriving = (9 - L.dailyDrivingTime);
	float timeToDailyRest = Math.min(timeTo24HourRule, timeLeftDailyDriving);
	float timeToBreak = Math.min(timeLeftDriving, timeLeftWorking);
	float drivingTimeBeforeFirstBreak = timeToBreak;
	float workingTimeAfterIntermediateBreak = 0;
	if (timeToBreak < timeToDailyRest - intermediateBreakTime) {   //intermediate break before daily rest
		startTimeIntermediateBreak = L.time  + L.node.weight*inputdata.timeTonService + timeToBreak;	
		if (timeLeftWorking < 0) {  //Break needed in the middle of loading time
			startTimeIntermediateBreak = L.time + (6 - L.consecutiveWorkingTime) ;//timeLeftWorking;
			workingTimeAfterIntermediateBreak =  L.node.weight*inputdata.timeTonService - (6 - L.consecutiveWorkingTime);
			//consecutiveWorkingTime = arcDrivingTime + workingTimeAfterIntermediateBreak;
			//consecutiveDrivingTime = arcDrivingTime;
			//dailyDrivingTime = arcDrivingTime + L.dailyDrivingTime; 
			drivingTimeBeforeFirstBreak = 0;
		}
		timeLeftDailyDriving = timeLeftDailyDriving - drivingTimeBeforeFirstBreak;
		if (startTimeIntermediateBreak >= L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { //first break taken on the end of the arc, no need for another break
			return null;
		}
		if (waitingTime  <= 0 && arcDrivingTime - drivingTimeBeforeFirstBreak < maxDrivingTime) { //no waiting time and less than 4.5 hours to drive after break, no need for another break
			return null;
		}
		startTimeDailyRest = Math.min(startTimeIntermediateBreak + intermediateBreakTime + workingTimeAfterIntermediateBreak + timeLeftDailyDriving - drivingTimeBeforeFirstBreak, startTimeIntermediateBreak + intermediateBreakTime + workingTimeAfterIntermediateBreak + maxDrivingTime);		
		startTimeDailyRest = Math.min(13 + 24 * (numberDailyRests - 1), startTimeDailyRest); 
		startTimeDailyRest = Math.min(startTimeDailyRest, arrivalTime - dailyRestTime);
		startTimeDailyRest = Math.min(startTimeDailyRest, startTimeIntermediateBreak + intermediateBreakTime + maxWorkingTime);
		arrivalTime = Math.max(L.time + arcDrivingTime + L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
		float drivingTimeBetweenBreaks = startTimeDailyRest - startTimeIntermediateBreak - intermediateBreakTime;
		consecutiveDrivingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
		consecutiveWorkingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
		dailyDrivingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
//		if (dailyDrivingTime < 0) {  //in the case where there is waiting time but less than 4,5 hours left to drive, the daily rest is placed at the end
//			startTimeDailyRest = arrivalTime - dailyRestTime;
//			consecutiveDrivingTime = 0;
//			consecutiveWorkingTime = 0;
//			dailyDrivingTime = 0;
//		}
		if (consecutiveDrivingTime > maxDrivingTime) { //in the case where three breaks are needed, 2 intermediate breaks and 1 daily rest (intermediate break - daily rest - intermediate break)
			arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + 2 * intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
			startTimeIntermediateBreak = startTimeDailyRest + maxDrivingTime;
			consecutiveDrivingTime = arcDrivingTime - drivingTimeBetweenBreaks - maxDrivingTime - drivingTimeBeforeFirstBreak;
			consecutiveWorkingTime = arcDrivingTime - drivingTimeBetweenBreaks -  maxDrivingTime - drivingTimeBeforeFirstBreak;
			dailyDrivingTime =  arcDrivingTime -  drivingTimeBetweenBreaks - drivingTimeBeforeFirstBreak;
		}
	}	
	else {
		return null;
	}
		
	/* else { // takes daily rest then intermediate break
		arrivalTime = Math.max(L.time + arcDrivingTime + L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
		startTimeDailyRest = Math.min(L.time + L.node.weight*inputdata.timeTonService + timeLeftDailyDriving, 13 + 24 * (numberDailyRests -1));
		float remainingDrivingTime = arcDrivingTime - timeToDailyRest;
		dailyDrivingTime = arcDrivingTime - timeToDailyRest;
		float drivingTimeBeforeDailyRest = timeToDailyRest;
		if (timeTo24HourRule < 0 ) { //if the daily rest must be taken during loading
			timeTo24HourRule = 13 + 24* (numberDailyRests -1) - L.time;
			remainingLoadingTime = L.node.weight*inputdata.timeTonService - timeTo24HourRule;
			remainingDrivingTime = arcDrivingTime;
			dailyDrivingTime = arcDrivingTime; 
			drivingTimeBeforeDailyRest = 0;
		} 
		startTimeIntermediateBreak = Math.min(startTimeDailyRest + dailyRestTime + remainingDrivingTime + remainingLoadingTime, startTimeDailyRest + dailyRestTime + maxDrivingTime + remainingLoadingTime);
		startTimeIntermediateBreak = Math.min(startTimeIntermediateBreak, arrivalTime - intermediateBreakTime);
		if (startTimeDailyRest >= L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { //break taken on the end of the arc, no need for another break
			return null;
		}
		if (waitingTime <= 0 && arcDrivingTime < timeLeftDailyDriving + maxDrivingTime ) { //no waiting time and less than 4.5 hours to drive after the break, no need for another break
			return null;
		}
		if(remainingDrivingTime < maxDrivingTime ) { 
			consecutiveDrivingTime = 0;
			consecutiveWorkingTime = 0;
			//startTimeIntermediateBreak = arrivalTime - intermediateBreakTime;
		}
		else {
			consecutiveDrivingTime = arcDrivingTime - maxDrivingTime - drivingTimeBeforeDailyRest;
			consecutiveWorkingTime = arcDrivingTime - maxDrivingTime - drivingTimeBeforeDailyRest;
		}
		if(remainingDrivingTime + remainingLoadingTime > maxWorkingTime) {
			startTimeIntermediateBreak = startTimeDailyRest + dailyRestTime + maxWorkingTime;
			consecutiveDrivingTime = maxWorkingTime - remainingLoadingTime - remainingDrivingTime;
			consecutiveWorkingTime = maxWorkingTime - remainingLoadingTime - remainingDrivingTime;
		}

		
		if (dailyDrivingTime > 9) { //one intermediate break and two daily rests
			startTimeDailyRest = startTimeIntermediateBreak + intermediateBreakTime + maxDrivingTime;
			arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService +  intermediateBreakTime + 2 * dailyRestTime, node.earlyTimeWindow);
			consecutiveDrivingTime = arcDrivingTime - 2* maxDrivingTime - drivingTimeBeforeDailyRest;
			consecutiveWorkingTime = arcDrivingTime - 2* maxDrivingTime - drivingTimeBeforeDailyRest;
			dailyDrivingTime =  arcDrivingTime - 2* maxDrivingTime - drivingTimeBeforeDailyRest;
			extraBreak = 1;
		}
	
	} */
	numberDailyRests = L.numberDailyRests + 1 + extraBreak;
	
	//if(arrivalTime - startTimeDailyRest > 24) {
	//	return null;
	//}	
	
	
	// If the restrictions on daily driving time (9 hours) or the limit of 24 hours without a daily rest are not met, do not extend the label
	if(arrivalTime > 13 + 24*(numberDailyRests-1)) {  //arrivalTime - 11 - startTimeDailyRest > 24
		return null;
	}

			 
	if(numberDailyRests == 1 && arrivalTime > 13) {
		return null;
	}

	
	// If the time is greater than the late time window of a node, return null
	if(arrivalTime> node.lateTimeWindow){
		return null;
	}
	
	if (consecutiveDrivingTime > maxDrivingTime || consecutiveWorkingTime > 6) {
		return null;
	}
	
	for(int i : L.openNodes) {
		if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//			System.out.println(node.number+" "+(i+1));
//			System.exit(0);
			return null;
		}
	}
	// Cannot arrive at end depot without delivering every pickup
	
	
		// Removing the pickup node from the open nodes list if its corresponding delivery node is visited
	

	if(node.type == "Depot") {
		if(!L.openNodes.isEmpty()){
			return null;	
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
		
		
//	
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		
		
		L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
		return L2;
	}
	

	// Deciding whether a pickup node is unreachable: if it is visited or if it is unreachable due to its time windows 	
	 if(node.type == "PickupNode"){
		// Returns null if the node is unreachable 
		if(L.unreachablePickupNodes.contains(node.number)) {
			return null;
		}
		
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
		
		

		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
//		L2.time = Math.max(L.time+InstanceData.getTime(L.node, node, inputdata)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		L2.time =arrivalTime;
		// If the time is greater than the late time window of a node, return null
		
		// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
		if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
			L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
		}
		else{
			return null;
		}
		if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
			L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
		}
		else{
			return null;
		}
		
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		L2.unreachablePickupNodes.add(node.number); 
		L2.openNodes.add(node.number);

		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}
		
		// Calculating the profit (revenue - costs) when a pickup node is visited 
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			return L2;
	}

	
	else if(node.type == "DeliveryNode") {
		// Cannot visit a delivery node whose pickup node has not been visited 
		if (!L.openNodes.contains((node.number-1))){	
			return null;
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
//	
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		L2.time = arrivalTime; 	
		
		// If the time is greater than the late time window of a node, return null
		
		if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
			L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
		}
		// Removing the weight corresponding to a delivery node when the delivery node is visited
		L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
		// Removing the volume corresponding to a delivery node when the delivery node is visited
		L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
		
		L2.profit = L.profit 
				- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
				- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
				- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
				- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
		

		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}

	
		return L2;
		
	}

//	L2.path.add(node.number);
	return null;
}




public Label LabelExtensionWithTwoBreaks2(Node node, Label L) { //daily rest before intermediate break

	
	// Cannot return to start depot
	if(node.number == 0){
		return null;
	}
	
	// Cannot leave end depot
	if (L.node.number == 1){
		return null;
	}
	
	float dailyDrivingTime = L.dailyDrivingTime; 
	float startTimeDailyRest = L.startTimeDailyRest;
	float startTimeIntermediateBreak = L.startTimeIntermediateBreak;
	float intermediateBreakTime = Float.parseFloat("0.75");
	float maxDrivingTime = Float.parseFloat("4.5");
	float maxWorkingTime = Float.parseFloat("6");
	int dailyRestTime = 11;
	float consecutiveWorkingTime = L.consecutiveWorkingTime;
	float consecutiveDrivingTime = L.consecutiveDrivingTime;	
	int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
	int numberDailyRests = L.numberDailyRests;
	float remainingLoadingTime = 0;
	int extraBreak = 0;
	
	
	float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow); 
	float waitingTime = node.earlyTimeWindow -  L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService  ;
	//float arrivalTimeNoWait = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreak;
	float arcDrivingTime = inputdata.getTime(L.node, node);
	float timeLeftDriving = maxDrivingTime - L.consecutiveDrivingTime;
	float timeLeftWorking = maxWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
	float timeTo24HourRule = (13 + 24* (numberDailyRests -1) - L.time - L.node.weight*inputdata.timeTonService );
	float timeLeftDailyDriving = (9 - L.dailyDrivingTime);
	float timeToDailyRest = Math.min(timeTo24HourRule, timeLeftDailyDriving);
	float timeToBreak = Math.min(timeLeftDriving, timeLeftWorking);
	float drivingTimeBeforeFirstBreak = timeToBreak;
	float workingTimeAfterIntermediateBreak = 0;
	/*
	if (timeToBreak < timeToDailyRest - intermediateBreakTime) {   //intermediate break before daily rest
		startTimeIntermediateBreak = L.time  + L.node.weight*inputdata.timeTonService + timeToBreak;	
		if (timeLeftWorking < 0) {  //Break needed in the middle of loading time
			startTimeIntermediateBreak = L.time + (6 - L.consecutiveWorkingTime) ;//timeLeftWorking;
			workingTimeAfterIntermediateBreak =  L.node.weight*inputdata.timeTonService - (6 - L.consecutiveWorkingTime);
			//consecutiveWorkingTime = arcDrivingTime + workingTimeAfterIntermediateBreak;
			//consecutiveDrivingTime = arcDrivingTime;
			//dailyDrivingTime = arcDrivingTime + L.dailyDrivingTime; 
			drivingTimeBeforeFirstBreak = 0;
		}
		timeLeftDailyDriving = timeLeftDailyDriving - drivingTimeBeforeFirstBreak;
		if (startTimeIntermediateBreak >= L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { //first break taken on the end of the arc, no need for another break
			return null;
		}
		if (waitingTime  <= 0 && arcDrivingTime - drivingTimeBeforeFirstBreak < maxDrivingTime) { //no waiting time and less than 4.5 hours to drive after break, no need for another break
			return null;
		}
		startTimeDailyRest = Math.min(startTimeIntermediateBreak + intermediateBreakTime + workingTimeAfterIntermediateBreak + timeLeftDailyDriving - drivingTimeBeforeFirstBreak, startTimeIntermediateBreak + intermediateBreakTime + workingTimeAfterIntermediateBreak + maxDrivingTime);		
		startTimeDailyRest = Math.min(13 + 24 * (numberDailyRests - 1), startTimeDailyRest); 
		startTimeDailyRest = Math.min(startTimeDailyRest, arrivalTime - dailyRestTime);
		arrivalTime = Math.max(L.time + arcDrivingTime + L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
		float drivingTimeBetweenBreaks = startTimeDailyRest - startTimeIntermediateBreak - intermediateBreakTime;
		consecutiveDrivingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
		consecutiveWorkingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
		dailyDrivingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
//		if (dailyDrivingTime < 0) {  //in the case where there is waiting time but less than 4,5 hours left to drive, the daily rest is placed at the end
//			startTimeDailyRest = arrivalTime - dailyRestTime;
//			consecutiveDrivingTime = 0;
//			consecutiveWorkingTime = 0;
//			dailyDrivingTime = 0;
//		}
		if (consecutiveDrivingTime > maxDrivingTime) { //in the case where three breaks are needed, 2 intermediate breaks and 1 daily rest (intermediate break - daily rest - intermediate break)
			arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + 2 * intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
			startTimeIntermediateBreak = startTimeDailyRest + maxDrivingTime;
			consecutiveDrivingTime = arcDrivingTime - drivingTimeBetweenBreaks - maxDrivingTime - drivingTimeBeforeFirstBreak;
			consecutiveWorkingTime = arcDrivingTime - drivingTimeBetweenBreaks -  maxDrivingTime - drivingTimeBeforeFirstBreak;
			dailyDrivingTime =  arcDrivingTime -  drivingTimeBetweenBreaks - drivingTimeBeforeFirstBreak;
		}
	}	
	*/
	 // takes daily rest then intermediate break
		arrivalTime = Math.max(L.time + arcDrivingTime + L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
		startTimeDailyRest = Math.min(L.time + L.node.weight*inputdata.timeTonService + timeLeftDailyDriving, 13 + 24 * (numberDailyRests -1));
		startTimeDailyRest = Math.min(startTimeDailyRest, L.time +  L.node.weight*inputdata.timeTonService + timeLeftDriving);
		startTimeDailyRest = Math.min(startTimeDailyRest, L.time +  L.node.weight*inputdata.timeTonService + timeLeftWorking); //startTimeDailyRest is the is started when the first of the four rules are reached.
		timeToDailyRest = startTimeDailyRest - (L.time + L.node.weight*inputdata.timeTonService);
		float remainingDrivingTime = arcDrivingTime - timeToDailyRest;
		dailyDrivingTime = arcDrivingTime - timeToDailyRest;
		float drivingTimeBeforeDailyRest = timeToDailyRest;
		if (timeTo24HourRule < 0 && timeTo24HourRule < timeLeftWorking) { //if the daily rest must be taken during loading
			timeTo24HourRule = 13 + 24* (numberDailyRests -1) - L.time;
			remainingLoadingTime = L.node.weight*inputdata.timeTonService - timeTo24HourRule;
			remainingDrivingTime = arcDrivingTime;
			dailyDrivingTime = arcDrivingTime; 
			drivingTimeBeforeDailyRest = 0;
		} 
		if (timeLeftWorking < 0 && timeLeftWorking < timeTo24HourRule) {  // if daily rest must be started inside the loading time
			startTimeDailyRest = L.time + (6 - L.consecutiveWorkingTime);  // the time inside the loading time where the daily rest is started 
			remainingLoadingTime = L.node.weight*inputdata.timeTonService - (6 - L.consecutiveWorkingTime);
			remainingDrivingTime = arcDrivingTime;
			dailyDrivingTime = arcDrivingTime; 
			drivingTimeBeforeDailyRest = 0;
		}
		startTimeIntermediateBreak = Math.min(startTimeDailyRest + dailyRestTime + remainingDrivingTime + remainingLoadingTime, startTimeDailyRest + dailyRestTime + maxDrivingTime + remainingLoadingTime);
		startTimeIntermediateBreak = Math.min(startTimeIntermediateBreak, arrivalTime - intermediateBreakTime);
		if (startTimeDailyRest >= L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { //break taken on the end of the arc, no need for another break
			return null;
		}
		if (waitingTime <= 0 && arcDrivingTime < timeLeftDailyDriving + maxDrivingTime ) { //no waiting time and less than 4.5 hours to drive after the break, no need for another break
			return null;
		}
		if(remainingDrivingTime < maxDrivingTime ) { 
			consecutiveDrivingTime = 0;
			consecutiveWorkingTime = 0;
			//startTimeIntermediateBreak = arrivalTime - intermediateBreakTime;
		}
		else if(remainingDrivingTime + remainingLoadingTime > maxWorkingTime) {
			startTimeIntermediateBreak = startTimeDailyRest + dailyRestTime + maxWorkingTime;
			float drivingTimeBetweenBreaks = Math.min(maxWorkingTime - remainingLoadingTime, maxDrivingTime);
			consecutiveDrivingTime = Math.max(arcDrivingTime  - drivingTimeBetweenBreaks, 0);
			consecutiveWorkingTime = Math.max(arcDrivingTime  - drivingTimeBetweenBreaks, 0);
		}
		else {
			consecutiveDrivingTime = arcDrivingTime - maxDrivingTime - drivingTimeBeforeDailyRest;
			consecutiveWorkingTime = arcDrivingTime - maxDrivingTime - drivingTimeBeforeDailyRest;
		}
		
		if (dailyDrivingTime > 9 || consecutiveDrivingTime > maxDrivingTime) { //one intermediate break and two daily rests
			startTimeDailyRest = startTimeIntermediateBreak + intermediateBreakTime + maxDrivingTime;
			arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService +  intermediateBreakTime + 2 * dailyRestTime, node.earlyTimeWindow);
			consecutiveDrivingTime = arcDrivingTime -  maxDrivingTime - Math.min(maxDrivingTime, maxWorkingTime - remainingLoadingTime) - drivingTimeBeforeDailyRest;
			consecutiveWorkingTime =  arcDrivingTime -  maxDrivingTime - Math.min(maxDrivingTime, maxWorkingTime - remainingLoadingTime) - drivingTimeBeforeDailyRest;
			dailyDrivingTime = arcDrivingTime -  maxDrivingTime - Math.min(maxDrivingTime, maxWorkingTime - remainingLoadingTime) - drivingTimeBeforeDailyRest;
			extraBreak = 1;
		}
		
	
	numberDailyRests = L.numberDailyRests + 1 + extraBreak;
	
	//if(arrivalTime - startTimeDailyRest > 24) {
	//	return null;
	//}	
	
	
	// If the restrictions on daily driving time (9 hours) or the limit of 24 hours without a daily rest are not met, do not extend the label
	if(arrivalTime > 13 + 24*(numberDailyRests-1)) {  //arrivalTime - 11 - startTimeDailyRest > 24
		return null;
	}

			 
	if(numberDailyRests == 1 && arrivalTime > 13) {
		return null;
	}

	
	// If the time is greater than the late time window of a node, return null
	if(arrivalTime> node.lateTimeWindow){
		return null;
	}
	
	if (consecutiveDrivingTime > maxDrivingTime || consecutiveWorkingTime > 6) {
		return null;
	}
	
	for(int i : L.openNodes) {
		if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//			System.out.println(node.number+" "+(i+1));
//			System.exit(0);
			return null;
		}
	}
	// Cannot arrive at end depot without delivering every pickup
	
	
		// Removing the pickup node from the open nodes list if its corresponding delivery node is visited
	

	if(node.type == "Depot") {
		if(!L.openNodes.isEmpty()){
			return null;	
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
		
		
//	
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		
		
		L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
		return L2;
	}
	

	// Deciding whether a pickup node is unreachable: if it is visited or if it is unreachable due to its time windows 	
	 if(node.type == "PickupNode"){
		// Returns null if the node is unreachable 
		if(L.unreachablePickupNodes.contains(node.number)) {
			return null;
		}
		
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
		
		

		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
//		L2.time = Math.max(L.time+InstanceData.getTime(L.node, node, inputdata)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		L2.time =arrivalTime;
		// If the time is greater than the late time window of a node, return null
		
		// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
		if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
			L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
		}
		else{
			return null;
		}
		if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
			L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
		}
		else{
			return null;
		}
		
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		L2.unreachablePickupNodes.add(node.number); 
		L2.openNodes.add(node.number);

		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}
		
		// Calculating the profit (revenue - costs) when a pickup node is visited 
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			return L2;
	}

	
	else if(node.type == "DeliveryNode") {
		// Cannot visit a delivery node whose pickup node has not been visited 
		if (!L.openNodes.contains((node.number-1))){	
			return null;
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = consecutiveWorkingTime;
//	
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		L2.time = arrivalTime; 	
		
		// If the time is greater than the late time window of a node, return null
		
		if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
			L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
		}
		// Removing the weight corresponding to a delivery node when the delivery node is visited
		L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
		// Removing the volume corresponding to a delivery node when the delivery node is visited
		L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
		
		L2.profit = L.profit 
				- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
				- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
				- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
				- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
		

		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}

	
		return L2;
		
	}

//	L2.path.add(node.number);
	return null;
}




	
	
	public Vector<Label> BuildPaths() {
		Vector<Label> list = new Vector<Label>();  // List of non-dominated labels 
		Label L = new Label();
		// Initializing label
		L.labelNumber = 0;
//		L.path = new Vector<Integer>();
		L.node = nodes.get(0);
		L.time = Float.parseFloat("0");
		L.profit = 0;
		L.weightCapacityUsed = 0;
		L.volumeCapacityUsed = 0;
		L.numberDailyRests = 1;
		L.predesessor = null;
		L.totalDistance = 0;
		L.startTimeDailyRest = 0;
		L.unreachablePickupNodes = new Vector<Integer>();
		L.openNodes = new Vector<Integer>();		
		L.startTimeIntermediateBreak = 0;
		L.consecutiveDrivingTime = 0;
		L.consecutiveWorkingTime = 0;
//		L.path.add(L.node.number);
		ArrayList<Vector<Label>> unprocessedAtNode = new ArrayList<Vector<Label>>();
		ArrayList<Vector<Label>> processedAtNode = new ArrayList<Vector<Label>>();
		for(int i = 0; i < nodes.size(); i++) {
			Vector<Label> processed = new Vector<Label>();
			processedAtNode.add(i, processed);
			Vector<Label> unprocessed = new Vector<Label>();
			unprocessedAtNode.add(i, unprocessed);
		}
		PriorityQueue<Label> unprocessedQueue = new PriorityQueue<Label>(5, new UnprocessedComparator()); 
		unprocessedQueue.add(L);
		
		int counter =0;
		//Going through all unprocessed labels
		while(!unprocessedQueue.isEmpty()) { 
			Label label = unprocessedQueue.remove();
			counter++;
			if(counter%1000 == 0) {
				//System.out.println(counter+" "+label.toString());
				//System.out.println("number of unprocessed labels: "+unprocessedQueue.size());
			}
//			for(int i = 2; i < nodes.size(); i++) { // Going through all nodes except node 0 and node 1 (the depot nodes)
//				Label newLabel = LabelExtension(nodes.get(i), label);
//				if(newLabel!=null) {
//					if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
//						unprocessedQueue.add(newLabel); 
//						unprocessedAtNode.get(newLabel.node.number).add(newLabel);
//					}
//				}
//			}
			for(Node pickup :pickupNodes) { // Going through all nodes except node 0 and node 1 (the depot nodes)
				
				float arcDrivingTime = inputdata.getTime(label.node,  pickup);
				float dailyDrivingTime = label.dailyDrivingTime;
				
				if (arcDrivingTime + dailyDrivingTime <= 9){
				Label newLabel = LabelExtension(pickup, label);
				
				if(newLabel!=null) {
		//			System.out.println(newLabel.toString());
					if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
						unprocessedQueue.add(newLabel); 
						unprocessedAtNode.get(newLabel.node.number).add(newLabel);
					}
				}
				
				Label newLabel3 = LabelExtensionWithIntermediateBreak(pickup, label);
				
				if(newLabel3!=null) {
				//	System.out.println(newLabel3.toString());
					if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number))) {
						unprocessedQueue.add(newLabel3); 
						unprocessedAtNode.get(newLabel3.node.number).add(newLabel3);
					}
				}
				}
				Label newLabel2 = LabelExtensionWithDailyRest(pickup, label);
				
				if(newLabel2!=null) {
			//		System.out.println(newLabel2.toString());
					if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number))) {
						unprocessedQueue.add(newLabel2); 
						unprocessedAtNode.get(newLabel2.node.number).add(newLabel2);
					}
				}
				

				float intermediateBreakTime = Float.parseFloat("0.75");
				float maxDrivingTime = Float.parseFloat("4.5");
				int dailyRestTime = 9;
				float waitingTime = pickup.earlyTimeWindow - (label.time + inputdata.getTime(label.node,  pickup) + label.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime);
				
				
				if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
				Label newLabel4 = LabelExtensionWithTwoBreaks(pickup, label);
				
					if(newLabel4!=null) {
				//		System.out.println(newLabel3.toString());
						if(checkdominance(newLabel4, unprocessedQueue, unprocessedAtNode.get(newLabel4.node.number), processedAtNode.get(newLabel4.node.number))) {
							unprocessedQueue.add(newLabel4); 
							unprocessedAtNode.get(newLabel4.node.number).add(newLabel4);
						}
					}			
				}
				
				if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
				Label newLabel5 = LabelExtensionWithTwoBreaks2(pickup, label);
				
					if(newLabel5!=null) {
				//		System.out.println(newLabel3.toString());
						if(checkdominance(newLabel5, unprocessedQueue, unprocessedAtNode.get(newLabel5.node.number), processedAtNode.get(newLabel5.node.number))) {
							unprocessedQueue.add(newLabel5); 
							unprocessedAtNode.get(newLabel5.node.number).add(newLabel5);
						}
					}			
				}
			}	
				
			for(int i : label.openNodes) { // Going through all nodes except node 0 and node 1 (the depot nodes)

				Node node = nodes.get(i+1); 
				float arcDrivingTime = inputdata.getTime(label.node, node);
				float intermediateBreakTime = Float.parseFloat("0.75");
				float maxDrivingTime = Float.parseFloat("4.5");
				int dailyRestTime = 9;
				float waitingTime = node.earlyTimeWindow - (label.time + inputdata.getTime(label.node,  node) + label.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime);
				float dailyDrivingTime = label.dailyDrivingTime;
				
				if (arcDrivingTime + dailyDrivingTime < 9) {
				
				Label newLabel = LabelExtension(nodes.get(i+1), label);
				
				if(newLabel!=null) {
				//System.out.println(newLabel.toString());
					if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
						unprocessedQueue.add(newLabel); 
						unprocessedAtNode.get(newLabel.node.number).add(newLabel);
					}
				}
				
				Label newLabel3 = LabelExtensionWithIntermediateBreak(nodes.get(i+1), label);
				
				if(newLabel3!=null) {
				//	System.out.println(newLabel3.toString());
					if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number))) {
						unprocessedQueue.add(newLabel3); 
						unprocessedAtNode.get(newLabel3.node.number).add(newLabel3);
					}
				}
				}
				
				Label newLabel2 = LabelExtensionWithDailyRest(nodes.get(i+1), label);
				
				if(newLabel2!=null) {
					//System.out.println(newLabel2.toString());
					if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number))) {
						unprocessedQueue.add(newLabel2); 
						unprocessedAtNode.get(newLabel2.node.number).add(newLabel2);
					}
				}
				

				
				if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
				
				Label newLabel4 = LabelExtensionWithTwoBreaks(nodes.get(i+1), label);
				
				if(newLabel4!=null) {
				//	System.out.println(newLabel3.toString());
					if(checkdominance(newLabel4, unprocessedQueue, unprocessedAtNode.get(newLabel4.node.number), processedAtNode.get(newLabel4.node.number))) {
						unprocessedQueue.add(newLabel4); 
						unprocessedAtNode.get(newLabel4.node.number).add(newLabel4);
					}
				}
				}
				if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
					
				Label newLabel5 = LabelExtensionWithTwoBreaks2(nodes.get(i+1), label);
				
				if(newLabel5!=null) {
				//	System.out.println(newLabel3.toString());
					if(checkdominance(newLabel5, unprocessedQueue, unprocessedAtNode.get(newLabel5.node.number), processedAtNode.get(newLabel5.node.number))) {
						unprocessedQueue.add(newLabel5); 
						unprocessedAtNode.get(newLabel5.node.number).add(newLabel5);
					}
				}
				}
				
			}
			Node node = nodes.get(1);
			float arcDrivingTime = inputdata.getTime(label.node, node);
			float dailyDrivingTime = label.dailyDrivingTime;
			
			if (arcDrivingTime + dailyDrivingTime < 9) {
			
			Label newLabel = LabelExtension(nodes.get(1), label); // Adding node 1 (the end depot node) to the end of the path 
			
			if(newLabel!=null) {
				//System.out.println(newLabel.toString());
				if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
					list.add(newLabel);
				}
			}
			Label newLabel3 = LabelExtensionWithIntermediateBreak(nodes.get(1), label);
			
			if(newLabel3!=null) {
				//System.out.println(newLabel3.toString());
				if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number))) {
					list.add(newLabel3);
				}
			}
			}
			
			
			Label newLabel2 = LabelExtensionWithDailyRest(nodes.get(1), label);
			
			if(newLabel2!=null) {
			//	System.out.println(newLabel2.toString());
				if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number))) {
					list.add(newLabel2);
				}
			}
			

	
			float intermediateBreakTime = Float.parseFloat("0.75");
			float maxDrivingTime = Float.parseFloat("4.5");
			int dailyRestTime = 9;
			float waitingTime = node.earlyTimeWindow - (label.time + inputdata.getTime(label.node,  node) + label.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime);
			
			
			if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
			
			Label newLabel4 = LabelExtensionWithTwoBreaks(nodes.get(1), label);
			
			if(newLabel4!=null) {
				//System.out.println(newLabel3.toString());
				if(checkdominance(newLabel4, unprocessedQueue, unprocessedAtNode.get(newLabel4.node.number), processedAtNode.get(newLabel4.node.number))) {
					list.add(newLabel4);
				}
			}
			}
			
			if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
				
			Label newLabel5 = LabelExtensionWithTwoBreaks2(nodes.get(1), label);
			
			if(newLabel5!=null) {
				//System.out.println(newLabel3.toString());
				if(checkdominance(newLabel5, unprocessedQueue, unprocessedAtNode.get(newLabel5.node.number), processedAtNode.get(newLabel5.node.number))) {
					list.add(newLabel5);
				}
			}
			}
			
			processedAtNode.get(label.node.number).add(label); // The label removed from unprocessed is added to processed
		}
		
//		System.out.println("Number of paths:" + processed.size());
		System.out.println("number of non-dominated paths: "+list.size());
		pw.println("number of non-dominated paths: "+list.size());
		System.out.println("number of dominated labels: "+numberOfDominatedLabels);
		pw.println("number of dominated labels: "+numberOfDominatedLabels);
		System.out.println("The best label is:");
		pw.println ("The best label is: ");
		System.out.println(findBestLabel(list).toString());
		//pw.println(findBestLabel(list).toString());
		//for(Label i : list) {
		//System.out.println(i.toString());
		//}
		//for (Label i : list) {
		//System.out.println (i.toString());
		//}
		
	
		return list;
	}
	
	
	private boolean dominateLabel(Label L1, Label L2) { //Checks if L1 dominates L2
		if (L1.node.number != L2.node.number) {
			return false;
		}
		if (L1.profit+zeroTol<L2.profit) {
			return false;
		}
		if (L1.time-zeroTol>L2.time) {
			return false;
		}

		//if( L1.startTimeDailyRest >= L2.startTimeDailyRest && L1.dailyDrivingTime <= L2.dailyDrivingTime && L1.startTimeIntermediateBreak >= L2.startTimeIntermediateBreak &&  L1.consecutiveDrivingTime <= L2.consecutiveDrivingTime && L1.consecutiveWorkingTime <= L2.consecutiveWorkingTime) { 
		
		if( L1.startTimeDailyRest >= L2.startTimeDailyRest) {
			
			if ( L1.dailyDrivingTime <= L2.dailyDrivingTime ) {
				
				if ( L1.consecutiveDrivingTime <= L2.consecutiveDrivingTime) {
					
					
					if(  L1.startTimeIntermediateBreak >= L2.startTimeIntermediateBreak ) {
				
			
						if (L1.consecutiveWorkingTime <= L2.consecutiveWorkingTime) {
		
		
							for (int i : L1.openNodes ){
								if (!L2.openNodes.contains(i)){
									return false;
								}
							}
							for (int i : L1.unreachablePickupNodes ){
								if (!L2.unreachablePickupNodes.contains(i)){
									return false;
								}	
							}
							return true;	
						}
						else return false;
						
					}		
					else return false; 	
				}
				else return false; 	
			}
			else return false; 	
		}
		else return false; 
	}
	
	
	
	//Updates the processed and unprocessed lists according to the dominated labels.
	private boolean checkdominance(Label newLabel, PriorityQueue<Label> unprocessedQueue, Vector<Label> unprocessed, Vector<Label> processed) {
		Vector<Label> remove = new Vector<Label>();
		
		for(Label oldLabel : unprocessed) {
			if(dominateLabel(oldLabel, newLabel)) {
				numberOfDominatedLabels++;
				unprocessedQueue.removeAll(remove);
				unprocessed.removeAll(remove);
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel)) {
				remove.add(oldLabel);
				numberOfDominatedLabels++;
			}
		}
		unprocessedQueue.removeAll(remove);
		unprocessed.removeAll(remove);
		
		remove = new Vector<Label>();
		for(Label oldLabel : processed) {
			if(dominateLabel(oldLabel, newLabel)) {
				processed.removeAll(remove);
				numberOfDominatedLabels++;
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel)) {
				numberOfDominatedLabels++;
				remove.add(oldLabel);
			}
		}
		processed.removeAll(remove);
		
		return true;
	}
	
	public Label findBestLabel(Vector<Label> list) {
		float currentBestProfit = 0;
		Label bestLabel = null;
		for(Label i : list) {
			if(i.profit > currentBestProfit) {
				currentBestProfit = i.profit;
				bestLabel = i;
			}
		}
		
		Label temp = bestLabel.predesessor;
		while(temp!=null) {
			System.out.println(temp.toString());
			pw.println(temp.toString());
		temp=temp.predesessor;
		} 
		pw.println(bestLabel.toString());
		return bestLabel;
		
	}
	


}
