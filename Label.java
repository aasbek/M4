
import java.util.ArrayList;
import java.util.Vector;


	public class Label {
		public int labelNumber;
		public float time; 
		public float profit;
		public float weightCapacityUsed;
		public float volumeCapacityUsed;
		public Label predesessor;
		public Node node;
//		public Vector<Integer> path;
		public Vector<Integer> unreachablePickupNodes;
		public float dailyDrivingTime;
		public float consecutiveDrivingTime;
		public float startTimeDailyRest;
		public float startTimeIntermediateBreak;
		public Vector<Integer> openNodes; //pickupnodes
		public int numberDailyRests;
		public float workingTime;
	//	public float waitingTime;
		public int totalDistance;
		

	public String toString() {
		String string = "Node: " + node.number+ ", Time: " + time+", Profit: "+ profit +
				", unreachablePickupNodes: " + unreachablePickupNodes + ", openNodes: " + openNodes + ", dailyDrivingTime: " + dailyDrivingTime + ", startTimeDailyRest: " + startTimeDailyRest + ", numberDailyRests: " + numberDailyRests + ", StartTimeIntermediateBreak: "+ startTimeIntermediateBreak + ", totalDistance: " + totalDistance;
		
		// ", WeightCapacityUsed: " + weightCapacityUsed + ", VolumeCapacityUsed: " + volumeCapacityUsed+ 
		
		//for (int  i : path) { 
			//string += i;
		//}
//		Label temp = predesessor;
//		
//		while(temp!=null) {
//			string+=", Predessesor: "+temp.node.number;
//			temp=temp.predesessor;
//		}
//		
//		System.out.println(string);
//		System.out.println("");
//		System.out.println("");
		return string;
	}
}
