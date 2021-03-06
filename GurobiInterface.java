import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Vector;
import gurobi.GRB.DoubleAttr;
import gurobi.GRB.IntAttr;
import gurobi.*;

	public class GurobiInterface {
		
		// Creating Gurobi environment
	    GRBEnv    env   = new GRBEnv("mip1.log");
	    GRBModel  model = new GRBModel(env);
	    
	    // Creating Gurobi variables
	    private Vector<GRBVar> variables;
	    private GRBVar[][] lambdaVars;
		
	    // Creating Gurobi constraints
		public GRBConstr[] visitedPickupsCon;
		public GRBConstr[] oneVisitCon; 
		
		// Creating Gurobi objective function
		public GRBLinExpr objective;
		
		// Lists of dual variables
		public Vector<Float> dualVisitedPickupsCon;  
		public Vector<Float> dualOneVisitCon;
		
	    public InstanceData inputdata;
		public float profit = 0;
		public Vector<Node> pickupNodes;
		public Vector<Node> deliveryNodes;
		public Vector<Node> nodes;
		public Vector<Node> depot;
		public PrintWriter pw;
		public Vector<Node> path;
		//public Vector<Route> routes;
		public Vector<Vehicle> vehicles;
		public PathBuilder builder;
		//public Route route;
		public int numberOfRoutes = 0;
		//public Vector<Vector<Vector<Integer>>> visitedPickupsByVehicleOnRoute;
		int[][][] visitedPickupsByVehicleOnRoute;
	
		 
		//private Hashtable<Integer, Route> pathList;
		
		public GRBLinExpr visitedPickupsLeftHand[];
		public GRBLinExpr oneVisitLeftHand[];
		
		public GRBColumn col;
		public GRBColumn col2;
		
		public GurobiInterface(InstanceData inputdata, Vector<Node> nodes, Vector<Node> depot, Vector<Node> deliveryNodes, Vector<Node> pickupNodes, Vector<Vehicle> vehicles, Vector<Float> dualVisitedPickupsCon, Vector<Float> dualOneVisitCon, PrintWriter pw) throws Exception {
			this.vehicles = vehicles; 
			//this.builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot, inputdata, pw, vehicles);
			//this.path = route.path;
			//this.profit = label.profit;
			this.inputdata = inputdata;
			this.pickupNodes = pickupNodes;
			this.deliveryNodes = deliveryNodes;
			this.depot = depot;
			this.nodes = nodes;
			this.pw = pw;
			this.vehicles = vehicles;
			this.dualVisitedPickupsCon = dualVisitedPickupsCon;
			this.dualOneVisitCon = dualOneVisitCon;
			//solveProblem();
		}
		
		public void buildProblem() throws Exception {
			this.objective = new GRBLinExpr();
			
			this.variables = new Vector<GRBVar>();
			this.lambdaVars = new GRBVar[vehicles.size()][100];
			
			this.visitedPickupsCon = new GRBConstr[pickupNodes.size()];
			this.oneVisitCon = new GRBConstr[vehicles.size()];
			
			this.oneVisitLeftHand = new GRBLinExpr[vehicles.size()];
			this.visitedPickupsLeftHand = new GRBLinExpr[pickupNodes.size()];
			
			this.dualVisitedPickupsCon = new Vector<Float>();
			this.dualOneVisitCon = new Vector<Float>();
			
			this.col = new GRBColumn();
			
			Label firstLabel = new Label();
			firstLabel.profit = 0;
		
			
			this.visitedPickupsByVehicleOnRoute = new int[100][vehicles.size()][pickupNodes.size()];
			

	
			
			
			for(int k = 0; k < vehicles.size(); k++) {
				this.vehicles.get(k).vehicleRoutes = new Vector<Integer>();
				numberOfRoutes += 1;
				//System.out.println(vehicles.get(k).startDepot.number);
				//System.out.println(vehicles.get(k).vehicleRoutes);
				vehicles.get(k).vehicleRoutes.add(numberOfRoutes);
				for (int r : vehicles.get(k).vehicleRoutes) {
			//		System.out.println(numberOfRoutes);
					this.lambdaVars[k][r] = model.addVar(0, GRB.INFINITY, firstLabel.profit, GRB.CONTINUOUS, col, "lambda_"+k+r);
					this.objective.addTerm(firstLabel.profit, this.lambdaVars[k][r]);
					
				
				}
			}
			model.setObjective(objective, GRB.MAXIMIZE);
			
			model.update();
			
			// visited pickups constraint 
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				GRBLinExpr temp = new GRBLinExpr();
				for(int k = 0; k < vehicles.size(); k++) {
					for(int r : vehicles.get(k).vehicleRoutes) {
					//	System.out.println(r);
						temp.addTerm(visitedPickupsByVehicleOnRoute[r][k][i], lambdaVars[k][r]);
						System.out.println(visitedPickupsByVehicleOnRoute[r][k][i]);
					
					}
				}
				this.visitedPickupsCon[i] = model.addConstr(temp, GRB.LESS_EQUAL,1,"visitedPickupCon"+i);	
			}
			
			model.update();
			
			
			// one visit per vehicle constraint 
			
			for(int k = 0; k < vehicles.size(); k++) {
				GRBLinExpr temp = new GRBLinExpr();
				for (int r : vehicles.get(k).vehicleRoutes) {
					temp.addTerm(1, lambdaVars[k][r]);

				}
				this.oneVisitCon[k] = model.addConstr(temp, GRB.EQUAL, 1, "oneVisitCon"+k);		// skal egentlig v�re Equal 
			}
			
			model.update();

		}	
		
		public void addRoute(Label l) throws Exception{
			
			col2 = new GRBColumn();
			
			lambdaVars[l.vehicle.number][l.vehicle.vehicleRoutes.lastElement()] = model.addVar(0, GRB.INFINITY, l.profit, GRB.CONTINUOUS,  "lambda_"+l.vehicle.number+ numberOfRoutes);

			this.objective.addTerm(l.profit, this.lambdaVars[l.vehicle.number][l.vehicle.vehicleRoutes.lastElement()]);
			
			for(int i = 0; i < pickupNodes.size(); i++) {
		
				if(l.pickupNodesVisited.contains(pickupNodes.get(i).location)) {
					System.out.println("PICKUP" + i);
					visitedPickupsByVehicleOnRoute[l.vehicle.vehicleRoutes.lastElement()][l.vehicle.number][i] = 1;	
					System.out.println(visitedPickupsByVehicleOnRoute[l.vehicle.vehicleRoutes.lastElement()][l.vehicle.number][i]);
				}	
				
				model.chgCoeff(oneVisitCon[l.vehicle.number], lambdaVars[l.vehicle.number][l.vehicle.vehicleRoutes.lastElement()], 1);
				
				model.chgCoeff(visitedPickupsCon[i], lambdaVars[l.vehicle.number][l.vehicle.vehicleRoutes.lastElement()], visitedPickupsByVehicleOnRoute[l.vehicle.vehicleRoutes.lastElement()][l.vehicle.number][i]);
			//	col2.addTerm(visitedPickupsByVehicleOnRoute[l.vehicle.vehicleRoutes.lastElement()][l.vehicle.number][i], visitedPickupsCon[i]);
			//	col2.addTerm(1, oneVisitCon[l.vehicle.number]);

				model.update();
			}

		}
		
		
		public void solveProblem() throws Exception {
			
			buildProblem();
			model.optimize();

			
			//print
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r : vehicles.get(k).vehicleRoutes) {
					System.out.println(lambdaVars[k][r].get(GRB.StringAttr.VarName)  + " " +lambdaVars[k][r].get(GRB.DoubleAttr.X));
				}
			}
			
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
				this.dualVisitedPickupsCon.add(dualPickup_i);
				System.out.println("DUAL: " + dualPickup_i);
				//System.out.println("HER");
				//System.out.println(dualVisitedPickupsCon.get(i));
				model.update();
			}
			
			for(int k = 0; k < vehicles.size(); k++) {
				float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
				this.dualOneVisitCon.add(dualVehicle_k);
				System.out.println("DUAL: " + dualVehicle_k);
				//System.out.println("HER");
				//System.out.println(dualOneVisitCon.get(k));
				model.update();
			}
			

			Label bestLabel = new Label();
			bestLabel.reducedCost = -100000000;
			System.out.println("Objective value" +model.get(GRB.DoubleAttr.ObjVal));
			
			while(bestLabel.reducedCost < 0) {
			
			for(int k = 0; k < vehicles.size(); k++) {
				nodes.set(0, vehicles.get(k).startDepot);
				for (int i = 0; i < nodes.size(); i++) {
					System.out.println(nodes.get(i).location);
				}
				
				builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot, inputdata, pw, vehicles);
				Vector<Label> list = builder.BuildPaths(vehicles.get(k), dualVisitedPickupsCon, dualOneVisitCon);
				bestLabel = builder.findBestLabel(list);
				numberOfRoutes += 1;
				vehicles.get(k).vehicleRoutes.add(numberOfRoutes);
				System.out.println ("HER: " +numberOfRoutes);
				addRoute(bestLabel);
				
				model.optimize();

				for(int i = 0; i < pickupNodes.size(); i++) {
					float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
					this.dualVisitedPickupsCon.add(dualPickup_i);
					System.out.println("DUAL_pickup: " + dualPickup_i);
					//System.out.println("HER");
					//System.out.println(dualVisitedPickupsCon.get(i));
					model.update();
				}
				
			
				float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
				this.dualOneVisitCon.add(dualVehicle_k);
				model.update();
				System.out.println("DUAL_vehicle: " + dualVehicle_k);

			}
				
				
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r : vehicles.get(k).vehicleRoutes) {
					System.out.println(lambdaVars[k][r].get(GRB.StringAttr.VarName)  + " " +lambdaVars[k][r].get(GRB.DoubleAttr.X));
				}
			}
			
			}

		model.dispose();
	    env.dispose();
			
		}
	}
	

    // Optimize model

 //  

  //  System.out.println(lambda.get(GRB.StringAttr.VarName)
//                         + " " +lambda.get(GRB.DoubleAttr.X));
   // System.out.println(y.get(GRB.StringAttr.VarName)
    //                   + " " +y.get(GRB.DoubleAttr.X));
   // System.out.println(z.get(GRB.StringAttr.VarName)
    //                   + " " +z.get(GRB.DoubleAttr.X));

   // System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));
		  
	//   }