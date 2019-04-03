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
		public int numberOfRoutes;
		//public Vector<Vector<Vector<Integer>>> visitedPickupsByVehicleOnRoute;
		int[][][] visitedPickupsByVehicleOnRoute;
	
		 
		//private Hashtable<Integer, Route> pathList;
		
		public GRBLinExpr visitedPickupsLeftHand[];
		public GRBLinExpr oneVisitLeftHand[];
		
		public GRBColumn col;
		
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
			this.lambdaVars = new GRBVar[vehicles.size()][numberOfRoutes];
			
			this.visitedPickupsCon = new GRBConstr[pickupNodes.size()];
			this.oneVisitCon = new GRBConstr[vehicles.size()];
			
			this.oneVisitLeftHand = new GRBLinExpr[vehicles.size()];
			this.visitedPickupsLeftHand = new GRBLinExpr[pickupNodes.size()];
			
			this.dualVisitedPickupsCon = new Vector<Float>();
			this.dualOneVisitCon = new Vector<Float>();
			
			this.col = new GRBColumn();
			
			//this.visitedPickupsByVehicleOnRoute = new Vector<Vector<Vector<Integer>>>();
			
			this.visitedPickupsByVehicleOnRoute = new int[100][vehicles.size()][pickupNodes.size()];
			
			
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r = 0; r < numberOfRoutes; r++) {
			//		System.out.println(numberOfRoutes);
					this.lambdaVars[k][r] = model.addVar(0, GRB.INFINITY, profit, GRB.CONTINUOUS, col, "lambda_"+k+r);
					this.objective.addTerm(profit, this.lambdaVars[k][r]);
					
				//	   System.out.println(lambdaVars[k][r].get(GRB.StringAttr.VarName) + " " +lambdaVars[k][r].get(GRB.DoubleAttr.X));
				}
			}
			model.setObjective(objective, GRB.MAXIMIZE);
			
			model.update();
			
			// visited pickups constraint 
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				GRBLinExpr temp = new GRBLinExpr();
				for(int k = 0; k < vehicles.size(); k++) {
					for(int r = 0; r < numberOfRoutes; r++) {
					//	System.out.println(r);
						temp.addTerm(visitedPickupsByVehicleOnRoute[i][k][r], lambdaVars[k][r]);
						//System.out.println(visitedPickupsByVehicleOnRoute.get(r).get(k).get(i));
						//this.visitedPickupsLeftHand[i].addTerm(0, this.lambdaVars[k][r]);
						
						//double dualPickup_i = visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
						//float dualPickup_ii = (float) dualPickup_i;
						//this.dualVisitedPickupsCon.add(dualPickup_ii);
					}
				}
				this.visitedPickupsCon[i] = model.addConstr(temp, GRB.LESS_EQUAL,1,"visitedPickupCon"+i);	
			}
			
			model.update();
			
			
			// one visit per vehicle constraint 
			
			for(int k = 0; k < vehicles.size(); k++) {
				GRBLinExpr temp = new GRBLinExpr();
				for (int r = 0; r < numberOfRoutes; r++) {
					temp.addTerm(1, lambdaVars[k][r]);
				//	this.oneVisitLeftHand[k].addTerm(1, this.lambdaVars[k][r]);
						
					//float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
					//this.dualOneVisitCon.add(dualVehicle_k);
				}
				this.oneVisitCon[k] = model.addConstr(temp, GRB.LESS_EQUAL, 1, "oneVisitCon"+k);		// skal egentlig være Equal 
			}
			
			model.update();

		}	
		
		public void addRoute(Label l) throws Exception{
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				GRBLinExpr temp = new GRBLinExpr();
				GRBLinExpr temp2 = new GRBLinExpr();
			//	for(int k = 0; k < vehicles.size(); k++) {
			//	System.out.println(l.vehicle.number);
			//		if(l.vehicle.number == k) {
				if(l.pickupNodesVisited.contains(pickupNodes.get(i).location)) {
					visitedPickupsByVehicleOnRoute[l.bestLabelNumber][l.vehicle.number][i] = 1;		
				}	
				col.addTerm(visitedPickupsByVehicleOnRoute[i][l.vehicle.number][l.bestLabelNumber], visitedPickupsCon[i]);
				col.addTerm(1, oneVisitCon[l.vehicle.number]);
					//	temp.addTerm(visitedPickupsByVehicleOnRoute[i][k][l.bestLabelNumber], lambdaVars[k][l.bestLabelNumber]);
					//	temp2.addTerm(1, lambdaVars[k][l.bestLabelNumber]);
					//	System.out.println("tull");
					//	System.out.println(l.bestLabelNumber);
					//	visitedPickupsCon[i].addTerm(1, this.lambdaVars[k][l.bestLabelNumber]);
					//	model.chgCoeff(this.visitedPickupsCon[i], this.lambdaVars[k][l.bestLabelNumber], 1);	
						
			//		}
				model.update();
			//	lambdaVars[l.vehicle.number][l.bestLabelNumber] = model.addVar(0, GRB.INFINITY, l.profit, GRB.CONTINUOUS, "lambda_"+l.vehicle.number+l.bestLabelNumber);
				System.out.println(l.vehicle.number);
				System.out.println(l.bestLabelNumber);
				this.objective.addTerm(l.profit, this.lambdaVars[l.vehicle.number][l.bestLabelNumber]);
					//this.visitedPickupsCon[i] = model.addConstr(temp, GRB.LESS_EQUAL,1,"visitedPickupCon"+i);	
					//this.oneVisitCon[l.vehicle.number] = model.addConstr(temp2, GRB.LESS_EQUAL, 1, "oneVisitCon"+l.vehicle.number);
					
					//else {
					//	model.chgCoeff(this.visitedPickupsCon[i], this.lambdaVars[k][l.bestLabelNumber], 0);
					//}
				//}
			}
			
			
			model.update();
			
			//for(int k = 0; k < vehicles.size(); k++) {
			//	for (int r = 0; r < numberOfRoutes; r++) {
			//			model.chgCoeff(this.oneVisitCon[k], this.lambdaVars[k][r], 1);
			//	}
			//}
			
			model.update();
		}
		
		
		public void solveProblem() throws Exception {
			
			buildProblem();
			model.optimize();
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
				this.dualVisitedPickupsCon.add(dualPickup_i);
				//System.out.println("HER");
				//System.out.println(dualVisitedPickupsCon.get(i));
			}
			
			for(int k = 0; k < vehicles.size(); k++) {
				float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
				this.dualOneVisitCon.add(dualVehicle_k);
				//System.out.println("HER");
				//System.out.println(dualOneVisitCon.get(k));
			}
			
			// Call initial master problem 
			//numberOfRoutes = 1;
			
			//for(int p = 0; p < pickupNodes.size(); p++) {
			//	float zero = (float) 0.0;
			//	dualVisitedPickupsCon.add(zero);
			//}
			
			//for(int k = 0; k < vehicles.size(); k++) {
			//	float zero = (float) 0.0;
			//	dualOneVisitCon.add(zero);
			//}
			
			//Vehicle vehicle = vehicles.get(0);
			//System.out.println(vehicle.number);
			Label bestLabel = new Label();
			
			while(bestLabel.reducedCost < 0  || model.get(GRB.DoubleAttr.ObjVal) == 0) {
			
			for(int k = 0; k < vehicles.size(); k++) {
				builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot, inputdata, pw, vehicles);
				Vector<Label> list = builder.BuildPaths(vehicles.get(k), dualVisitedPickupsCon, dualOneVisitCon);
				bestLabel = builder.findBestLabel(list);
				//profit = bestLabel.profit;
				addRoute(bestLabel);
			}
			model.optimize();
			
			}
			/*
			while((bestLabel.reducedCost < 0 && vehicles.size() != 1) || (bestLabel.reducedCost - profit == 0 && numberOfRoutes == 1) ) {
				
				//buildProblem();
				System.out.println("while");
				addRoute(bestLabel);
				model.optimize();
				
				for(int i = 0; i < pickupNodes.size(); i++) {
					float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
					this.dualVisitedPickupsCon.add(dualPickup_i);
					System.out.println("HER");
					System.out.println(dualVisitedPickupsCon.get(i));
				}
				
				for(int k = 0; k < vehicles.size(); k++) {
					float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
					this.dualOneVisitCon.add(dualVehicle_k);
					//System.out.println("HER");
					//System.out.println(dualOneVisitCon.get(k));
				}
				
				for(int k = 0; k < vehicles.size(); k++) {
					for (int r = 0; r < numberOfRoutes; r++) {
						System.out.println(lambdaVars[k][r].get(GRB.StringAttr.VarName)  + " " +lambdaVars[k][r].get(GRB.DoubleAttr.X));
					}}
						
				for(int k = 0; k < vehicles.size(); k++) {
					System.out.println(k);
					list = builder.BuildPaths(vehicles.get(k), dualVisitedPickupsCon, dualOneVisitCon);
					bestLabel = builder.findBestLabel(list);
					profit = bestLabel.profit;
					numberOfRoutes += 1;	
					
					buildProblem();
					addRoute(bestLabel);
					model.optimize();
					
					for( k = 0; k < vehicles.size(); k++) {
						for (int r = 0; r < numberOfRoutes; r++) {
							System.out.println(lambdaVars[k][r].get(GRB.StringAttr.VarName)  + " " +lambdaVars[k][r].get(GRB.DoubleAttr.X));
						}}
					
					for(int i = 0; i < pickupNodes.size(); i++) {
						float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
						this.dualVisitedPickupsCon.add(dualPickup_i);
						System.out.println("HER");
						System.out.println(dualVisitedPickupsCon.get(i));
					}
					
					for(k = 0; k < vehicles.size(); k++) {
						float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
						this.dualOneVisitCon.add(dualVehicle_k);
						System.out.println("HER2");
						System.out.println(dualOneVisitCon.get(k));
					}
					
				}
		}*/
	      
		model.dispose();
	    env.dispose();
			
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
			   
		   
	}
	

