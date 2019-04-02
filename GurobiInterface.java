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
		 
		//private Hashtable<Integer, Route> pathList;
		
		public GRBLinExpr visitedPickupsLeftHand[];
		public GRBLinExpr oneVisitLeftHand[];
		
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
			
			
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r = 0; r < numberOfRoutes; r++) {
			//		System.out.println(numberOfRoutes);
					this.lambdaVars[k][r] = model.addVar(0, GRB.INFINITY, profit, GRB.CONTINUOUS, "lambda_"+k+r);
					this.objective.addTerm(profit, this.lambdaVars[k][r]);
					model.setObjective(objective, GRB.MAXIMIZE);
				//	   System.out.println(lambdaVars[k][r].get(GRB.StringAttr.VarName) + " " +lambdaVars[k][r].get(GRB.DoubleAttr.X));
				}
			}
			
			model.update();
			
			// visited pickups constraint 
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				for(int k = 0; k < vehicles.size(); k++) {
					for(int r = 0; r < numberOfRoutes; r++) {
					//	System.out.println(r);
						//this.visitedPickupsLeftHand[i].addTerm(0, this.lambdaVars[k][r]);
						this.visitedPickupsCon[i] = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL,1,"visitedPickupCon"+i);	
						//double dualPickup_i = visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
						//float dualPickup_ii = (float) dualPickup_i;
						//this.dualVisitedPickupsCon.add(dualPickup_ii);
					}
				}			
			}
			
			model.update();
			
			// one visit per vehicle constraint 
			
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r = 0; r < numberOfRoutes; r++) {
				//	this.oneVisitLeftHand[k].addTerm(1, this.lambdaVars[k][r]);
					this.oneVisitCon[k] = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL, 1, "oneVisitCon"+k);		// skal egentlig være Equal 	
					//float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
					//this.dualOneVisitCon.add(dualVehicle_k);
				}
			}
			
			model.update();

		}	
		
		public void addRoute(Label l) throws Exception{
			
			for(int i = 0; i < l.pickupNodesVisited.size(); i++) {
				for(int k = 0; k < vehicles.size(); k++) {
			//	System.out.println(l.vehicle.number);
					if(l.vehicle.number == k) {
					//	System.out.println("tull");
					//	System.out.println(l.bestLabelNumber);
						model.chgCoeff(this.visitedPickupsCon[i], this.lambdaVars[k][l.bestLabelNumber], 1);	
						
					}
					//else {
					//	model.chgCoeff(this.visitedPickupsCon[i], this.lambdaVars[k][l.bestLabelNumber], 0);
					//}
				}
			}
			
			model.update();
			
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r = 0; r < numberOfRoutes; r++) {
						model.chgCoeff(this.oneVisitCon[k], this.lambdaVars[k][r], 1);
				}
			}
			
			model.update();
		}
		
		
		public void solveProblem() throws Exception {
			
			for(int p = 0; p < pickupNodes.size(); p++) {
				float zero = (float) 0.0;
				dualVisitedPickupsCon.add(zero);
			}
			
			for(int k = 0; k < vehicles.size(); k++) {
				float zero = (float) 0.0;
				dualOneVisitCon.add(zero);
			}
			
			Vehicle vehicle = vehicles.get(0);
			//System.out.println(vehicle.number);
			builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot, inputdata, pw, vehicles);
			Vector<Label> list = builder.BuildPaths(vehicle, dualVisitedPickupsCon, dualOneVisitCon);
			Label bestLabel = builder.findBestLabel(list);
			profit = bestLabel.profit;
			numberOfRoutes = 1;
			
			
			
			//if(bestLabel.reducedCost - profit == 0) {
				
			//	buildProblem();
			//	System.out.println("while");
			//	addRoute(bestLabel);
			//	model.optimize();
				
				
			
	
		//		list = builder.BuildPaths(vehicle, dualVisitedPickupsCon, dualOneVisitCon);
			//	bestLabel = builder.findBestLabel(list);
			//	profit = bestLabel.profit;
					
			//	numberOfRoutes += 1;	
			
			
			while(bestLabel.reducedCost - profit >= 0 && vehicles.size() != 1) {
				
				buildProblem();
				System.out.println("while");
				addRoute(bestLabel);
				model.optimize();
				
				for(int i = 0; i < pickupNodes.size(); i++) {
					float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
					this.dualVisitedPickupsCon.add(dualPickup_i);
				}
				
				for(int k = 0; k < vehicles.size(); k++) {
					float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
					this.dualOneVisitCon.add(dualVehicle_k);
					System.out.println("HER");
					System.out.println(dualOneVisitCon.get(k));
				}
	
				for(int k = 0; k < vehicles.size(); k++) {
					list = builder.BuildPaths(vehicles.get(k), dualVisitedPickupsCon, dualOneVisitCon);
					bestLabel = builder.findBestLabel(list);
					profit = bestLabel.profit;
					
					numberOfRoutes += 1;	
				}
			//}
		}
	      
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


			
		      // Dispose of model and environment
		   
	//      model.dispose();
	 //     env.dispose();
			   
			   
			  
		//   }
			   
		   
	}
	

