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
		
	    public InstanceData inputdata;
		public double profit;
		public Vector<Node> pickupNodes;
		public Vector<Node> deliveryNodes;
		public Vector<Node> nodes;
		public Vector<Node> depot;
		public PrintWriter pw;
		public Vector<Node> path;
		public Vector<Route> routes;
		public Vector<Vehicle> vehicles;
		public PathBuilder builder;
		public Route route;
		 
		//private Hashtable<Integer, Route> pathList;
		
		public GRBLinExpr visitedPickupsLeftHand[];
		public GRBLinExpr oneVisitLeftHand[];
		
		public GurobiInterface(InstanceData inputdata, Vector<Node> pickupNodes, Vector<Vehicle> vehicles) throws Exception {
			this.vehicles = vehicles; 
			this.builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot, inputdata, pw, routes, vehicles);
			this.path = route.path;
			this.profit = route.profit;
			this.inputdata = inputdata;
			buildProblem();
		}
		
		public void buildProblem() throws Exception {
			this.objective = new GRBLinExpr();
			
			this.variables = new Vector<GRBVar>();
			this.lambdaVars = new GRBVar[vehicles.size()][routes.size()];
			
			this.visitedPickupsCon = new GRBConstr[pickupNodes.size()];
			this.oneVisitCon = new GRBConstr[vehicles.size()];
			
			this.oneVisitLeftHand[vehicles.size()] = new GRBLinExpr();
			this.visitedPickupsLeftHand[pickupNodes.size()] = new GRBLinExpr();
			
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r = 0; r < routes.size(); r++) {
					this.lambdaVars[k][r] = model.addVar(0, GRB.INFINITY, profit, GRB.INTEGER, "lambda_"+k);
					this.objective.addTerm(profit, this.lambdaVars[k][r]);
					model.setObjective(objective, GRB.MAXIMIZE);
				}
			}
			
			model.update();
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				for(int k = 0; k < vehicles.size(); k++) {
					for(int r = 0; i < routes.size(); r++) {
						this.visitedPickupsLeftHand[i].addTerm(0, this.lambdaVars[k][r]);
						this.visitedPickupsCon[i] = model.addConstr(this.visitedPickupsLeftHand[i], GRB.LESS_EQUAL,1,"visitedPickupCon"+i);	
					}
				}			
			}
			
			model.update();
			
			for(int k = 0; k < vehicles.size(); k++) {
				for (int r = 0; r < routes.size(); r++) {
					this.oneVisitLeftHand[k].addTerm(1, this.lambdaVars[k][r]);
					this.oneVisitCon[k] = model.addConstr(this.oneVisitLeftHand[k], GRB.LESS_EQUAL, 1, "oneVisitCon"+k);		// skal egentlig være Equal 	
				}
			}
			
			model.update();

		}	
		
		public void addRoute(Route r) throws Exception{
			
			for(int i : r.pickupNodesVisited) {
				for(int k = 0; k < vehicles.size(); k++) {
					if(r.vehicle.number == k) {
						model.chgCoeff(this.visitedPickupsCon[i], this.lambdaVars[k][r.number], 1);
					}
				}
			}
		}
		
		//dual1 = c1.get(GRB.DoubleAttr.Pi);
		
	
		
	
	      // Optimize model

	   //   model.optimize();

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
	

