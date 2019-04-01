	import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Vector;
import gurobi.GRB.DoubleAttr;
import gurobi.GRB.IntAttr;
import gurobi.*;

	public class GurobiInterface {
		public Vector<Node> route;
		public double profit;
		//GRBConstr c1;
	    //GRBConstr c2;
	    GRBEnv    env   = new GRBEnv("mip1.log");
	    GRBModel  model = new GRBModel(env);
		public InstanceData inputdata;
		int numRoutes;
		public Vector<Node> pickupNodes;
		public Vector<Node> deliveryNodes;
		public Vector<Node> nodes;
		public Vector<Node> depot;
		public PrintWriter pw;
		public Vector<Route> routes;
		
		
		public GurobiInterface(PathBuilder pathBuilder, Route route, InstanceData inputdata, Vector<Node> PickupNodes) throws GRBException {
			this.route = route.path;
			this.profit = route.profit;
			this.inputdata = inputdata;
			//GRBConstr c1;
		    //GRBConstr c2;
		    //GRBVar lambda[][];
		    //this.env  = new GRBEnv("mip1.log");
		    //GRBModel  model = new GRBModel(env);
		    int numRoutes = pathBuilder.numRoutes;
		}
		
			
		
		
		
		public void MasterProblem (int profit, Route route)  {
			try {
				
				double dual1 = 0;
				double dual2 = 0;
		
			while(dual1 > 0.0 && dual2 > 0.0) {
				
				PathBuilder builder;
				builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot,inputdata, pw, routes);
				builder.BuildPaths();
				
				
			int numberOfVehicles = inputdata.numberOfVehicles;

			GRBVar[][] lambdas = new GRBVar[numberOfVehicles][numRoutes];
			for (int i = 0; i < numberOfVehicles; i++) {
				for (int j = 0; j < numRoutes; j++) {
					lambdas[i][j] = model.addVar(0.0, GRB.INFINITY, profit, GRB.CONTINUOUS, null);
	    		}
			}
			
			
			// Objective function: profit * lambda 
			
			//for (int i = 0; i < numberOfVehicles; i++) {
			//	for (int j = 0; j < numRoutes; j++) {
					GRBLinExpr objective = new GRBLinExpr();
					objective = new GRBLinExpr();
					objective.addTerms(profit, lambdas);
					model.setObjective(objective, GRB.MAXIMIZE);
	    	//	}
			//}
			
			// Constraint 1: Visited(ikr) * lambda <= 1
			
			GRBLinExpr c1;
			for (int i = 0; i < pickupNodes.size(); i++) {	
				c1 = new GRBLinExpr();
				c1.addTerms(Visited[i], lambdas);
				model.addConstr(c1, GRB.LESS_EQUAL, 1.0, "c1");
			}
			
			dual1 = c1.get(GRB.DoubleAttr.Pi);
			
			
			GRBLinExpr c2;
			for(int i = 0; i < numberOfVehicles; i++) {
				c2 = new GRBLinExpr();
				c2.addTerms(null, lambdas[i]);
				model.addConstr(c2, GRB.EQUAL, 1.0, "c2");
			} 
			
			dual2 = c2.get(GRB.DoubleAttr.Pi);
	       
		//lambda[numberOfVehicles][numRoutes] = model.addVars( null, null, null, null, null);
	      
//	      GRBVar A = model.addVar(3, 0.0, 1.0, GRB.BINARY, "A");
	      //GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "z");
	      //GRBVar P = model.addVar(2, -GRB.INFINITY, GRB.INFINITY, GRB.CONTINUOUS, "profit");
	      

	      // Set objective: maximize profit * lambda
		/*
	      GRBLinExpr expr = new GRBLinExpr();
	      expr = quicksum(model.getVars());
	      expr.addTerms(profit, lambda[][]); 
	      model.setObjective(expr, GRB.MAXIMIZE);

	      // Add constraint: visited freight orders * lambda <= 1

	      expr = new GRBLinExpr();
	      expr.addTerm(visited, lambda); 
	      model.addConstr(expr, GRB.LESS_EQUAL, 1.0, "c1");
	      
	      double dual1 = c1.get(GRB.DoubleAttr.Pi);

	      // Add constraint: lambda = 1 forall vehicles

	      expr = new GRBLinExpr();
	      expr.addTerm(1.0, lambda); 
	      model.addConstr(expr, GRB.EQUAL, 1.0, "c2");
	      
	      double dual2 = c2.get(GRB.DoubleAttr.Pi);
	      
	      
	      // Optimize model

	      model.optimize();

	      System.out.println(lambda.get(GRB.StringAttr.VarName)
	                         + " " +lambda.get(GRB.DoubleAttr.X));
	     // System.out.println(y.get(GRB.StringAttr.VarName)
	      //                   + " " +y.get(GRB.DoubleAttr.X));
	     // System.out.println(z.get(GRB.StringAttr.VarName)
	      //                   + " " +z.get(GRB.DoubleAttr.X));

	      System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));

	
	      */
			
			}
			
		      // Dispose of model and environment
		   
	      model.dispose();
	      env.dispose();
			   }
			   
			   catch (GRBException e) {System.out.println("error code: " + e.getErrorCode() + "." + e.getMessage());
			   }
		   }
			   
		   
	}
	

