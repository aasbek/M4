	
	import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Vector;
import gurobi.GRB.DoubleAttr;
import gurobi.GRB.IntAttr;
import gurobi.*;

	public class GurobiInterface {
		public Vector<Node> route;
		public float profit;
		GRBConstr c1;
	    GRBConstr c2;
	    GRBEnv    env   = new GRBEnv("mip1.log");
	    GRBModel  model = new GRBModel(env);
		public InstanceData inputdata;
		int numRoutes;
		
		
		
		public GurobiInterface(PathBuilder pathBuilder, Route route, InstanceData inputdata) throws GRBException {
			this.route = route.path;
			this.profit = route.profit;
			this.inputdata = inputdata;
			//GRBConstr c1;
		    //GRBConstr c2;
		    GRBVar lambda[][];
		    //this.env  = new GRBEnv("mip1.log");
		    //GRBModel  model = new GRBModel(env);
		    int numRoutes = pathBuilder.numRoutes;
		}
		
		//PathBuilder builder;
		//builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot,inputdata, pw);
		
	 // GRBVar lambda[Int][Int] = new GRBVar[Int][Int]();
	//    try {
	   //   new GRBVar lambda[][] = new GRBVar[][]();
		   public void MasterProblem (int profit, Route route)  {
			   try {
			   
			   int numberOfVehicles = inputdata.numberOfVehicles;
	      // Create variables
	  
	      //PathBuilder subproblem = new PathBuilder(); 
	    GRBVar[][] lambdas = new GRBVar[numberOfVehicles][numRoutes];
	    for (int i = 0; i < numberOfVehicles; i++) {
	    	for (int j = 0; j < numRoutes; j++) {
	    		lambdas[i][j] = model.addVar( 0.0, GRB.INFINITY, profit, GRB.CONTINUOUS, null);
	    	}
	    }
	       
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

	      // Dispose of model and environment
	      */
		   
	      model.dispose();
	      env.dispose();
			   }
			   
			   catch (GRBException e) {System.out.println("error code: " + e.getErrorCode() + "." + e.getMessage());
			   }
		   }
			   
		   
	}
	

