import ilog.concert.*;
import ilog.cplex.*;

public class TSP {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		TSP.solveModel(25);
	}

	public static void solveModel(int n) {
	//Create random position and find Cij
	double[] xPos = new double[n];
	double[] yPos = new double[n];
	for(int i=0; i<n;i++) {
		xPos[i]=Math.random()*100;
		yPos[i]=Math.random()*100;
	}
	double[][] c = new double[n][n];
	for(int i =0;i<n;i++) {
		for(int j=0;j<n;j++) {
			c[i][j]=Math.sqrt(Math.pow(xPos[i]-xPos[j], 2)+Math.pow(yPos[i]-yPos[j], 2));
		}
	}
	
	//Model creation
	try {
		IloCplex cplex = new IloCplex();
	//Decision variables
	IloNumVar[][] x = new IloNumVar[n][];  //the last one has to be empty
	for(int i =0;i<n;i++) {
		x[i]=cplex.boolVarArray(n);
	}
		
	IloNumVar[] u = cplex.numVarArray(n,0,Double.MAX_VALUE);
	
	//Objective Function
	//min sum.sum(c[i][j.x[i][j) for all i, j: i!=j
	IloLinearNumExpr obj = cplex.linearNumExpr();
	for(int i =0;i<n;i++) {
		for(int j =0;j<n;j++) {
			if(j!=i) {
				obj.addTerm(c[i][j],x[i][j]);
			}
		}
	}
	cplex.addMinimize(obj);
	
	//Constraint I
	//sum(x[i][j] =1 from i=1 to n,i!=j for all j
	for(int j =0;j<n;j++) {
		IloLinearNumExpr expression = cplex.linearNumExpr();
		for(int i=0;i<n;i++) {
			if(i!=j) {
				expression.addTerm(1,x[i][j]);
			}
		}
		cplex.addEq(expression, 1);
	}
	
	//Constraint II
	//sum(x[i][j] =1 from j=1 to n,j!=i for all i
	for(int i =0;i<n;i++) {
		IloLinearNumExpr expression = cplex.linearNumExpr();
		for(int j=0;j<n;j++) {
			if(j!=i) {
				expression.addTerm(1,x[i][j]);
			}
		}
		cplex.addEq(expression, 1);
	}
	
	//Last constraint to avoid sub route
	// u[i]-u[j] +(n-1)x[i][j] <= n-2 for i,j = 1 to n except{1} first node and i!=j
	for(int i =1;i<n;i++) {   //1st city is not considered
		for(int j = 1; j<n;j++) {
			if(i!=j) {
				IloLinearNumExpr expr = cplex.linearNumExpr();
				expr.addTerm(1, u[i]);
				expr.addTerm(-1, u[j]);
				expr.addTerm(n-1, x[i][j]);
				cplex.addLe(expr, n-2);
				
			}
		}
	}
	cplex.solve();
	System.out.println();
	System.out.println("Objective Value: "+cplex.getObjValue());
	// Print the status of the objective value
	System.out.println(cplex.getStatus());
	cplex.end();
	
	
	}catch (IloException exc) {
		exc.printStackTrace();
	}
	
	}
	
}
