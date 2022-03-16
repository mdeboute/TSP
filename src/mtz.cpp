#include "gurobi_c++.h"
#include "parser.hpp"
#include <cstring>
using namespace std;

int main(int argc,
         char *argv[])
{
     bool verbose = true;
     if (argv[2] != NULL && strcmp(argv[2], "-nv") == 0)
     {
          verbose = false;
     }
     // parse and save the data
     vector<vector<int>> c = parse(argv[1]);
     int n = c.size();

     GRBVar **x = nullptr;
     GRBVar *u = nullptr;
     try
     {
          // --- Creation of the Gurobi environment ---
          if (verbose)
               cout << "--> Creating the Gurobi environment" << endl;
          GRBEnv env = GRBEnv(true);
          // env.set("LogFile", "mip1.log"); ///< prints the log in a file
          env.start();

          // --- Creation of the Gurobi model ---
          if (verbose)
               cout << "--> Creating the Gurobi model" << endl;
          GRBModel model = GRBModel(env);

          if (not verbose)
          {
               model.set(GRB_IntParam_OutputFlag, 0);
          }

          // --- Creation of the variables ---
          if (verbose)
               cout << "--> Creating the variables" << endl;

          x = new GRBVar *[n];
          u = new GRBVar[n];

          for (size_t j = 0; j < n; ++j)
          {
               x[j] = new GRBVar[n];
               for (size_t i = 0; i < n; ++i)
               {
                    stringstream ss;
                    ss << "x(" << j << "," << i << ")";
                    x[j][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, ss.str());
               }

               stringstream ss;
               ss << "u(" << j << ")";
               u[j] = model.addVar(0.0, 1000.0, 0.0, GRB_INTEGER, ss.str());
          }

          // --- Creation of the objective function ---
          if (verbose)
               cout << "--> Creating the objective function" << endl;
          GRBLinExpr obj = 0;
          for (size_t i = 0; i < n; ++i)
          {
               for (size_t j = 0; j < n; ++j)
               {
                    if (i != j)
                         obj += c[j][i] * x[j][i];
               }
          }
          model.setObjective(obj, GRB_MINIMIZE);

          // --- Creation of the constraints ---
          if (verbose)
               cout << "--> Creating the constraints" << endl;

          // Respect flot
          for (size_t j = 0; j < n; ++j)
          {
               GRBLinExpr flot1 = 0;
               GRBLinExpr flot2 = 0;
               for (size_t i = 0; i < n; ++i)
               {
                    if (i != j)
                    {
                         flot1 += x[j][i];
                         flot2 += x[i][j];
                    }
               }
               stringstream ss;
               ss << "Flot1(" << j << ")";
               model.addConstr(flot1 == 1, ss.str());
               ss << "Flot2(" << j << ")";
               model.addConstr(flot2 == 1, ss.str());
          }

          // Elim. sous-tours
          for (size_t i = 1; i < n; ++i)
          {
               for (size_t j = 1; j < n; ++j)
               {
                    GRBLinExpr lhs = u[i] - u[j] + (n - 1) * x[j][i];
                    stringstream ss;
                    ss << "Sous-tours(" << j << "," << i << ")";
                    model.addConstr(lhs <= n - 2, ss.str());
               }
          }

          // Optimize model
          // --- Solver configuration ---
          if (verbose)
               cout << "--> Configuring the solver" << endl;
          model.set(GRB_DoubleParam_TimeLimit, 600.0); //< sets the time limit (in seconds)
          model.set(GRB_IntParam_Threads, 1);          //< limits the solver to single thread usage

          // --- Solver launch ---
          if (verbose)
               cout << "--> Running the solver" << endl;
          model.optimize();
          // model.write("model.lp"); //< Writes the model in a file

          // --- Solver results retrieval ---
          if (verbose)
               cout << "--> Retrieving solver results " << endl;

          int status = model.get(GRB_IntAttr_Status);
          if (status == GRB_OPTIMAL || (status == GRB_TIME_LIMIT && model.get(GRB_IntAttr_SolCount) > 0))
          {
               // the solver has computed the optimal solution or a feasible solution (when the time limit is reached before proving optimality)
               if (verbose)
               {
                    cout << "Success! (Status: " << status << ")" << endl; //< prints the solver status (see the gurobi documentation)
                    cout << "--> Printing results " << endl;
               }

               cout << "Result: ";
               cout << argv[1] << "; ";
               cout << "runtime = " << model.get(GRB_DoubleAttr_Runtime) << " sec; ";
               cout << "objective value = " << model.get(GRB_DoubleAttr_ObjVal) << endl; //< gets the value of the objective function for the best computed solution (optimal if no time limit)

               if (verbose)
               {
                    int i = 0;
                    for (size_t j = 0; j < n; j++)
                    {
                         if (x[j][i].get(GRB_DoubleAttr_X) == 1.0)
                         {
                              cout << "ville " << i << " --> "
                                   << "ville " << j << endl;
                              i = j;
                              if (i == 0)
                                   break;
                              j = -1;
                         }
                    }
               }
               // model.write("solution.sol"); //< Writes the solution in a file
          }
          else
          {
               // the model is infeasible (maybe wrong) or the solver has reached the time limit without finding a feasible solution
               cerr << "Fail! (Status: " << status << ")" << endl; //< see status page in the Gurobi documentation
          }
     }
     catch (GRBException e)
     {
          cout << "Error code = " << e.getErrorCode() << endl;
          cout << e.getMessage() << endl;
     }
     catch (...)
     {
          cout << "Exception during optimization" << endl;
     }

     delete[] u;

     for (size_t j = 0; j < n; ++j)
     {
          delete[] x[j];
     }

     delete[] x;

     return 0;
}