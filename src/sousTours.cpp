#include "gurobi_c++.h"
#include "parser.hpp"
#include <stack>
using namespace std;


/**
   This callback ensures that x \leq 1.
 */
class SeparationCallback : public GRBCallback {
public:
    GRBVar** x;
    int n;

    /**
       The constructor is used to get a pointer to the variables that are needed.
     */
    SeparationCallback(GRBVar** _x, int _n) {
        x = _x;
        n = _n;
    }

protected:

    void callback() {
        try {
            if (where == GRB_CB_MIPSOL) {
                GRBLinExpr tour = 0;
                int taille = 0;
                int i = 0;
                for (size_t j = 0; j < n; ++j)
                {
                    int xVal = (int)getSolution(x[i][j]);
                    if (xVal == 1)
                    {
                        tour += x[i][j];
                        taille += 1;
                        i = j;
                        if (i == 0) break;
                        j = -1;
                    }
                }
                cout << endl << "#######" << taille << "#######" << endl;
                if (taille < n) { //sous-tour existe
                    cout << "Constraint not satisfied : sous tour de taille " << taille << " existe. Adding this constraint." << endl;
                    addLazy(tour <= taille - 1);
                }
            }
        }
        catch (GRBException e) {
            cout << "Error number: " << e.getErrorCode() << endl;
            cout << e.getMessage() << endl;
        }
        catch (...) {
            cout << "Error during callback" << endl;
        }
    }

};



int main(int argc,
    char* argv[])
{
    // parse and save the data
    vector<vector<int> > c = parse(argv[1]);
    int n = c.size();

    GRBVar** x = nullptr;
    GRBVar* u = nullptr;
    try
    {
        // --- Creation of the Gurobi environment ---
        cout << "--> Creating the Gurobi environment" << endl;
        GRBEnv env = GRBEnv(true);
        // env.set("LogFile", "mip1.log"); ///< prints the log in a file
        env.start();

        // --- Creation of the Gurobi model ---
        cout << "--> Creating the Gurobi model" << endl;
        GRBModel model = GRBModel(env);

        // --- Creation of the variables ---
        cout << "--> Creating the variables" << endl;

        x = new GRBVar * [n];

        for (size_t i = 0; i < n; ++i)
        {
            x[i] = new GRBVar[n];
            for (size_t j = 0; j < n; ++j)
            {
                stringstream ss;
                ss << "x(" << i << "," << j << ")";
                x[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, ss.str());
            }
        }

        // --- Creation of the objective function ---
        cout << "--> Creating the objective function" << endl;
        GRBLinExpr obj = 0;
        for (size_t i = 0; i < n; ++i)
        {
            for (size_t j = 0; j < n; ++j)
            {
                obj += c[i][j] * x[i][j];
            }
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // --- Creation of the constraints ---
        cout << "--> Creating the constraints" << endl;

        // Respect flot 1
        for (size_t j = 0; j < n; ++j)
        {
            GRBLinExpr flot1 = 0;
            for (size_t i = 0; i < n; ++i)
            {
                flot1 += x[i][j];
            }
            stringstream ss;
            ss << "Flot1(" << j << ")";
            model.addConstr(flot1 == 1, ss.str());
        }

        // Respect flot 2
        for (size_t i = 0; i < n; ++i)
        {
            GRBLinExpr flot2 = 0;
            for (size_t j = 0; j < n; ++j)
            {
                flot2 += x[i][j];
            }
            stringstream ss;
            ss << "Flot2(" << i << ")";
            model.addConstr(flot2 == 1, ss.str());
        }

        //Callback
        SeparationCallback* myCallback = new SeparationCallback(x, n); // passing variable x to the solver callback  
        model.setCallback(myCallback); // adding the callback to the model 

        /*
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
        */

        // Optimize model
        // --- Solver configuration ---
        cout << "--> Configuring the solver" << endl;
        model.set(GRB_DoubleParam_TimeLimit, 600.0); //< sets the time limit (in seconds)
        model.set(GRB_IntParam_Threads, 1);          //< limits the solver to single thread usage

        // --- Solver launch ---
        cout << "--> Running the solver" << endl;
        model.optimize();
        model.write("model.lp"); //< Writes the model in a file

        // --- Solver results retrieval ---
        cout << "--> Retrieving solver results " << endl;

        int status = model.get(GRB_IntAttr_Status);
        if (status == GRB_OPTIMAL || (status == GRB_TIME_LIMIT && model.get(GRB_IntAttr_SolCount) > 0))
        {
            // the solver has computed the optimal solution or a feasible solution (when the time limit is reached before proving optimality)
            cout << "Succes! (Status: " << status << ")" << endl; //< prints the solver status (see the gurobi documentation)
            cout << "Runtime : " << model.get(GRB_DoubleAttr_Runtime) << " seconds" << endl;

            cout << "--> Printing results " << endl;
            // model.write("solution.sol"); //< Writes the solution in a file
            cout << "Objective value = " << model.get(GRB_DoubleAttr_ObjVal) << endl; //<gets the value of the objective function for the best computed solution (optimal if no time limit)

            int i = 0;
            for (size_t j = 0; j < n; j++)
            {
                if (x[j][i].get(GRB_DoubleAttr_X) == 1.0)
                {
                    cout << "ville " << i << " --> "
                        << "ville " << j << endl;
                    i = j;
                    if (i == 0) break;
                    j = -1;
                }
            }       
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