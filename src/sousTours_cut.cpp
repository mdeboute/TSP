#include "gurobi_c++.h"
#include "parser.hpp"
#include <stack>
#include <cstring>
using namespace std;

class Callback : public GRBCallback
{
public:
    GRBVar **_x;
    int n;

    /**
       The constructor is used to get a pointer to the variables that are needed.
     */
    Callback(GRBVar **x, int nb)
    {
        cout << "here" << endl;
        _x = x;
        n = nb;
    }

protected:
    void callback()
    {
        try
        {
            if (where == GRB_CB_MIPNODE && getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL)
            {
                vector<int> indices;
                GRBLinExpr tour = 0;
                int taille = 0;
                int i = 0;
                for (int j = 0; j < n; ++j)
                {
                    double xVal = getNodeRel(_x[i][j]);

                    cout << "x(" << i << "," << j << ") = " << xVal << endl;
                    if (xVal > 0)
                    {
                        cout << "x(" << i << "," << j << ")" << xVal << endl;
                        indices.push_back(i);
                        // tour += x[i][j];
                        taille += 1;
                        i = j;
                        if (i == 0)
                            break;
                        j = -1;
                    }
                }
                // cout << endl  << "#######" << taille << "#######" << endl;
                if (taille < n)
                { // sous-tour existe
                    // cout << tour << endl;
                    cout << "Constraint not satisfied : sous tour de taille " << taille << " existe. Adding this constraint." << endl;
                    for (int k : indices)
                    {
                        for (int l : indices)
                        {
                            tour += _x[k][l];
                        }
                    }
                    addCut(tour <= taille - 1);
                }
            }
        }
        catch (GRBException e)
        {
            cout << "Error number: " << e.getErrorCode() << endl;
            cout << e.getMessage() << endl;
        }
        catch (...)
        {
            cout << "Error during callback" << endl;
        }
    }
};

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

        if (!verbose)
        {
            model.set(GRB_IntParam_OutputFlag, 0);
        }

        model.getEnv().set(GRB_IntParam_PreCrush, 1);

        // --- Creation of the variables ---
        if (verbose)
            cout << "--> Creating the variables" << endl;

        x = new GRBVar *[n];

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
        if (verbose)
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
        if (verbose)
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

        // Callback
        Callback *cb = new Callback(x, n); // passing variable x to the solver callback
        model.setCallback(cb);             // adding the callback to the model

        // Optimize model
        // --- Solver configuration ---
        if (verbose)
            cout << "--> Configuring the solver" << endl;
        model.set(GRB_DoubleParam_TimeLimit, 600.0); //< sets the time limit (in seconds)
        model.set(GRB_IntParam_Threads, 1);          //< limits the solver to single thread usage

        // Callback
        Callback *cb = new Callback(x, n); // passing variable x to the solver callback
        model.setCallback(cb);             // adding the callback to the model

        //  --- Solver launch ---
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
                for (size_t j = 0; j < n; ++j)
                {
                    if ((int)x[i][j].get(GRB_DoubleAttr_X) == 1)
                    {
                        cout << "ville " << i << " --> "
                             << "ville " << j << endl;
                        i = j;
                        if (i == 0)
                            break;
                        j = -1;
                    }
                }

                cout << endl
                     << "representation brute:" << endl
                     << endl;

                for (size_t i = 0; i < n; ++i)
                {
                    for (size_t j = 0; j < n; ++j)
                    {
                        if (x[i][j].get(GRB_DoubleAttr_X) == 1.0)
                        {
                            cout << "ville " << i << " --> "
                                 << "ville " << j << endl;
                        }
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
        delete cb;
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