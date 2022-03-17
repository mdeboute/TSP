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

    GRBVar ***x = nullptr;
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

        x = new GRBVar **[n];

        for (size_t i = 0; i < n; ++i)
        {
            x[i] = new GRBVar *[n];
            for (size_t j = 0; j < n; ++j)
            {
                x[i][j] = new GRBVar[n];
                for (size_t k = 0; k < n; ++k)
                {
                    if (i != j && (i == 0 || k != 0) && (i != 0 || k == 0) && (j == 0 || k != n - 1) && (j != 0 || k == n - 1))
                    {
                        // cout << "x(" << i << "," << j << "," << k << ")" << endl;
                        stringstream ss;
                        ss << "x(" << i << "," << j << "," << k << ")";
                        x[i][j][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, ss.str());
                    }
                }
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
                for (size_t k = 0; k < n; ++k)
                {
                    if (i != j && (i == 0 || k != 0) && (i != 0 || k == 0) && (j == 0 || k != n - 1) && (j != 0 || k == n - 1))
                        obj += c[i][j] * x[i][j][k];
                }
            }
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // --- Creation of the constraints ---
        if (verbose)
            cout << "--> Creating the constraints" << endl;

        // Le sommet 0 est le seul pris en position 0 ****** maybe unnecessary
        GRBLinExpr arcDeb = 0;
        for (size_t j = 1; j < n; ++j)
        {
            arcDeb += x[0][j][0];
        }
        model.addConstr(arcDeb == 1);

        // Respect 1 flot � tout niveau k
        for (size_t k = 0; k < n; ++k)
        {
            GRBLinExpr flot = 0;
            for (size_t i = 0; i < n; ++i)
            {
                for (size_t j = 0; j < n; ++j)
                {
                    if (i != j && (i == 0 || k != 0) && (i != 0 || k == 0) && (j == 0 || k != n - 1) && (j != 0 || k == n - 1))
                    {
                        flot += x[i][j][k];
                    }
                }
            }
            stringstream ss;
            ss << "Flot(" << k << ")";
            model.addConstr(flot == 1, ss.str());
        }

        // Respect flot � tout noeud (j,k)
        for (size_t k = 1; k < n; ++k)
        {
            for (size_t j = 1; j < n; ++j)
            {
                GRBLinExpr flot1 = 0;
                GRBLinExpr flot2 = 0;
                for (size_t i = 0; i < n; ++i)
                {
                    if (i != j && (i == 0 || k - 1 != 0) && (i != 0 || k - 1 == 0) && (j == 0 || k - 1 != n - 1) && (j != 0 || k - 1 == n - 1))
                    {
                        // cout << "x1(" << i << "," << j << "," << k-1 << ")" << endl;

                        flot1 += x[i][j][k - 1];
                    }
                    if (j != i && (j == 0 || k != 0) && (j != 0 || k == 0) && (i == 0 || k != n - 1) && (i != 0 || k == n - 1))
                    {
                        // cout << "x2(" << j << "," << i << "," << k << ")" << endl;
                        flot2 += x[j][i][k];
                    }
                }
                stringstream ss;
                ss << "Flot(" << j << "," << k << ")";
                model.addConstr(flot1 == flot2, ss.str());
            }
        }

        // Respect flot pour chaque sommet j
        for (size_t j = 0; j < n; ++j)
        {
            GRBLinExpr flot1 = 0;
            GRBLinExpr flot2 = 0;
            for (size_t k = 0; k < n; ++k)
            {
                for (size_t i = 0; i < n; ++i)
                {
                    if (i != j && (i == 0 || k != 0) && (i != 0 || k == 0) && (j == 0 || k != n - 1) && (j != 0 || k == n - 1))
                    {
                        flot1 += x[i][j][k];
                    }
                    if (j != i && (j == 0 || k != 0) && (j != 0 || k == 0) && (i == 0 || k != n - 1) && (i != 0 || k == n - 1))
                    {
                        flot2 += x[j][i][k];
                    }
                }
            }
            stringstream ss;
            ss << "Flot1(" << j << ")";
            model.addConstr(flot1 == 1, ss.str());
            ss << "Flot2(" << j << ")";
            model.addConstr(flot2 == 1, ss.str());
        }

        // On retourne sur le sommet 0 en derni�re position
        GRBLinExpr arcSor = 0;
        for (size_t i = 1; i < n; ++i)
        {
            arcSor += x[i][0][n - 1];
        }
        model.addConstr(arcSor == 1);

        // Optimize model
        // --- Solver configuration ---
        if (verbose)
            cout << "--> Configuring the solver" << endl;
        model.set(GRB_DoubleParam_TimeLimit, 600.0); //< sets the time limit (in seconds)
        model.set(GRB_IntParam_Threads, 3);          //< limits the solver to single thread usage

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
                int k = 0;
                for (size_t j = 0; j < n; ++j)
                {
                    if (k == n)
                        break;
                    if (i != j && (i == 0 || k != 0) && (i != 0 || k == 0) && (j == 0 || k != n - 1) && (j != 0 || k == n - 1))
                    {
                        if ((int)x[i][j][k].get(GRB_DoubleAttr_X) == 1)
                        {
                            cout << "ville " << i << " --> "
                                 << "ville " << j << endl;
                            i = j;
                            if (i == 0)
                                break;
                            j = -1;
                            k++;
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

    for (size_t j = 0; j < n; ++j)
    {
        for (size_t i = 0; i < n; ++i)
            delete[] x[j][i];
        delete[] x[j];
    }
    delete[] x;

    return 0;
}