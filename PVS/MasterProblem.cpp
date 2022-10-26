#include"MasterProblem.h"

MasterProblem::MasterProblem(ModelData& gmd):md(&gmd) {

	cout << "initialize master model..." << endl;

	env = new GRBEnv();
	model = new GRBModel(*env);

	addVariables();
	addConstraints();

	model->optimize();
}

MasterProblem::~MasterProblem() {
	cout << "release the master model..." << endl;

	delete model;
	delete env;
}

void MasterProblem::addVariables() {
	//U

	cout << "add U" << endl;
	for (auto wt : md->type_set) {
		for (auto k : md->vehicle_set) {
			u_var[wt][k] = model->addVar(0, 1, 0, GRB_BINARY);
		}
	}

	//X, PSI
	cout << "add X,PSI" << endl;
	for (auto i : md->task_set) {
		for (int r = 0; r <= md->R_amount; r++) {
			for (auto k : md->vehicle_set) {
				x_var[i][r][k] = model->addVar(0, 1, 0, GRB_BINARY);
				psi_var[i][r][k] = model->addVar(0, 1, 0, GRB_BINARY);
			}
		}
	}

	//Y
	cout << "add Y" << endl;
	for (auto i : md->task_set) {
		for (auto j : md->task_set) {
			for (int r = 0; r <= md->R_amount; r++) {
				for (auto k : md->vehicle_set) {
					y_var[i][j][r][k] = model->addVar(0, 1, 0, GRB_BINARY);
				}
			}
		}
	}

	cout << "finish adding variables" << endl;
}

void MasterProblem::addConstraints() {
	//orgin and destination
	cout << "constraints: orgin and destination" << endl;
	for (auto k : md->vehicle_set) {
		x_var[md->depot][0][k].set(GRB_DoubleAttr_LB, 1);
		x_var[md->depot][md->R_amount][k].set(GRB_DoubleAttr_LB, 1);
		psi_var[md->depot][0][k].set(GRB_DoubleAttr_LB, 1);
		psi_var[md->depot][md->R_amount][k].set(GRB_DoubleAttr_LB, 1);

		for (auto i : md->task_set) {
			if (i == md->depot) continue;
			x_var[i][0][k].set(GRB_DoubleAttr_UB, 0);
			x_var[i][md->R_amount][k].set(GRB_DoubleAttr_UB, 0);
			psi_var[i][0][k].set(GRB_DoubleAttr_UB, 0);
			psi_var[i][md->R_amount][k].set(GRB_DoubleAttr_UB, 0);
		}
	}
	for (int r = 1; r < md->R_amount; r++) {
		for (auto k : md->vehicle_set) {
			x_var[md->depot][r][k].set(GRB_DoubleAttr_UB, 0);
		}
	}

	for (int r = 0; r <= md->R_amount; r++) {
		for (auto k : md->vehicle_set) {
			for (auto i : md->task_set) {
				y_var[i][i][r][k].set(GRB_DoubleAttr_UB, 0);
			}
		}
	}

	//single occupied
	cout << "constraints: single occupied" << endl;
	for (int r = 0; r < md->R_amount; r++) {
		GRBLinExpr expr1 = 0, expr2 = 0;

		for (auto i : md->task_set) {
			for (auto k : md->vehicle_set) {
				expr2 += x_var[i][r][k] - x_var[i][r + 1][k];
				expr1 += x_var[i][r][k];
			}
		}
		if (r) model->addConstr(expr1 <= 1, "single occupied 1 ");
		if (r < md->R_amount - 1) model->addConstr(expr2 >= 0, "single occupied 2 ");

		for (auto k : md->vehicle_set) {
			GRBLinExpr expr3 = 0, expr4 = 0;
			for (auto i : md->task_set) {
				expr3 += psi_var[i][r][k] - psi_var[i][r + 1][k];
				expr4 += psi_var[i][r][k];
			}
			//if (r < md->R_amount - 1) model->addConstr(expr3 >= 0);
			if (r) model->addConstr(expr4 == 1);

		}
	}

	//single vehicle sequence
	cout << "constraints: single vehicle sequence" << endl;
	for (int r = 1; r < md->R_amount; r++) {
		for (auto k : md->vehicle_set) {
			for (auto i : md->task_set) {
				GRBLinExpr expr = 0;
				for (auto j : md->task_set) {
					expr -= y_var[i][j][r][k];
				}
				expr += x_var[i][r][k] + psi_var[i][r - 1][k] - psi_var[i][r][k];
				model->addConstr(expr == 0, "single vehicle sequence ");
			}
		}
	}

	//compatibility constraints
	cout << "constraints: compatibility constraints" << endl;
	for (auto k : md->vehicle_set) {
		GRBLinExpr expr = 0;
		for (auto w : md->type_set) {
			expr += u_var[w][k];

			for (auto d : md->demand_set[w]) {
				for (int r = 1; r < md->R_amount; r++) {
					model->addConstr(y_var[d[0]][d[1]][r][k] <= u_var[w][k],
						"compatibility 1 _");
				}
			}


		}
		model->addConstr(expr <= 1, "compatibility 2 " );
	}

	//fulfill demand
	cout << "constraints: fulfill demand" << endl;
	for (auto dd : md->demand_set) {
		for (auto d : dd.second) {
			GRBLinExpr expr = 0;
			for (auto k : md->vehicle_set) {
				for (int r = 1; r < md->R_amount; r++) {
					expr += md->capacity[k] * y_var[d[0]][d[1]][r][k];
				}
			}
			model->addConstr(expr >= md->demand_amount[dd.first][d[0]][d[1]],
				"fulfill demand_");
		}
	}

	//Y
	cout << "constraints: Y ref" << endl;
	for (auto k : md->vehicle_set) {
		for (int r = 1; r <= md->R_amount; r++) {
			for (auto i : md->task_set) {
				for (auto j : md->task_set) {
					model->addConstr(y_var[i][j][r][k] <= psi_var[i][r - 1][k]);
					model->addConstr(y_var[i][j][r][k] <= x_var[j][r][k]);
					model->addConstr(y_var[i][j][r][k] >=
						psi_var[i][r - 1][k] + x_var[j][r][k] - 1);
				}
			}
		}
	}
}