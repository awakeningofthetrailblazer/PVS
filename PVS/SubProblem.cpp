#include"SubProblem.h"

SubProblem::SubProblem(ModelData& gmd):md(&gmd) {

	cout << "initialize sub model..." << endl;

	env = new GRBEnv();
	model = new GRBModel(*env);
}

SubProblem::~SubProblem() {
	cout << "release the sub model..." << endl;

	delete model;
	delete env;
}

void SubProblem::addVariables() {
	//Z, PI
	for (auto i : md->task_set) {
		for (int r = 0; r <= md->R_amount; r++) {
			for (auto k : md->vehicle_set) {
				z_var[i][r][k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS);
				pi_var[i][r][k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS);
			}
		}
	}

	//EPS
	for (int r = 0; r <= md->R_amount; r++) {
		for (auto k : md->vehicle_set) {
			epsi_var[r][k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS);
		}
	}

	//G
	for (auto i : md->task_set) {
		for (int r = 0; r <= md->R_amount; r++) {
			g_var[i][r] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS);
		}
	}

	//TAU
	for (auto k : md->vehicle_set) {
		tau_f_var[k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS);
	}

	//T
	T_var = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS, "makespan");
}

void SubProblem::addConstraints() {
	//orgin and destination
	for (auto i : md->task_set) {
		g_var[i][0].set(GRB_DoubleAttr_UB, 0);
	}

	for (int r = 0; r <= md->R_amount; r++) {

		for (auto i : md->task_set) {
			if (r) {
				//model->addConstr(g_var[i][r] >= g_var[i][r - 1]);
				GRBLinExpr expr = 0;
				for (auto k : md->vehicle_set) {
					expr += md->x_val[i][r - 1][k];
				}

				model->addConstr(g_var[i][r] <= g_var[i][r - 1] + md->T_m * expr);
				model->addConstr(g_var[i][r] >= g_var[i][r - 1] - md->T_m * expr);
			}
		}
	}

	//total time
	for (auto k : md->vehicle_set) {
		GRBLinExpr expr = 0;
		for (int r = 1; r <= md->R_amount; r++) {
			for (auto i : md->task_set) {
				//if (i == md->depot) continue;
				expr += md->serving_time[i][k] * md->x_val[i][r - 1][k];
				for (auto j : md->task_set) {
					//if (j == md->depot) continue;
					expr += md->travel_cost[i][j] * md->y_val[i][j][r][k];
				}
			}
			if (r != md->R_amount) expr += epsi_var[r][k];
		}
		model->addConstr(expr == tau_f_var[k]);
		model->addConstr(tau_f_var[k] <= T_var);
	}

	//one by one, avaliable time
	for (int r = 0; r < md->R_amount; r++) {
		for (auto k : md->vehicle_set) {
			GRBLinExpr expr_left = 0;

			for (int gamma = 1; gamma <= r; gamma++) {
				for (auto i : md->task_set) {
					expr_left += md->serving_time[i][k] * md->x_val[i][gamma - 1][k];
					for (auto j : md->task_set) {
						expr_left += md->travel_cost[i][j] * md->y_val[i][j][gamma][k];
					}
				}
				expr_left += epsi_var[gamma][k];
			}

			GRBLinExpr expr_right_z = 0, expr_right_pi = 0,
				expr_left_sertime = 0, expr_right_x = 0;
			for (auto i : md->task_set) {
				expr_right_x += md->x_val[i][r][k];
				expr_right_z += z_var[i][r][k];
				expr_right_pi += pi_var[i][r][k];
				expr_left_sertime += md->serving_time[i][k] * md->x_val[i][r][k];
			}

			if (r) model->addConstr(expr_right_x <= 1);

			model->addConstr(expr_left >= expr_right_z
				+ md->T_m * expr_right_x - md->T_m);

			//if (r == md->R_amount) continue;
			if (!r) continue;

			model->addConstr(expr_left + expr_left_sertime >=
				expr_right_pi + md->T_m * expr_right_x - md->T_m);
			model->addConstr(expr_left + expr_left_sertime <=
				expr_right_pi + md->T_m - md->T_m * expr_right_x);
		}
	}

	for (auto k : md->vehicle_set) {
		for (int r = 1; r < md->R_amount; r++) {
			GRBLinExpr expr = 0;
			for (auto i : md->task_set) {
				expr += md->x_val[i][r][k];
			}

			model->addConstr(epsi_var[r][k] <= md->T_m * expr);
			model->addConstr(epsi_var[r][k] >= -md->T_m * expr);
		}
	}

	//Z,PI
	for (auto k : md->vehicle_set) {
		for (int r = 0; r < md->R_amount; r++) {
			for (auto i : md->task_set) {
				model->addConstr(z_var[i][r][k] <= md->T_m * md->x_val[i][r][k]);
				model->addConstr(z_var[i][r][k] <= g_var[i][r]);
				model->addConstr(z_var[i][r][k] >=
					md->T_m * (md->x_val[i][r][k] - 1) + g_var[i][r]);

				if (r == md->R_amount) continue;

				model->addConstr(pi_var[i][r][k] <= md->T_m * md->x_val[i][r][k]);
				model->addConstr(pi_var[i][r][k] <= g_var[i][r + 1]);
				model->addConstr(pi_var[i][r][k] >=
					md->T_m * (md->x_val[i][r][k] - 1) + g_var[i][r + 1]);
			}
		}
	}
}