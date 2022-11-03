#include "BaseModel.h"

BaseModel::BaseModel(ModelData& gmd):md(&gmd) {
	cout << "initialize base model..." << endl;

	env = new GRBEnv();
	model = new GRBModel(*env);
}

BaseModel::~BaseModel() {
	cout << "release the base model..." << endl;
	
	delete model;
	delete env;
}

void BaseModel::addVariables() {
	//U
	cout << "add U" << endl;
	for (auto wt : md->type_set) {
		for (auto k : md->vehicle_set) {
			u_var[wt][k] = model->addVar(0, 1, 0, GRB_BINARY, "u_" + wt + "_" + itos(k));
		}
	}

	//X, PSI, Z, PI
	cout << "add X, PSI, Z, PI" << endl;
	for (auto i : md->task_set) {
		for (int r = 0; r <= md->R_amount; r++) {
			for (auto k : md->vehicle_set) {
				x_var[i][r][k] = model->addVar(0, 1, 0,	GRB_BINARY,
					"x_" + itos(i) + "_" + itos(r) + "_" + itos(k));
				psi_var[i][r][k] = model->addVar(0, 1, 0, GRB_BINARY,
					"psi_" + itos(i) + "_" + itos(r) + "_" + itos(k));

				z_var[i][r][k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS,
					"z_" + itos(i) + "_" + itos(r) + "_" + itos(k));
				pi_var[i][r][k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS,
					"pi_" + itos(i) + "_" + itos(r) + "_" + itos(k));
			}
		}
	}

	//Y
	cout << "add Y" << endl;
	for (auto i : md->task_set) {
		for (auto j : md->task_set) {
			for (int r = 0; r <= md->R_amount; r++) {
				for (auto k : md->vehicle_set) {
					y_var[i][j][r][k] = model->addVar(0, 1, 0, GRB_BINARY,
						"y_" + itos(i) + "_" + itos(j) + "_" + itos(r) + "_" + itos(k));
				}
			}
		}
	}

	//EPS
	cout << "add EPSI" << endl;
	for (int r = 0; r <= md->R_amount; r++) {
		for (auto k : md->vehicle_set) {
			epsi_var[r][k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS,
				"epsi_" + itos(r) + "_" + itos(k));
		}
	}

	//G
	cout << "add G" << endl;
	for (auto i : md->task_set) {
		for (int r = 0; r <= md->R_amount; r++) {
			g_var[i][r]=model->addVar(0, md->T_m, 0, GRB_CONTINUOUS,
				"epsi_" + itos(i) + "_" + itos(r));
		}
	}

	//TAU
	cout << "add TAU" << endl;
	for (auto k : md->vehicle_set) {
		tau_f_var[k] = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS,
			"tau_f_" + itos(k));
	}

	//T
	cout << "add T" << endl;
	T_var = model->addVar(0, md->T_m, 0, GRB_CONTINUOUS, "makespan");
}

void BaseModel::addConstraints() {
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
	for (auto i : md->task_set) {
		g_var[i][0].set(GRB_DoubleAttr_UB, 0);
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

		for (auto i : md->task_set) {
			if (r) {
				//model->addConstr(g_var[i][r] >= g_var[i][r - 1]);
				GRBLinExpr expr=0;
				for (auto k : md->vehicle_set) {
					expr += x_var[i][r-1][k];
				}

				model->addConstr(g_var[i][r] <= g_var[i][r - 1] + md->T_m * expr);
				model->addConstr(g_var[i][r] >= g_var[i][r - 1] - md->T_m * expr);
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
		if (r) model->addConstr(expr1 <= 1, "single occupied 1 " + itos(r));
		if (r < md->R_amount-1) model->addConstr(expr2 >= 0, "single occupied 2 " + itos(r));

		for (auto k : md->vehicle_set) {
			GRBLinExpr expr3 = 0, expr4=0;
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
				model->addConstr(expr == 0, "single vehicle sequence "
					+ itos(i) + "_" + itos(r) + "_" + itos(k) + "_");
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
						"compatibility 1 _" + itos(d[0]) + "_" + itos(d[1])
						+ "_" + itos(r) + "_" + itos(k));
				}
			}

			
		}
		model->addConstr(expr <= 1, "compatibility 2 " + itos(k));
	}

	//total time
	cout << "constraints: total time" << endl;
	for (auto k : md->vehicle_set) {
		GRBLinExpr expr = 0;
		for (int r = 1; r <= md->R_amount; r++) {
			for (auto i : md->task_set) {
				//if (i == md->depot) continue;
				expr += md->serving_time[i][k] * x_var[i][r - 1][k];
				for (auto j : md->task_set) {
					//if (j == md->depot) continue;
					expr += md->travel_cost[i][j] * y_var[i][j][r][k];
				}
			}
			if (r != md->R_amount) expr += epsi_var[r][k];
		}
		model->addConstr(expr == tau_f_var[k], "total_time_" + itos(k));
		model->addConstr(tau_f_var[k] <= T_var);
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
				"fulfill demand_" + itos(d[0]) + "_" + itos(d[1]));
		}
	}

	//one by one, avaliable time
	cout << "constraints: one by one, avaliable time" << endl;
	for (int r = 0; r < md->R_amount; r++) {
		cout << "r:" << r << endl;
		for (auto k : md->vehicle_set) {
			GRBLinExpr expr_left = 0;

			for (int gamma = 1; gamma <= r; gamma++) {
				for (auto i : md->task_set) {
					expr_left += md->serving_time[i][k] * x_var[i][gamma - 1][k];
					for (auto j : md->task_set) {
						expr_left += md->travel_cost[i][j] * y_var[i][j][gamma][k];
					}
				}
				expr_left += epsi_var[gamma][k];
			}

			GRBLinExpr expr_right_z = 0, expr_right_pi = 0,
				expr_left_sertime = 0, expr_right_x = 0;
			for (auto i : md->task_set) {
				expr_right_x += x_var[i][r][k];
				expr_right_z += z_var[i][r][k];
				expr_right_pi += pi_var[i][r][k];
				expr_left_sertime += md->serving_time[i][k] * x_var[i][r][k];
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
				expr += x_var[i][r][k];
			}

			model->addConstr(epsi_var[r][k] <= md->T_m * expr);
		}
	}

	//Y
	cout << "constraints: Y" << endl;
	for (auto k : md->vehicle_set) {
		for (int r = 1; r <= md->R_amount; r++) {
			for (auto i : md->task_set) {
				for (auto j : md->task_set) {
					model->addConstr(y_var[i][j][r][k] <= psi_var[i][r - 1][k],
						"yc1"+itos(i) + "_" + itos(j) + "_" + itos(r) + "_" + itos(k));
					model->addConstr(y_var[i][j][r][k] <= x_var[j][r][k],
						"yc2" + itos(i) + "_" + itos(j) + "_" + itos(r) + "_" + itos(k));
					model->addConstr(y_var[i][j][r][k] >=
						psi_var[i][r - 1][k] + x_var[j][r][k] - 1,
						"yc3" + itos(i) + "_" + itos(j) + "_" + itos(r) + "_" + itos(k));
				}
			}
		}
	}

	//Z,PI
	cout << "constraints: Z,PI" << endl;
	for (auto k : md->vehicle_set) {
		for (int r = 0; r < md->R_amount; r++) {
			for (auto i : md->task_set) {
				model->addConstr(z_var[i][r][k] <= md->T_m * x_var[i][r][k],
					"zc1_" + itos(i) + "_" + itos(r) + "_" + itos(k));
				model->addConstr(z_var[i][r][k] <= g_var[i][r],
					"zc2_" + itos(i) + "_" + itos(r) + "_" + itos(k));
				model->addConstr(z_var[i][r][k] >= 
					md->T_m * (x_var[i][r][k] - 1) + g_var[i][r],
					"zc3_" + itos(i) + "_" + itos(r) + "_" + itos(k));

				if (r == md->R_amount) continue;

				model->addConstr(pi_var[i][r][k] <= md->T_m * x_var[i][r][k],
					"pic1_" + itos(i) + "_" + itos(r) + "_" + itos(k));
				model->addConstr(pi_var[i][r][k] <= g_var[i][r + 1],
					"pic2_" + itos(i) + "_" + itos(r) + "_" + itos(k));
				model->addConstr(pi_var[i][r][k] >=
					md->T_m * (x_var[i][r][k] - 1) + g_var[i][r + 1],
					"pic3_" + itos(i) + "_" + itos(r) + "_" + itos(k));
			}
		}
	}
}

void BaseModel::setObjective() {
	GRBLinExpr expr = T_var;
	model->setObjective(expr, GRB_MINIMIZE);
}

void BaseModel::solveModel(double prec, double time_limit) {
	model->set(GRB_DoubleParam_MIPGap, prec);

	model->set(GRB_DoubleParam_TimeLimit, time_limit);

	model->optimize();
}

void BaseModel::saveSolution() {

	if (model->get(GRB_IntAttr_SolCount) == 0) return;

	/*
	for (auto w : u_var) {
		for (auto k : w.second) {
			md->u_val[w.first][k.first] = 
				k.second.get(GRB_DoubleAttr_X);
		}
	}
	*/

	for (auto i : x_var) {
		for (auto r : i.second) {
			for (auto k : r.second) {
				md->x_val[i.first][r.first][k.first] =
					k.second.get(GRB_DoubleAttr_X);

				/*
				md->psi_val[i.first][r.first][k.first]=
					psi_var[i.first][r.first][k.first].get(GRB_DoubleAttr_X);
				*/
			}
		}
	}

	md->saveMasterSolution(md->x_val);

	for (auto r : epsi_var) {
		for (auto k : r.second) {
			md->epsi_val[r.first][k.first] =
				k.second.get(GRB_DoubleAttr_X);
		}
	}

	for (auto i : g_var) {
		for (auto r : i.second) {
			md->g_val[i.first][r.first] =
				r.second.get(GRB_DoubleAttr_X);
		}
	}

	for (auto k : tau_f_var) {
		md->tau_finish[k.first] = k.second.get(GRB_DoubleAttr_X);
	}

	md->T_finish = T_var.get(GRB_DoubleAttr_X);
}

void BaseModel::setInitialSolution(unordered_map<int, unordered_map<int,
	unordered_map<int, double>>>& x_ini_val) {

	md->saveMasterSolution(x_ini_val);

	for (auto k : md->vehicle_set) {
		for (auto w : md->type_set)
			u_var[w][k].set(GRB_DoubleAttr_Start, md->u_val[w][k]);

		for (int r = 0; r <= md->R_amount; r++) {
			for (auto i : md->task_set) {
				x_var[i][r][k].set(GRB_DoubleAttr_Start, md->x_val[i][r][k]);
				psi_var[i][r][k].set(GRB_DoubleAttr_Start, md->psi_val[i][r][k]);

				for (auto j : md->task_set)
					y_var[i][j][r][k].set(GRB_DoubleAttr_Start, md->y_val[i][j][r][k]);
			}
		}
	}
}