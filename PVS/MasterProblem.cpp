#include"MasterProblem.h"

MasterProblem::MasterProblem(ModelData& gmd):md(&gmd) {

	cout << "initialize master model..." << endl;

	env = new GRBEnv();
	model = new GRBModel(*env);

	addVariables();
	addConstraints();

	solveModel(0.2, 60);

	showResult();
}

MasterProblem::~MasterProblem() {
	cout << "release the master model..." << endl;

	delete model;
	delete env;
}

void MasterProblem::addVariables() {
	//U
	cout << "add U" << endl;
	for (auto w : md->type_set) {
		for (auto k : md->vehicle_set) {
			u_var[w][k] = model->addVar(0, 1, 0, GRB_BINARY);
		}
	}

	//P
	cout << "add P" << endl;
	for (auto dd : md->demand_set) {
		for (auto d : dd.second) {
			for (auto k : md->vehicle_set) {
				double cost = md->serving_time[d[0]][k]
					+ md->serving_time[d[1]][k]
					+ md->travel_cost[d[0]][d[1]];
				p_var[d[0]][d[1]][k] = model->addVar(0, md->R_amount, 0, GRB_INTEGER);
			}
		}
	}

	//T_l
	T_l = model->addVar(0, md->T_m, 1, GRB_CONTINUOUS);

	cout << "finish adding variables" << endl;
}

void MasterProblem::addConstraints() {
	//compatibility constraints
	for (auto k : md->vehicle_set) {
		GRBLinExpr expr = 0;
		for (auto w : md->type_set) {
			expr += u_var[w][k];
		}
		model->addConstr(expr <= 1);
	}

	for (auto dd : md->demand_set) {
		for (auto d : dd.second) {
			for (auto k : md->vehicle_set) {
				model->addConstr(p_var[d[0]][d[1]][k] <= md->R_amount * u_var[dd.first][k]);
			}
		}
	}

	//fulfill demand
	for (auto dd : md->demand_set) {
		for (auto d : dd.second) {
			GRBLinExpr expr = 0;
			for (auto k : md->vehicle_set) {
				expr += p_var[d[0]][d[1]][k] * md->capacity[k];
			}
			model->addConstr(expr >= md->demand_amount[dd.first][d[0]][d[1]]);
		}
	}

	//makespan
	cout << "add P" << endl;
	for (auto k : md->vehicle_set) {
		GRBLinExpr expr = 0;
		for (auto dd : md->demand_set) {
			for (auto d : dd.second) {
				double cost = 0;
				cost += md->travel_cost[d[0]][d[1]];
				cost += md->serving_time[d[0]][k];
				cost += md->serving_time[d[1]][k];
				expr += cost * p_var[d[0]][d[1]][k];
			}
		}
		model->addConstr(expr <= T_l);
	}

	cout << "finish adding constraints" << endl;
}

void MasterProblem::showResult() {
	if (model->get(GRB_IntAttr_SolCount) == 0) return;

	//U
	cout << "add U" << endl;
	for (auto w : md->type_set) {
		cout << w << ":";
		for (auto k : md->vehicle_set) {
			if (u_var[w][k].get(GRB_DoubleAttr_X) > 0.5)cout << "\t" << k;
		}
		cout << endl;
	}

	//P
	cout << "add P" << endl;
	for (auto k : md->vehicle_set) {
		cout << k << ":" << endl;
		for (auto dd : md->demand_set) {
			for (auto d : dd.second) {
				if (p_var[d[0]][d[1]][k].get(GRB_DoubleAttr_X) > 0.5)
					cout << "\t" << d[0] << "->" << d[1] << ": \t"
					<< p_var[d[0]][d[1]][k].get(GRB_DoubleAttr_X) << endl;
			}
		}
	}
}

void MasterProblem::solveModel(double prec, double time_limit) {
	model->set(GRB_DoubleParam_MIPGap, prec);

	model->set(GRB_DoubleParam_TimeLimit, time_limit);

	model->optimize();
}