#pragma once

#include"headfiles.h"
#include"ModelData.h"
#include"gurobi_c++.h"

class SubProblem {
private:
	ModelData* md;
	GRBModel* model = nullptr;
	GRBEnv* env = nullptr;

	//continuous
	unordered_map<int, unordered_map<int,
		GRBVar>> epsi_var;//|R|*|K|
	unordered_map<int, unordered_map<int,
		GRBVar>> g_var;//|V|*|R|
	unordered_map<int, GRBVar> tau_f_var;//|K|
	GRBVar T_var;//makespan

	unordered_map<int, unordered_map<int,
		unordered_map<int,
		GRBVar>>> z_var, pi_var;//|V|*|R|*|K|

	void addVariables();

	void addConstraints();

public:
	SubProblem(ModelData& gmd);
	~SubProblem();
};