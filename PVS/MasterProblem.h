#pragma once

#include"headfiles.h"
#include"ModelData.h"
#include"gurobi_c++.h"
#include"SubProblem.h"
#include"BendersCut.h"

class MasterProblem {
private:
	ModelData* md;
	GRBModel* model = nullptr;
	GRBEnv* env = nullptr;

	//binary
	unordered_map<string, unordered_map<int,
		GRBVar>> u_var;//|W|*|K|
	unordered_map<int, unordered_map<int,
		unordered_map<int,
		GRBVar>>> p_var;//|V|*|V|*|K|
	GRBVar T_l;

	void addVariables();

	void addConstraints();

	void showResult();

public:
	MasterProblem(ModelData& gmd);
	~MasterProblem();

	void solveModel(double prec, double time_limit);
};