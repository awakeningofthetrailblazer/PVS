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
		GRBVar>>> x_var, psi_var;//|V|*|R|*|K|

	unordered_map<int, unordered_map<int,
		unordered_map<int,
		unordered_map<int,
		GRBVar>>>> y_var;//|V|*|V|*|R|*|K|

	void addVariables();

	void addConstraints();

public:
	MasterProblem(ModelData& gmd);
	~MasterProblem();
};