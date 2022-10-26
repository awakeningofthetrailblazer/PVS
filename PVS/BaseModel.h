#pragma once

#include"headfiles.h"
#include"ModelData.h"
#include"gurobi_c++.h"

using namespace std;

class BaseModel
{
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

public:
	BaseModel(ModelData& gmd);

	~BaseModel();

	void addVariables();

	void addConstraints();

	void setObjective();

	void solveModel(double prec,double time_limit);

	void saveSolution();

	void setInitialSolution(unordered_map<int, 
		unordered_map<int,
		unordered_map<int, 
		double>>>& x_ini_val);

	
};

