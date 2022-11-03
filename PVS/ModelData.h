#pragma once

#include"headfiles.h"

#include<fstream>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class ModelData
{
private:

	//data port
	void tinyDemo();
	void LiLimBenchmark();
	void mineNetDB();
	void Rgernalize(double mincap);

	//H-type solution store
	struct SOL {
		int task_id;
		int veh_id;
		double start_time;
		double waiting_time;
	};
	vector<SOL> solution_vec;

	struct SCHEMA {
		string vehname;
		vector<double> xcoord, ycoord;
	};
	map<string, SCHEMA> gantt;

	struct SCHEMA3D {
		string vehname;
		vector<double> xcoord, ycoord, zcoord;
	};
	map<string, SCHEMA3D> gantt3d;

public:
	/*====sets====*/
	unordered_set<int> vehicle_set;//K
	unordered_set<int> task_set;//V
	unordered_set<string> type_set;//W
	unordered_map<string, 
		vector<vector<int>>> demand_set;//D(w,i,j)

	int R_amount;//|R|=sum(q)/min(c)+1

	//location
	unordered_map<int, double> xcoord, ycoord;

	/*====parameters====*/
	int depot; //depot in task_set
	unordered_map<int, double> capacity;//|K|
	unordered_map<int, unordered_map<int, 
		double>> serving_time;//|V|*|K|
	unordered_map<int,unordered_map<int,
		double>> travel_cost;//|V|*|V|
	unordered_map<string, unordered_map<int,
		unordered_map<int, 
		double>>> demand_amount;//|W|*|V|*|V|
	double T_m = 1e6;//manual val

	/*====variables values====*/
	//binary
	unordered_map<string, unordered_map<int, 
		double>> u_val;//|W|*|K|
	unordered_map<int, unordered_map<int, 
		unordered_map<int, 
		double>>> x_val,psi_val;//|V|*|R|*|K|
	//continuous
	unordered_map<int, unordered_map<int, 
		double>> epsi_val;//|R|*|K|
	unordered_map<int, unordered_map<int, 
		double>> g_val;//|V|*|R|
	unordered_map<int, double> tau_finish;//|K|
	double T_finish;//makespan

	//other variables
	unordered_map<int, unordered_map<int,
		unordered_map<int,unordered_map<int,
		double>>>> y_val;//|V|*|V|*|R|*|K|
	unordered_map<int, unordered_map<int,
		unordered_map<int,
		double>>> z_val, pi_val;//|V|*|R|*|K|


	ModelData(int param);

	void printDataDetails();

	void fetchSolveResult();

	void saveMasterSolution(unordered_map<int, unordered_map<int,
		unordered_map<int, double>>>& x_ini_val);

	void saveSlaveSolution();

	~ModelData();
};

