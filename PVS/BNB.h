#pragma once
#include"headfiles.h"
#include"ModelData.h"

class BNB
{
private:
	unordered_map<string, bool> type_visited;//|W|

	unordered_map<int, string> the_chosen_type;//|K|->|W|

	vector<int> the_vehicles;//|K|

	unordered_map<string, unordered_map<int,
		int>> dynamic_u;//|W|*|K|

	ModelData* md;

	double LB = 0;
	double UB = INFINITY;
public:
	BNB(ModelData&gmd);

	void typeAssignByDfs(int rk);



	double checkTheProject(const unordered_map<int, 
		string>& u_root, bool& fit_type_constraints);

	~BNB();
};

