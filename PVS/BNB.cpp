#include "BNB.h"

BNB::BNB(ModelData& gmd) :md(&gmd) {
	cout << "branch and bound environment ready ..." << endl;
}

BNB::~BNB() {
	cout << "branch and bound process over ..." << endl;
}

void BNB::typeAssignByDfs(int rk) {
	if (rk == the_vehicles.size()) {
		bool ft = true;
		double temp_bound = checkTheProject(the_chosen_type, ft);

		if (ft) {
			if (UB > temp_bound) UB = temp_bound;
			else {}
		}
		else {
			if (LB < temp_bound) LB = temp_bound;
		}
	}

}

double BNB::checkTheProject(const unordered_map<int,
	string>& u_root, bool& fit_type_constraints) {
	double res;

	return res;
}
