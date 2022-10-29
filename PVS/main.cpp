#include"BaseModel.h"
#include"ModelData.h"
#include"HFunRace.h"
#include"MasterProblem.h"

int main() {

	ModelData gmd(1);

	HFunRace hfr(gmd);

	MasterProblem mstr(gmd);

	//return 0;

	BaseModel bm(gmd);

	bm.addVariables();

	if(0) {
		HFunRace hfr(gmd);

		auto x_ini = gmd.x_val;

		hfr.iniSolutionGen(x_ini, 0);

		bm.setInitialSolution(x_ini);
	}

	bm.addConstraints();

	bm.setObjective();

	bm.solveModel(0.01, 100);

	bm.saveSolution();

	gmd.printDataDetails();

	return 0;
}