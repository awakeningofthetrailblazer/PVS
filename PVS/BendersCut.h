#pragma once

#include"headfiles.h"
#include"ModelData.h"
#include"gurobi_c++.h"

class BendersCut: public GRBCallback {
public:
	BendersCut();
protected:
	void callback();
};