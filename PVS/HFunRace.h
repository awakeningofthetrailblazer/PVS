#pragma once

#include"headfiles.h"
#include"ModelData.h"

class HFunRace {
	ModelData* md = nullptr;
public:
	HFunRace(ModelData& gmd);
	~HFunRace();

	void iniSolutionGen(unordered_map<int, unordered_map<int,
		unordered_map<int,
		double>>>& x_ini_val,int para);//初始解生成算法

};