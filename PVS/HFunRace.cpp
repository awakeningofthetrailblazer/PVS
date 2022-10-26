#include"HFunRace.h"

HFunRace::HFunRace(ModelData& gmd) {
	cout << "activate H function race..." << endl;
	md = &gmd;
}

HFunRace::~HFunRace() {
	cout << "deactivate H function race..." << endl;
}

void HFunRace::iniSolutionGen(unordered_map<int, unordered_map<int,
	unordered_map<int, double>>>& x_ini_val, int para = 0) {

	vector<double> demand_cut_rate;
	demand_cut_rate.push_back(0);

	auto demand_to_assign = md->demand_amount;

	double totdemand = 0;

	//save demand
	vector<string> type_list;
	for (auto w : md->demand_set) {
		type_list.push_back(w.first);
		for (auto d : w.second) {
			totdemand += demand_to_assign[w.first][d[0]][d[1]];
		}
		demand_cut_rate.push_back(totdemand);
	}
	
	//get cut rate
	for (int d = 0; d < demand_cut_rate.size(); d++) {
		demand_cut_rate[d] /= demand_cut_rate[demand_cut_rate.size() - 1];
		cout << "demand_cut_rate[d]" << demand_cut_rate[d] << endl;
	}

	//pair worktype
	int cnt = 0, cnt2 = 0;
	unordered_map<int, string> vehicle_worktype;
	for (auto k : md->vehicle_set) {
		if (double(cnt) / md->vehicle_set.size() < demand_cut_rate[cnt2+1]) {
			vehicle_worktype[k] = type_list[cnt2];
		}
		else {
			cnt2++;
			vehicle_worktype[k] = type_list[cnt2];
		}
		cnt++;

		if (cnt == md->vehicle_set.size()) {
			vehicle_worktype[k] = type_list[type_list.size() - 1];
		}

		cout << "veh " << k << " " << vehicle_worktype[k] << endl;
	}

	//assign tasks
	unordered_map<int,vector<int>> node_list;
	do {
		for (auto k : md->vehicle_set) {
			for (auto d : md->demand_set[vehicle_worktype[k]]) {
				if (demand_to_assign[vehicle_worktype[k]][d[0]][d[1]] <= 0) continue;
				if (md->capacity[k] <= demand_to_assign[vehicle_worktype[k]][d[0]][d[1]]) {
					demand_to_assign[vehicle_worktype[k]][d[0]][d[1]] -= md->capacity[k];
				}
				else {
					demand_to_assign[vehicle_worktype[k]][d[0]][d[1]] = 0;
				}
				node_list[k].push_back(d[0]);
				node_list[k].push_back(d[1]);

				//cout << k << ' ' << d[0] << ' ' << d[1] << endl;
				break;
			}
		}

		totdemand = 0;
		for (auto w : md->demand_set) {
			for (auto d : w.second) {
				totdemand += demand_to_assign[w.first][d[0]][d[1]];
			}
		}
	} while (totdemand>0);

	for (auto k : md->vehicle_set) {
		cout << k << ':';
		for (auto i : node_list[k]) cout << ' ' << i;
		cout << endl;
	}

	

	//solution generation
	for (auto i : md->task_set) {
		for (auto k : md->vehicle_set) {
			for (int r = 0; r <= md->R_amount; r++) {
				x_ini_val[i][r][k] = 0;
			}
		}
	}

	for (auto k : md->vehicle_set) {
		x_ini_val[md->depot][0][k] = 1;
		x_ini_val[md->depot][md->R_amount][k] = 1;
	}

	unordered_map<int, int> rkk;
	int rcnt = 1;

	if (para == 0) {
		while (true) {
			int ccnntt = 0;
			for (auto k : md->vehicle_set) {
				if (rkk[k] < node_list[k].size()) {
					x_ini_val[node_list[k][rkk[k]++]][rcnt++][k] = 1;
					ccnntt++;
				}
			}
			if (!ccnntt) break;
		}
	}
	else {
		for (auto k : md->vehicle_set) {
			for (auto nd : node_list[k]) {
				x_ini_val[nd][rcnt++][k] = 1;
			}
		}
	}

	md->x_val = x_ini_val;
}

