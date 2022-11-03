#include "ModelData.h"

//using namespace std;

ModelData::ModelData(int param) {
	cout << "loading data..." << endl;

	switch (param)
	{
	case 0:
		tinyDemo();
		break;
	case 1:
		LiLimBenchmark();
		break;
	case 2:
		mineNetDB();
		break;
	default:
		break;
	}
}

ModelData::~ModelData() {
	cout << "unloading data..." << endl;
}

void ModelData::fetchSolveResult() {

}

void ModelData::Rgernalize(double mincap) {
	R_amount = 1;
	for (auto w : type_set) {
		for (auto d : demand_set[w]) {
			R_amount += 2 * ceil(demand_amount[w][d[0]][d[1]] / mincap);
		}
	}

	R_amount++;

	cout << "R_amount: " << R_amount << endl;
}

void ModelData::tinyDemo() {
	cout << "use a tiny demo" << endl;

	ifstream fin("tinydemo.txt", ios::in);

	if (!fin.is_open()) {
		cout << "cannot open file !" << endl;
		return;
	}
	string str;
	int len;

	//location
	fin >> str >> len;
	cout << str << len << endl;

	int id;
	double xv, yv;
	while (len--) {
		fin >> id >> xv >> yv;
		task_set.insert(id);
		xcoord[id] = xv;
		ycoord[id] = yv;
	}

	double mincap = 1e6, sumq = 0;
	depot = 0;

	//demand
	len = 0;
	fin >> str >> len;
	cout << str << len << endl;

	while (len--) {
		int sz,s,t;
		fin >> str >> sz;

		//cout << str << sz << endl;

		type_set.insert(str);
		while (sz--) {
			fin >> s >> t >> xv;
			demand_set[str].push_back({ s, t });
			demand_amount[str][s][t] = xv;
			sumq += xv;
		}
	}

	//vehicle
	fin >> str >> len;
	//cout << str <<len<< endl;

	while (len--) {

		//cout << "what ?" << endl;

		fin >> id >> xv >> yv;
		vehicle_set.insert(id);
		capacity[id] = xv;
		mincap = min(mincap, xv);
		for (auto i : task_set) {
			if (i == depot) continue;
			serving_time[i][id] = yv;
		}
	}

	Rgernalize(mincap);

	for (auto i : task_set) {
		for (auto j : task_set) {
			travel_cost[i][j] = sqrt(
				(xcoord[i] - xcoord[j]) * (xcoord[i] - xcoord[j])
				+ (ycoord[i] - ycoord[j]) * (ycoord[i] - ycoord[j])
			);

			//cout << "travel_cost[i][j]" << travel_cost[i][j] << endl;
		}
	}

	fin.close();
}

void ModelData::LiLimBenchmark() {
	cout << "use Li&Lim benchmark" << endl;
	cout << "intance:";
	string intance_name = "test10";
	cin >> intance_name;

	double mincap = 1e6, sumq = 0;
	depot = 0;

	ifstream fin("./pdptwdata/" + intance_name + ".txt", ios::in);

	if (!fin.is_open()) {
		cout << "fail in opening " << intance_name << endl;
		return;
	}

	int a, b, c, id;
	fin >> a >> b >> c;
	double demand, st, et, ser_time;
	string type_name;

	type_set.insert("TYPE-1");
	type_set.insert("TYPE-2");
	type_set.insert("TYPE-3");

	for (int k = 1; k <= max(a, int(type_set.size())); k++) {
		vehicle_set.insert(k);

		capacity[k] = k % 2 ? 7 : 8;

		mincap = min(mincap, capacity[k]);
	}


	int cnt = 0;
	while (!fin.eof()) {
		fin >> id;

		cnt = task_set.size();

		task_set.insert(id);

		if (task_set.size() == cnt) break;

		cout << "id" << id << endl;

		fin >> xcoord[id] >> ycoord[id];
		fin >> demand;
		fin >> st >> et;
		fin >> ser_time;

		//cout <<"sertime" << ser_time << endl;

		for (auto k : vehicle_set) {
			serving_time[id][k] = ser_time / 10;
		}

		fin >> a >> b;

		if (b % 3 == 0) type_name = "TYPE-1";
		if (b % 3 == 1) type_name = "TYPE-2";
		if (b % 3 == 2) type_name = "TYPE-3";

		if (b) {
			//cout << type_name << ' ' << id << ' ' << b << ' ' << demand << endl;
			demand_set[type_name].push_back({ id, b });
			demand_amount[type_name][id][b] = demand;
			sumq += demand;
		}

		//if (task_set.size() == 11) break;
	}

	Rgernalize(mincap);

	fin.close();

	for (auto i : task_set) {
		for (auto j : task_set) {
			travel_cost[i][j] = sqrt(
				(xcoord[i] - xcoord[j]) * (xcoord[i] - xcoord[j])
				+ (ycoord[i] - ycoord[j]) * (ycoord[i] - ycoord[j])
			);

			//cout << "travel_cost[i][j]" << travel_cost[i][j] << endl;
		}
	}

	cout << "type number" << type_set.size() << endl;
	cout << "vehicle number" << vehicle_set.size() << endl;
	cout << "task number" << task_set.size() << endl;
}

void ModelData::mineNetDB() {
	cout << "use a mine road network" << endl;
}

void ModelData::printDataDetails() {
	cout << "task number:" << task_set.size() << endl;
	cout << "vehicle number:" << vehicle_set.size() << endl;
	cout << "work type number:" << type_set.size() << endl;

	cout << "======U:" << endl;
	for (auto w : u_val) {
		for (auto k : w.second) {
			if (u_val[w.first][k.first] > 0.5)
				cout << w.first << "--vehicle" << k.first << endl;
		}
	}

	for (auto k : vehicle_set) {
		gantt["vehicle" + itos(k)].vehname = "vehicle" + itos(k);
		gantt["vehicle" + itos(k)].xcoord.push_back(0);
		gantt["vehicle" + itos(k)].ycoord.push_back(0);
	}

	solution_vec.push_back({ 0,0,0,0 });

	for (int r = 1; r < R_amount; r++) {
		solution_vec.push_back({});
		for (auto i : task_set) {
			for (auto k : vehicle_set) {
				if (x_val[i][r][k] > 0.5) {
					

					solution_vec[r] = { i,k };
					solution_vec[r].start_time = g_val[i][r + 1] - serving_time[i][k];

					if (epsi_val[r][k] > 0.1 && i) {
						gantt["vehicle" + itos(k)].xcoord.push_back(solution_vec[r].start_time - epsi_val[r][k]);
						gantt["vehicle" + itos(k)].ycoord.push_back(i);
					}

					gantt["vehicle" + itos(k)].xcoord.push_back(solution_vec[r].start_time);
					gantt["vehicle" + itos(k)].ycoord.push_back(i);

					solution_vec[r].waiting_time = epsi_val[r][k];
					cout << "\t" << r << ":\tvehicle-" << k << "\tstart task-" << i;
					cout << "\tat\t" << solution_vec[r].start_time;
					cout << "\tfor\t" << serving_time[i][k];

					gantt["vehicle" + itos(k)].xcoord.push_back(solution_vec[r].start_time + serving_time[i][k]);
					gantt["vehicle" + itos(k)].ycoord.push_back(i);

					if (epsi_val[r][k] > 0.01)
						cout << "\tafter waiting for\t" << epsi_val[r][k] << " ," << endl;
					else cout << "\twithout waiting ," << endl;
				}
			}
		}
	}

	

	cout << "======TAU:" << endl;
	for (auto k : tau_finish) {
		cout << "vehicle" << k.first << " 0"
			<< "->" << k.second << endl;
	}

	cout << "T_finish" << T_finish << endl;

	for (auto k : vehicle_set) {
		gantt["vehicle" + itos(k)].xcoord.push_back(tau_finish[k]);
		gantt["vehicle" + itos(k)].ycoord.push_back(0);

		plt::named_plot("vehicle" + itos(k),
			gantt["vehicle" + itos(k)].xcoord,
			gantt["vehicle" + itos(k)].ycoord);

	}

	plt::xlabel("time");

	plt::ylabel("machine");

	// Add graph title
	plt::title("Sample figure");
	// Enable legend.
	plt::legend();

	plt::grid(1);

	plt::show();

	//return;

	cout << "======X:" << endl;
	for (int r = 0; r <= R_amount; r++) {
		cout << "r" << r << '\t';
		for (auto k : vehicle_set) {
			cout << '\t';
			for (auto i : task_set) {
				if (x_val[i][r][k] > 0.5) {
					cout << i;
				}
			}
		}

		cout << "\tepsi";

		for (auto k : vehicle_set) {
			cout << '\t';
			for (auto i : task_set) {
				if (psi_val[i][r][k] > 0.5) {
					cout << i;
				}
			}
		}

		cout << endl;
	}

	cout << "======EPSI:" << endl;
	for (int r = 0; r <= R_amount; r++) {
		cout << r;
		for (auto k : vehicle_set) {
			cout << "\t" << k << "--" << epsi_val[r][k];
		}
		cout << endl;
	}

	cout << "======G:" << endl;
	for (int r = 0; r <= R_amount; r++) {
		cout << r;
		for (auto i : task_set) {
			cout << '\t';
			cout << i << "-" << g_val[i][r];
		}
		cout << endl;
	}
}

void ModelData::saveMasterSolution(unordered_map<int, unordered_map<int,
	unordered_map<int, double>>>& x_ini_val) {
	x_val = x_ini_val;

	for (int r = 0; r <= R_amount; r++) {
		for (auto k : vehicle_set) {
			double sum_x = 0;
			for (auto i : task_set) { sum_x += x_val[i][r][k]; }
			if (sum_x > 0.5) {
				for (auto i : task_set) { psi_val[i][r][k] = x_val[i][r][k]; }
			}
			else {
				for (auto i : task_set) { psi_val[i][r][k] = psi_val[i][r - 1][k]; }
			}

			if (r) {
				for (auto i : task_set) {
					for (auto j : task_set) {
						y_val[i][j][r][k] = psi_val[i][r - 1][k] * x_val[j][r][k];
					}
				}
			}
		}
	}

	for (auto dd : demand_set) {
		for (auto d : dd.second) {
			for (auto k : vehicle_set) {
				for (int r = 1; r < R_amount; r++) {
					if (y_val[d[0]][d[1]][r][k] > 0.5) {
						u_val[dd.first][k] = 1;
						break;
					}
				}
			}
		}
	}
}