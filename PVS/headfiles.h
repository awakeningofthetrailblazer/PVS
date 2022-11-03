#pragma once

#include<iostream>
#include<unordered_set>
#include<vector>
#include<unordered_map>
#include<string>
#include<set>
#include<cmath>
#include<sstream>

using namespace std;

static string itos(int i) { stringstream s; s << i; return s.str(); }
static string dtos(double d) { stringstream s; s << d; return s.str(); }