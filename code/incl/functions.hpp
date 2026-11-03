#define _USE_MATH_DEFINES
#include <bits/stdc++.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <cmath>
#include <random>
#include <stack>
using namespace std;
using ll = long long;
const ll INF = LLONG_MAX / 4;

struct State {
    std::pair<double,double> human_pos;

    bool wait = false;
    bool robot_moving = false;
    bool monitoring = false;
    bool terminate = false;
    bool manuevering = false;
    bool slow_down_mode = false;
    bool go_back = false;
    bool estimation_completed = false;
};

enum class HumanState {
    StandingCorridor,
    StandingNearDesk,
    Exercising,
    WaitingElevator,
    ReadingBoard,
    TalkingPhone,
    Tinkering,
    StandingKitchen,
    RestingLobby,
    CleaningShelves
};

struct StateInfo {
    HumanState state;
    int index;
};

//Functions for human and robot movement
int  human_robot_motion(vector<tuple<int,int,double>> edges_min, vector<vector<pair<int,double>>> adj_min, vector<tuple<int, int, double>> edges, vector<vector<pair<int,double>>> adj, vector<bool> blocked, int n, int m, vector<int> points);

vector<tuple<int,int,double>> BFS(vector<tuple<int,int,double>> OE, int N, int root, int goal);

vector<tuple<int, int, double>> OE_subgraph(int nodes, int root, int goal, vector<vector<pair<int,double>>> adj_plus, vector<tuple<int,int,double>> edges);

vector<tuple<int, int, double>> OE_min_subgraph(int nodes, int root, int goal, vector<vector<pair<int,double>>> adj_min, vector<tuple<int,int,double>> edges_min);
