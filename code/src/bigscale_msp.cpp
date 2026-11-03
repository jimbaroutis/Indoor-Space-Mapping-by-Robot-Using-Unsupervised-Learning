#include "functions.hpp"

vector<ll> dijkstra(int n, int start, const vector<vector<pair<int,double>>>& adj) {
    vector<ll> dist(n, INF);
    dist[start] = 0;
    priority_queue<pair<ll,int>, vector<pair<ll,int>>, greater<pair<ll,int>>> pq;
    pq.push({0, start});
    while(!pq.empty()) {
        auto p = pq.top(); pq.pop();
        ll d = p.first;
        int u = p.second;
        if(d > dist[u]) continue;
        for(auto &e : adj[u]) {
            int v = e.first;
            ll w = e.second;
            if(dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

vector<double> dijkstra_rmin(int n, int start, const vector<vector<pair<int,double>>>& adj) {

    vector<double> dist(n, -1e18);
    dist[start] = 1e18;

    priority_queue<pair<double,int>> pq; // max heap
    pq.push({dist[start], start});

    while(!pq.empty()) {
        auto p = pq.top();
        pq.pop();

        double curVal = p.first;
        int u = p.second;

        if(curVal < dist[u]) continue;

        for(auto &e : adj[u]) {
            int v = e.first;
            double w = e.second;

            double newVal = std::min(dist[u], w);

            if(newVal > dist[v]) {
                dist[v] = newVal;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

vector<tuple<int, int, double>> OE_min_subgraph(int nodes, int root, int goal, vector<vector<pair<int,double>>> adj_min, vector<tuple<int,int,double>> edges_min){

    vector<double> distFromRoot_min = dijkstra_rmin(nodes, root, adj_min);
    vector<double> distFromGoal_min = dijkstra_rmin(nodes, goal, adj_min);

    double c_min = distFromRoot_min[goal];

    vector<tuple<int, int, double>> OE_min;

    for(auto &te : edges_min){

        int u, v;
        double w;
        tie(u, v, w) = te;

        double bottleneck = min({distFromRoot_min[u], w, distFromGoal_min[v]});

        if(bottleneck == c_min){
            OE_min.emplace_back(u, v, w);
        }
    }
  return OE_min;
}

vector<tuple<int, int, double>> OE_subgraph(int nodes, int root, int goal, vector<vector<pair<int,double>>> adj_plus, vector<tuple<int,int,double>> edges){

    vector<ll> distFromRoot = dijkstra(nodes, root, adj_plus);
    vector<ll> distFromGoal = dijkstra(nodes, goal, adj_plus);

    ll c = distFromRoot[goal];

    vector<tuple<int, int, double>> OE;

    for(auto &te : edges){

        int u, v;
        double w;
        tie(u, v, w) = te;

        if(distFromRoot[u] + w + distFromGoal[v] == c){
            OE.emplace_back(u, v, w);
        }
    }
    return OE;
}

vector<tuple<int,int,double>> BFS(vector<tuple<int,int,double>> OE, int N, int root, int goal){

    vector<vector<pair<int,double>>> adj_OE(N);

    for (const auto& [u, v, w] : OE) {
        adj_OE[u].push_back({v, w});
        adj_OE[v].push_back({u, w});
    }

    vector<int> parent(N, -1);
    vector<double> parentWeight(N);

    vector<bool> visited(N, false);
    queue<int> q;

    q.push(root);
    visited[root] = true;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        if (u == goal)
            break;

        for (const auto& [v, w] : adj_OE[u]) {
            if (!visited[v]) {
                visited[v] = true;
                parent[v] = u;
                parentWeight[v] = w;
                q.push(v);
            }
        }
    }

    vector<tuple<int,int,double>> path;

    if (visited[goal]) {
        for (int v = goal; parent[v] != -1; v = parent[v]) {
            path.emplace_back(parent[v], v, parentWeight[v]);
        }

        reverse(path.begin(), path.end());
    }

    return path;
}

int main() {
    int m = 100;
    int n = 100;
    int N = m*n;  
    cout << "Using fixed grid dimensions: " << m << " x " << n << "\n";

    int numPoints;
    cout << "Enter number of points: ";
    cin >> numPoints;

    if(numPoints < 2)
    {
      cerr << "Need at least 2 points.\n";
      return 1;
    }

    vector<int> points;

    for(int i=0; i< numPoints; i++)
    {
        int r, c;
        cout <<"Point "<< i + 1 <<"(row, col): ";
        cin >> r >> c;
        if(r< 0 || r>=m || c< 0 || c>= n )
        {
        cerr << "Coordinates out of bounds. \n";
        return 1;
        }

        points.push_back(r * n + c);
    }

    vector<bool> blocked(N, false);
    int obs_count;
    cout << "Enter number of obstacle nodes: ";
    cin >> obs_count;
    if(obs_count < 0 || obs_count >= N - 2) {
       cerr << "Invalid obstacle count.\n";
       return 1;
    }

    std::mt19937_64 rng(std::random_device{}());
    std::uniform_int_distribution<int> nodeDist(0, N - 1);
    int created = 0;

    while(created < obs_count){
        int pos = nodeDist(rng);
        if(blocked[pos])
           continue;

        bool isWaypoint = false;
        for (int p: points)
        {
            if(pos == p)
            {
               isWaypoint = true;
                break;
            }
        }
        if(isWaypoint)
           continue;

        blocked[pos] = true;
        created++;
    }
    cout << obs_count << " random obstacles created.\n";

    vector<vector<pair<int,double>>> adj(N);
    vector<tuple<int,int,double>> edges; // (u,v,w)
    auto nodeId = [&](int i, int j){ return i * n + j; };

    for(int i = 0; i < m; i++) {
        for(int j = 0; j < n; j++) {
            int u = nodeId(i, j);
            if(j + 1 < n) {
                int v = nodeId(i, j+1);
                double w = 10;
                edges.emplace_back(u, v, w);
                adj[u].push_back({v, w});
                adj[v].push_back({u, w});
            }
            if(i + 1 < m) {
                int v = nodeId(i+1, j);
                double w = 10;
                edges.emplace_back(u, v, w);
                adj[u].push_back({v, w});
                adj[v].push_back({u, w});
            }
        }
    }

    vector<double> clearance(N, 1e18);
    // min-heap (distance, node)
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    // initialization,  directions
    vector<pair<int,int>> dirs = {
    {-1,0}, {1,0}, {0,-1}, {0,1}};
    
    for(int u = 0; u < N; u++) {
        int i = u / n;
        int j = u % n;
        // obstacles
        if(blocked[u]) {
            // obstacle itself
            if(clearance[u] > 0) {
                clearance[u] = 0;
                pq.push({0, u});
            }
            // neighbors of obstacle
            for(auto &d : dirs) {
                int ni = i + d.first;
                int nj = j + d.second;

                if(ni >= 0 && ni < m && nj >= 0 && nj < n) {
                    int v = nodeId(ni, nj);

                    if(clearance[v] > 0) {
                        clearance[v] = 0;
                        pq.push({0, v});
                    }
                }
            }
        }
        // boundries
        if(i == 0 || i == m-1 || j == 0 || j == n-1) {
            if(clearance[u] > 0) {
                clearance[u] = 0;
                pq.push({0, u});
            }
        }
    }
    // dijkstra
    while(!pq.empty()) {
        pair<double,int> top = pq.top();
        pq.pop();

        double dist = top.first;
        int u = top.second;

        if(dist > clearance[u]) continue;

        for(auto &edge : adj[u]) {
            int v = edge.first;
            double w = edge.second;
            if(blocked[v] ) continue;
            if(clearance[v] > clearance[u] + w) {
                clearance[v] = clearance[u] + w;
                pq.push({clearance[v], v});
            }
        }
    }
    // transform in 2D
    vector<vector<double>> finalWeight(m, vector<double>(n));

    for(int i = 0; i < m; i++) {
        for(int j = 0; j < n; j++) {
            int u = nodeId(i, j);
            finalWeight[i][j] = clearance[u];
        }
    }

    vector<vector<pair<int,double>>> adj_min(N);
    vector<tuple<int,int,double>> edges_min; // (u,v,w)
    for(int i = 0; i < m; i++) {
        for(int j = 0; j < n; j++) {

            int u = nodeId(i, j);
            // right
            if(j + 1 < n) {
                int v = nodeId(i, j+1);

                if(!blocked[u] && !blocked[v]) {
                    double w_uv = finalWeight[i][j+1]; // u -> v
                    double w_vu = finalWeight[i][j];

                    // edges list
                    edges_min.emplace_back(u, v, w_uv);
                    edges_min.emplace_back(v, u, w_vu);

                    // adjacency
                    adj_min[u].push_back({v, w_uv});
                    adj_min[v].push_back({u, w_vu});
                }
            }
            // down
            if(i + 1 < m) {
                int v = nodeId(i+1, j);

                if(!blocked[u] && !blocked[v]) {
                    double w_uv = finalWeight[i+1][j];
                    double w_vu = finalWeight[i][j];

                    edges_min.emplace_back(u, v, w_uv);
                    edges_min.emplace_back(v, u, w_vu);

                    adj_min[u].push_back({v, w_uv});
                    adj_min[v].push_back({u, w_vu});
                }
            }
        }
    }

    human_robot_motion(edges_min, adj_min, edges, adj, blocked, n, m, points);

    return 0;
}


