#include <bits/stdc++.h>
#include <iomanip>
using namespace std;

struct Gear {
    int x, y;
    int ir, orr; // inner radius, outer radius
};

int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    const double EPS = 1e-9;
    int sim = 1;
    while (true) {
        int mx, my, mir, mor, av, ng;
        if(!(cin >> mx >> my >> mir >> mor >> av >> ng)) break;
        vector<Gear> gears(ng+1); // 0 is motor
        gears[0] = {mx, my, mir, mor};
        vector<int> av_input(ng+1, 0);
        av_input[0] = av; // motor angular velocity (positive clockwise, negative counter)
        for(int i=1;i<=ng;i++){
            int x,y,ir,orrr;cin>>x>>y>>ir>>orrr;
            gears[i] = {x,y,ir,orrr};
        }
        // Check overlapping error
        bool overlap = false;
        for(int i=0;i<=ng && !overlap;i++){
            for(int j=i+1;j<=ng && !overlap;j++){
                double dx = gears[i].x - gears[j].x;
                double dy = gears[i].y - gears[j].y;
                double dist = sqrt(dx*dx + dy*dy);
                if(dist < gears[i].ir + gears[j].ir - EPS || dist < gears[i].orr + gears[j].orr - EPS){
                    overlap = true;
                }
            }
        }
        cout << "Simulation #" << sim << "\n";
        if(overlap){
            cout << "Error -- Overlapping Gears\n\n";
            sim++;
            continue;
        }
        // Build adjacency list with touching edges
        struct Edge{int to; double ratio;}; // ratio = - (r_i_touch / r_j_touch)
        vector<vector<Edge>> adj(ng+1);
        for(int i=0;i<=ng;i++){
            for(int j=i+1;j<=ng;j++){
                double dx = gears[i].x - gears[j].x;
                double dy = gears[i].y - gears[j].y;
                double dist = sqrt(dx*dx + dy*dy);
                // check touch types, prioritize to avoid duplicate edges
                if (fabs(dist - (gears[i].ir + gears[j].ir)) < EPS) {
                    double ratio = - (double)gears[i].ir / (double)gears[j].ir;
                    adj[i].push_back({j, ratio});
                    adj[j].push_back({i, - (double)gears[j].ir / (double)gears[i].ir});
                } else if (fabs(dist - (gears[i].orr + gears[j].orr)) < EPS) {
                    double ratio = - (double)gears[i].orr / (double)gears[j].orr;
                    adj[i].push_back({j, ratio});
                    adj[j].push_back({i, - (double)gears[j].orr / (double)gears[i].orr});
                } else if (fabs(dist - (gears[i].ir + gears[j].orr)) < EPS) {
                    double ratio = - (double)gears[i].ir / (double)gears[j].orr;
                    adj[i].push_back({j, ratio});
                    adj[j].push_back({i, - (double)gears[j].orr / (double)gears[i].ir});
                } else if (fabs(dist - (gears[i].orr + gears[j].ir)) < EPS) {
                    double ratio = - (double)gears[i].orr / (double)gears[j].ir;
                    adj[i].push_back({j, ratio});
                    adj[j].push_back({i, - (double)gears[j].ir / (double)gears[i].orr});
                }
            }
        }
        // BFS to assign angular velocities
        vector<double> omega(ng+1, NAN);
        vector<bool> visited(ng+1,false);
        bool conflict = false;
        queue<int> q;
        omega[0] = (double)av_input[0];
        visited[0]=true;
        q.push(0);
        while(!q.empty() && !conflict){
            int u=q.front();q.pop();
            for(const auto &e: adj[u]){
                int v=e.to;
                double w = omega[u] * e.ratio; // because omega_v = omega_u * ratio
                if(!visited[v]){
                    omega[v]=w;
                    visited[v]=true;
                    q.push(v);
                } else {
                    if (fabs(omega[v] - w) > 1e-6) {
                        cerr << "Conflict detected between " << u << " and " << v << " w=" << w << " existing=" << omega[v] << "\n";
                        conflict = true; break;
                    }
                }
            }
        }
        if(conflict){
            cout << "Error -- Conflicting Gear Rotation\n\n";
            sim++;
            continue;
        }
        // Output gears 1..ng
        for(int i=1;i<=ng;i++){
            if(!visited[i]){
                // idle gear (not driven)
                cout << setw(2) << i << ": Warning -- Idle Gear\n";
            } else {
                double val = fabs(omega[i]);
                if (val < 1e-6) {
                    cout << setw(2) << i << ": Warning -- Idle Gear\n";
                } else {
                    char dir = (omega[i] > 0) ? 'R' : 'L';
                    cout << setw(2) << i << ": " << dir << ' ' << fixed << setprecision(2) << val << "\n";
                }
            }
        }
        cout << "\n";
        sim++;
    }
    return 0;
}

