#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <string>
#include <limits>
#include <chrono>

using namespace std;

// Uniform Cost Search function
void uniformCostSearch(int n, vector<vector<int>> &mat) {
    // Convert 2D matrix to 1D start state
    vector<int> start;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            start.push_back(mat[i][j]);  

    // Prepare goal state [1,2,...,n*n-1,0]
    vector<int> goal(n*n);
    for (int i = 0; i < n*n-1; ++i)
        goal[i] = i+1;
    goal[n*n-1] = 0;  // blank tile at end

    queue<vector<int>> q;                 // FIFO queue for UCS
    unordered_set<string> visited;        // to track visited states
    unordered_map<string, string> parent; // to reconstruct path

    // Function to encode vector<int> to string key
    auto encode = [](const vector<int> &v) {
        return string(v.begin(), v.end());  // simple byte-wise encoding
    };

    string startKey = encode(start);
    string goalKey  = encode(goal);

    q.push(start);
    visited.insert(startKey);
    parent[startKey] = "";  // root has no parent

    int nodesExpanded = 0, maxQueueSize = 1;
    vector<int> cur, goalState;
    bool found = false;

    // Main UCS loop
    while (!q.empty() && !found) {
        maxQueueSize = max(maxQueueSize, (int)q.size());  // track peak queue size
        cur = q.front(); q.pop();
        ++nodesExpanded;
        string curKey = encode(cur);

        // find position of blank (0)
        int blank = find(cur.begin(), cur.end(), 0) - cur.begin();
        int r = blank / n, c = blank % n;

        // check if goal reached
        if (curKey == goalKey) {
            goalState = cur;
            found = true;
            break;
        }

        // generate neighbours by moving blank up/down/left/right
        if (r > 0) {
            vector<int> next = cur;
            swap(next[blank], next[(r-1)*n + c]);
            string nextKey = encode(next);
            if (!visited.count(nextKey)) {
                visited.insert(nextKey);
                parent[nextKey] = curKey;
                q.push(next);
            }
        }
        if (r < n-1) {
            vector<int> next = cur;
            swap(next[blank], next[(r+1)*n + c]);
            string nextKey = encode(next);
            if (!visited.count(nextKey)) {
                visited.insert(nextKey);
                parent[nextKey] = curKey;
                q.push(next);
            }
        }
        if (c > 0) {
            vector<int> next = cur;
            swap(next[blank], next[r*n + (c-1)]);
            string nextKey = encode(next);
            if (!visited.count(nextKey)) {
                visited.insert(nextKey);
                parent[nextKey] = curKey;
                q.push(next);
            }
        }
        if (c < n-1) {
            vector<int> next = cur;
            swap(next[blank], next[r*n + (c+1)]);
            string nextKey = encode(next);
            if (!visited.count(nextKey)) {
                visited.insert(nextKey);
                parent[nextKey] = curKey;
                q.push(next);
            }
        }
    }

    // if no solution
    if (!found) {
        cout << "No solution found\n";
        cout << "Nodes expanded: " << nodesExpanded << "\n";
        cout << "Max queue size: " << maxQueueSize << "\n";
        return;
    }

    // reconstruct path from goal back to start
    vector<vector<int>> path;
    string key = encode(goalState);
    while (!key.empty()) {
        vector<int> state(key.size());
        for (int i = 0; i < (int)key.size(); ++i)
            state[i] = int(key[i]);
        path.push_back(state);
        key = parent[key];
    }

    // print solution path
    for (int i = path.size()-1; i >= 0; --i) {
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c)
                cout << path[i][r*n+c] << " ";
            cout << "\n";
        }
        cout << "\n";
    }
    cout << "Solution depth: " << path.size()-1 << "\n";
    cout << "Nodes expanded: " << nodesExpanded << "\n";
    cout << "Max queue size: " << maxQueueSize << "\n";
}

// A* Search with Misplaced Tile heuristic
void aStarMisplaced(int n, vector<vector<int>> &mat) {
    // prepare start and goal as before
    vector<int> start;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            start.push_back(mat[i][j]);

    vector<int> goal(n*n);
    for (int i = 0; i < n*n-1; ++i)
        goal[i] = i+1;
    goal[n*n-1] = 0;

    // encode state to string
    auto encode = [&](const vector<int> &v) {
        string s;
        for (int x : v) s.push_back(char(x));
        return s;
    };

    // heuristic: count of misplaced tiles (excluding blank)
    auto h = [&](const vector<int> &v) {
        int cnt = 0;
        for (int i = 0; i < n*n; ++i)
            if (v[i] != 0 && v[i] != goal[i])
                ++cnt;
        return cnt;
    };

    struct Node {
        vector<int> state;
        int f, g;  // f = g + h, g = cost so far
    };
    struct Cmp {
        bool operator()(const Node &a, const Node &b) const {
            return a.f > b.f;  // min-heap behaviour
        }
    };

    priority_queue<Node, vector<Node>, Cmp> open;  // open list
    unordered_set<string> closed;                  // closed list
    unordered_map<string,int> gCost;               // best g for each state
    unordered_map<string,string> parent;           // to rebuild path

    string startKey = encode(start), goalKey = encode(goal);
    open.push(Node{start, h(start), 0});
    gCost[startKey] = 0;
    parent[startKey] = "";

    int nodesExpanded = 0, maxOpen = 1;
    vector<int> cur, goalState;
    bool found = false;

    // main A* loop
    while (!open.empty() && !found) {
        maxOpen = max(maxOpen, (int)open.size());
        Node node = open.top(); open.pop();
        cur = node.state;
        string curKey = encode(cur);
        int g = node.g;

        if (closed.count(curKey)) continue;
        closed.insert(curKey);
        ++nodesExpanded;

        if (curKey == goalKey) {
            goalState = cur;
            found = true;
            break;
        }

        // find blank position
        int blank = find(cur.begin(), cur.end(), 0) - cur.begin();
        int r = blank / n, c = blank % n;

        // generate neighbours with cost = g+1
        auto tryMove = [&](int nr, int nc) {
            vector<int> next = cur;
            swap(next[blank], next[nr*n + nc]);
            string nextKey = encode(next);
            if (!closed.count(nextKey)) {
                int tg = g + 1;
                if (!gCost.count(nextKey) || tg < gCost[nextKey]) {
                    gCost[nextKey] = tg;
                    parent[nextKey] = curKey;
                    open.push(Node{next, tg + h(next), tg});
                }
            }
        };

        if (r > 0)        tryMove(r-1, c);
        if (r < n-1)      tryMove(r+1, c);
        if (c > 0)        tryMove(r, c-1);
        if (c < n-1)      tryMove(r, c+1);
    }

    if (!found) {
        cout << "No solution found\n";
        cout << "Nodes expanded: " << nodesExpanded << "\n";
        cout << "Max queue size: " << maxOpen << "\n";
        return;
    }

    // reconstruct and print path
    vector<vector<int>> path;
    string key = encode(goalState);
    while (!key.empty()) {
        vector<int> state(key.size());
        for (int i = 0; i < (int)key.size(); ++i)
            state[i] = int(key[i]);
        path.push_back(state);
        key = parent[key];
    }
    for (int i = path.size()-1; i >= 0; --i) {
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c)
                cout << path[i][r*n+c] << " ";
            cout << "\n";
        }
        cout << "\n";
    }
    cout << "Solution depth: " << path.size()-1 << "\n";
    cout << "Nodes expanded: " << nodesExpanded << "\n";
    cout << "Max queue size: " << maxOpen << "\n";
}

// A* Search with Manhattan Distance heuristic
void aStarManhattan(int n, vector<vector<int>> &mat) {
    // prepare start and goal
    vector<int> start;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            start.push_back(mat[i][j]);

    vector<int> goal(n*n);
    for (int i = 0; i < n*n-1; ++i)
        goal[i] = i+1;
    goal[n*n-1] = 0;

    // encode function
    auto encode = [&](const vector<int> &v) {
        string s;
        for (int x : v) s.push_back(char(x));
        return s;
    };

    // heuristic: sum of Manhattan distances for each tile
    auto h = [&](const vector<int> &v) {
        int dist = 0;
        for (int i = 0; i < n*n; ++i) {
            if (v[i] == 0) continue;  // skip blank
            int val = v[i] - 1;
            int gr = val / n, gc = val % n;
            int r = i / n, c = i % n;
            dist += abs(r - gr) + abs(c - gc);
        }
        return dist;
    };

    struct Node {
        vector<int> state;
        int f, g;
    };
    struct Cmp {
        bool operator()(const Node &a, const Node &b) const {
            return a.f > b.f;
        }
    };

    priority_queue<Node, vector<Node>, Cmp> open;
    unordered_set<string> closed;
    unordered_map<string,int> gCost;
    unordered_map<string,string> parent;

    string startKey = encode(start), goalKey = encode(goal);
    open.push(Node{start, h(start), 0});
    gCost[startKey] = 0;
    parent[startKey] = "";

    int nodesExpanded = 0, maxOpen = 1;
    vector<int> cur, goalState;
    bool found = false;

    // A* main loop
    while (!open.empty() && !found) {
        maxOpen = max(maxOpen, (int)open.size());
        Node node = open.top(); open.pop();
        cur = node.state;
        string curKey = encode(cur);
        int g = node.g;

        if (closed.count(curKey)) continue;
        closed.insert(curKey);
        ++nodesExpanded;

        if (curKey == goalKey) {
            goalState = cur;
            found = true;
            break;
        }

        int blank = find(cur.begin(), cur.end(), 0) - cur.begin();
        int r = blank / n, c = blank % n;

        auto tryMove = [&](int nr, int nc) {
            vector<int> next = cur;
            swap(next[blank], next[nr*n + nc]);
            string nextKey = encode(next);
            if (!closed.count(nextKey)) {
                int tg = g + 1;
                if (!gCost.count(nextKey) || tg < gCost[nextKey]) {
                    gCost[nextKey] = tg;
                    parent[nextKey] = curKey;
                    open.push(Node{next, tg + h(next), tg});
                }
            }
        };

        if (r > 0){
            tryMove(r-1, c);
        }
        if (r < n-1){
            tryMove(r+1, c);
        }
        if (c > 0){
            tryMove(r, c-1);
        }
        if (c < n-1){
            tryMove(r, c+1);
        }
    }

    if (!found) {
        cout << "No solution found\n";
        cout << "Nodes expanded: " << nodesExpanded << "\n";
        cout << "Max queue size: " << maxOpen << "\n";
        return;
    }

    // reconstruct and print solution
    vector<vector<int>> path;
    string key = encode(goalState);
    while (!key.empty()) {
        vector<int> state(key.size());
        for (int i = 0; i < (int)key.size(); ++i)
            state[i] = int(key[i]);
        path.push_back(state);
        key = parent[key];
    }
    for (int i = path.size()-1; i >= 0; --i) {
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c)
                cout << path[i][r*n+c] << " ";
            cout << "\n";
        }
        cout << "\n";
    }
    cout << "Solution depth: " << path.size()-1 << "\n";
    cout << "Nodes expanded: " << nodesExpanded << "\n";
    cout << "Max queue size: " << maxOpen << "\n";
}

int main(){
    bool repeat;
    int n;
    vector<vector<int>> mat;

    // input validation loop
    do {
        repeat = false;
        cout << endl;
        cout << "Welcome to the world of n puzzle , Please enter the value of n : " << endl;
        if (!(cin >> n)) {
            // non-integer entered
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "Invalid input: n must be a positive integer." << endl;
            repeat = true; 
            continue;
        }
        if (n <= 0) {
            cout << "Invalid input: n must be positive." << endl;
            repeat = true;
            continue;
        }
        mat.clear();
        mat.resize(n);
        vector<int> validation(n*n, 0);

        // read each row
        for (int i = 0; i < n; ++i) {
            cout << "Enter row " << i+1 << " :  ";
            for (int j = 0; j < n; ++j) {
                int temp;
                if (!(cin >> temp)) {
                    cin.clear();
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    cout << "Invalid input : value should be between 0 & " << n*n - 1 << endl;
                    repeat = true;
                    break;
                }
                if (temp < 0 || temp >= n*n) {
                    cout << "Invalid input : value should be between 0 & " << n*n - 1 << endl;
                    cin.clear();
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    repeat = true;
                    break;
                }
                if (validation[temp] != 0) {
                    cout << "Invalid input : All values should be unique" << endl << endl;
                    cin.clear();
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    repeat = true;
                    break;
                }
                validation[temp]++;
                mat[i].push_back(temp);  // add to matrix
            }
            if (repeat) break;
            cout << endl;
        }
    } while (repeat == true);  // Rerun the loop in case of invalid input

    // print the initial puzzle
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            cout << mat[i][j] << " ";
        }
        cout << endl;
    }

    cout << endl << endl;
    cout << "============================" << endl << endl;

    // choose algorithm
    int choice;
    cout << "Enter algorithm choice (1/2/3): " << endl;
    cout << "1 for Uniform Cost Search" << endl;
    cout << "2 for A* with the Misplaced Tile heuristic." << endl;
    cout << "3 for A* with the Manhattan Distance heuristic." << endl << endl;
    cin >> choice;

    auto startTime = chrono::high_resolution_clock::now();  // timer starts

    switch (choice) {
        case 1:
            uniformCostSearch(n, mat);
            break;
        case 2:
            aStarMisplaced(n, mat);
            break;
        case 3:
            aStarManhattan(n, mat);
            break;
        default:
            cout << "Invalid choice" << endl;
            break;
    }

    auto endTime = chrono::high_resolution_clock::now();  // timer ends
    auto elapsed = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
    cout << endl;
    cout << "Execution time: " << elapsed << " ms" << endl;

    return 0;
}
