#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <queue>

#define fi first
#define se second

using namespace std;

typedef pair<int,int> ii;

const int MAXN = 22;
const int MAX_STATE_ID = 20203;

int N,M;
int INITIAL_STATE_ID, GOAL_STATE_ID;
int explored_num, max_depth;
int DIRS[4][2]={{0,-1},{-1,0},{0,1},{1,0}};
int cost[MAX_STATE_ID];
int last_move[MAX_STATE_ID];
int depth[MAX_STATE_ID];

bool used[MAX_STATE_ID];
bool explored[MAX_STATE_ID];

char ar[MAXN][MAXN],c;

queue<int> q;
vector<char> output;
priority_queue<ii> heap;

// Getting the input and creating the array.
// Also calculates the initial and goal state ids.
string init(char *argv1, char *argv2){

	freopen(argv1, "r", stdin);

	cin >> M >> N;
	string str;
	for(int i=0;i<=N;i++){
        getline(cin, str);
        for(int j=0;j<str.length();j++)
        	ar[i][j+1]=str.at(j); 
	}

	for(int i=1;i<=N;i++)
		for(int j=1;j<=M;j++)
			if(ar[i][j]=='s')
				INITIAL_STATE_ID=i*1000+j*10;
			else if(ar[i][j]=='g')
				GOAL_STATE_ID = i*1000+j*10;

	return argv2;
}

// Returns true if the given cell is available, false otherwise.
bool is_available(int x, int y){
	if(x<1 || x>N || y<1 || y>M)
		return false;
	return ar[x][y]!=' ';
}

// Is the state that I'll go valid?
// Caller of is_available
bool is_valid(int state){

	int x=state/1000;
	int y=(state/10)%100;
	if(!is_available(x, y)) return false;
	if(state%10==0) return true;
	if(state%10==1) return is_available(x+DIRS[2][0], y+DIRS[2][1]);
	return is_available(x+DIRS[1][0], y+DIRS[1][1]);
}

// Absolute value
int abs(int x){

	return x<0?-x:x;
}

// This function generates the next state id for a given state and direction.
int find_state_id(int state, int dir){

	int next_state = -1;
	if(state%10==0)
		switch(dir){
			case 0: next_state = state-19; break;
			case 1: next_state = state-998; break;
			case 2: next_state = state+11; break;
			case 3: next_state = state+2002; break;
		}
	else if(state%10==1) 
		switch(dir){
			case 0: next_state = state-11; break;
			case 1: next_state = state-1000; break;
			case 2: next_state = state+19; break;
			case 3: next_state = state+1000;break;
		}
	else
		switch(dir){
			case 0: next_state = state-10; break;
			case 1: next_state = state-2002; break;
			case 2: next_state = state+10; break;
			case 3: next_state = state+998; break;
		}
	return is_valid(next_state)?next_state:-1;
}

// I used this in order to find the solution path.
// This function finds the parent state for a given state and the last direction to come at that state.
int get_parent(int state, int dir){

	for(int i=1;i<=N;i++)
		for(int j=1;j<=M;j++)
			for(int k=0;k<=2;k++)
				if(find_state_id(i*1000+j*10+k, dir)==state)
					return i*1000+j*10+k;
}

// Calculates the cost of the move based on the project description.
int calculate_cost(int state, int dir){

	if(state%10==1 && (dir==0 || dir==2)) return 3;
	if(state%10==2 && (dir==1 || dir==3)) return 3;
	return 1;
}

// Converts direction id into direction letter.
char to_direction(int direction_id){

	switch(direction_id){
		case 0: return 'L';
		case 1: return 'U';
		case 2: return 'R';
		case 3: return 'D';
	}
}

// Manhattan distance + orientation check
int heuristic(int state){

	return abs((state/10)%100-(GOAL_STATE_ID/10)%100)+abs(state/1000-GOAL_STATE_ID/1000)+(state%10?1:0);
}

// Classical DFS algorithm.
// Goes wherever it can go, unless the state is visited before.
void dfs(int state, int depth, int cost, vector<int> moves){

	explored[state]=true;
	if(depth>max_depth) max_depth=depth;
	if(state==GOAL_STATE_ID){
		cout << cost << " " << explored_num << " " << max_depth << " " << moves.size() << endl;
		for(int i=0;i<moves.size();i++) cout << to_direction(moves[i]);
		return;
	}
	explored_num++;
	for(int i=0;i<4;i++){
		int next_state = find_state_id(state, i);
		if(next_state==-1) continue;
		if(explored[next_state]) continue;
		moves.push_back(i);
		dfs(next_state, depth+1, cost+calculate_cost(state, i), moves);
		moves.pop_back();
	}
}

// I push all new states to a queue.
// Finds the solution by picking the nodes one by one.
// Classical BFS algorithm, not optimal, but good.
void bfs(int state){

	explored[state]=true;
	q.push(state);
	while(!q.empty()){
		state = q.front();
		q.pop();
		explored_num++;
		for(int i=0;i<4;i++){
			int next_state = find_state_id(state, i);
			if(next_state==-1) continue;
			if(explored[next_state]) continue;
			explored[next_state]=true;
			depth[next_state]=depth[state]+1;
			cost[next_state]=cost[state]+calculate_cost(state, i);
			last_move[next_state]=i;
			q.push(next_state);
			if(next_state==GOAL_STATE_ID){
				state=next_state;
				cout << cost[state] << " " << explored_num << " " << depth[state] << " " << depth[state] << endl;
				while(state!=INITIAL_STATE_ID){
					output.push_back(to_direction(last_move[state]));
					state=get_parent(state, last_move[state]);
				}
				while(output.size()>0){
					cout << output[output.size()-1];
					output.pop_back();
				}
				return;
			}
		}
	}
}

// f()=g()
// I push all new states to a priority queue with their f() values.
// Finds the solution by picking the nodes with the smallest f() one by one.
// It seemed similar to Dijkstra's algorithm for me. Always finds the optimal solution.
void ucs(int state){

	explored[state]=true;
	heap.push(ii(0, state));
	while(!heap.empty()){
		state=heap.top().se;
		heap.pop();
		while(!heap.empty() && used[state]){
			state=heap.top().se;
			heap.pop();
		}
		if(heap.empty() && used[state]) return;
		used[state]=true;
		if(depth[state]>max_depth) max_depth=depth[state];
		explored_num++;
		for(int i=0;i<4;i++){
			int next_state = find_state_id(state, i);
			if(next_state==-1) continue;
			if(explored[next_state] && cost[next_state]<=cost[state]+calculate_cost(state,i)) continue;
			explored[next_state]=true;
			depth[next_state]=depth[state]+1;
			cost[next_state]=cost[state]+calculate_cost(state, i);
			last_move[next_state]=i;
			heap.push(ii(-cost[next_state], next_state));
			if(next_state==GOAL_STATE_ID){
				state=next_state;
				cout << cost[state] << " " << explored_num << " " << max(max_depth, depth[state]) << " " << depth[state] << endl;
				while(state!=INITIAL_STATE_ID){
					output.push_back(to_direction(last_move[state]));
					state=get_parent(state, last_move[state]);
				}
				while(output.size()>0){
					cout << output[output.size()-1];
					output.pop_back();
				}
				return;
			}
		}
	}
}

// f()=h'()
// I push all new states to a priority queue with their f() values.
// Finds the solution by picking the nodes with the smallest f() one by one.
// It always picks the node with the smallest heuristic cost.
void greedy(int state){

	explored[state]=true;
	heap.push(ii(heuristic(state), state));
	while(!heap.empty()){
		state=heap.top().se;
		heap.pop();
		if(depth[state]>max_depth) max_depth=depth[state];
		explored_num++;
		for(int i=0;i<4;i++){
			int next_state = find_state_id(state, i);
			if(next_state==-1) continue;
			if(explored[next_state]) continue;
			explored[next_state]=true;
			depth[next_state]=depth[state]+1;
			cost[next_state]=cost[state]+calculate_cost(state, i);
			last_move[next_state]=i;
			heap.push(ii(-heuristic(next_state), next_state));
			if(next_state==GOAL_STATE_ID){
				state=next_state;
				cout << cost[state] << " " << explored_num << " " << max(max_depth, depth[state]) << " " << depth[state] << endl;
				while(state!=INITIAL_STATE_ID){
					output.push_back(to_direction(last_move[state]));
					state=get_parent(state, last_move[state]);
				}
				while(output.size()>0){
					cout << output[output.size()-1];
					output.pop_back();
				}
				return;
			}
		}
	}
}

// f()=g()+h'()
// I push all new states to a priority queue with their f() values.
// Finds the solution by picking the nodes with the smallest f() one by one.
// Of course, if another(smaller) f() can be found for a state, it's been fixed and re-pushed.
void a_star(int state){

	explored[state]=true;
	heap.push(ii(-heuristic(state), state));
	while(!heap.empty()){
		state=heap.top().se;
		heap.pop();
		while(!heap.empty() && used[state]){
			state=heap.top().se;
			heap.pop();
		}
		if(heap.empty() && used[state]) return;
		used[state]=true;
		if(depth[state]>max_depth)
			max_depth=depth[state];
		explored_num++;
		for(int i=0;i<4;i++){
			int next_state = find_state_id(state, i);
			if(next_state==-1) continue;
			if(explored[next_state] && cost[next_state]<cost[state]+calculate_cost(state,i)) continue;
			explored[next_state]=true;
			depth[next_state]=depth[state]+1;
			cost[next_state]=cost[state]+calculate_cost(state, i);
			last_move[next_state]=i;
			heap.push(ii(-cost[next_state]-heuristic(next_state), next_state));
			if(next_state==GOAL_STATE_ID){
				state=next_state;
				cout << cost[state] << " " << explored_num << " " << max(max_depth, depth[state]) << " " << depth[state] << endl;
				while(state!=INITIAL_STATE_ID){
					output.push_back(to_direction(last_move[state]));
					state=get_parent(state, last_move[state]);
				}
				while(output.size()>0){
					cout << output[output.size()-1];
					output.pop_back();
				}
				return;
			}
		}
	}
}

int main(int argc, char **argv){

	string method = init(argv[1], argv[2]);
	vector<int> empty_vector;

	if(method=="dfs") dfs(INITIAL_STATE_ID, 0, 0, empty_vector);
	else if(method=="bfs") bfs(INITIAL_STATE_ID);
	else if(method=="ucs") ucs(INITIAL_STATE_ID);
	else if(method=="gs") greedy(INITIAL_STATE_ID);
	else if(method=="as") a_star(INITIAL_STATE_ID);
	else cout << "WRONG ARGUMENT: "+method << endl;

	return 0;
}