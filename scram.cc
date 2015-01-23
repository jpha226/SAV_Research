#define MMDR_DYNA 1
#define MMDR_N5 2
#define MMDR_N4 3
#define MMD_MSD2 4

// options for simulation

#define REPETITIONS 2
//#define ALGORITHM MMDR_DYNA 
//#define ALGORITHM MMDR_N5
//#define ALGORITHM MMDR_N4
#define ALGORITHM MMD_MSD2
#define N 2000
#define MAXN N




#if ALGORITHM == MMDR_DYNA
#define SOLVER mmdr_dyna
#elif ALGORITHM == MMDR_N5
#define SOLVER mmdr_n5
#elif ALGORITHM == MMDR_N4
#define SOLVER mmdr_n4
#elif ALGORITHM == MMD_MSD2
#define SOLVER mmd_msd2
#endif // ALGORITHM


#include<iostream>
#include<fstream>
#include<cstdio>
#include<cmath>
#include<string>
#include<algorithm>
#include<vector>
#include<climits>
#include<float.h>
#include<string.h>

#include <boost/cstdint.hpp>

typedef uint64_t uint64;

// remove an element from a vector by value.
#define VECREMOVE(vec, v) (vec).erase(  \
              std::remove((vec).begin(), (vec).end(), (v)), (vec).end())


typedef std::pair<double, double> Point;

// Edge looks like (dist, (left node, right node))
typedef std::pair<double, std::pair<int, int> > Edge;

struct Test{
  std::vector<Point> starts;
  std::vector<Point> targets;
};

inline double getdist(const Point &a, const Point &b){
  return hypot(a.first-b.first, a.second-b.second);
}

inline bool edgeVectorLessThan(const std::vector<Edge> &a, const std::vector<Edge> &b) {
  const double ERROR_THRESH = .00005;
  for (int i = 0; i < a.size(); i++) {
    if (std::abs(a[i].first-b[i].first) < ERROR_THRESH) {
      continue;
    }
    return a[i] < b[i];
  }
  return false;
}

std::vector<Test> tests;

////////////  Begin implementation of getMinimalMaxEdgeInPerfectMatching  ////////////////

#if ALGORITHM == MMDR_N4 || ALGORITHM == MMD_MSD2

// We consider a bipartite graph with n nodes on the left and right.

// Global variables used in the floodfill.
// foo[0][i] corresponds to i'th node on left; foo[1][i] for the right.

std::vector<std::vector<int> > out[2];  //adjacency list for each node
std::vector<bool> visited[2];           //reached in floodfill
std::vector<int> back[2];               //way to reach it
std::vector<int> used;                  //whether a left node is used
//right nodes are used if and only if out[1][j].size() == 1.

// Floodfill from a node.
//  x in {0, 1}: left or right side
//  y in {0, ..., n-1}: node on side x
//  prev in {-1, 0, ..., n-1}: node on side 1-x that we came in on
//                             (-1 for unassigned node on left)
// Returns:
//  If it reaches an unassigned right node, the index of this node.
//  Otherwise, -1.
int flood(int x, int y, int prev){
  visited[x][y] = 1;
  back[x][y] = prev;
  if (x == 1 && out[x][y].size() == 0) //reached an unassigned right node!
    return y;

  for(int j = 0; j < out[x][y].size(); j++){
    if (!visited[1-x][out[x][y][j]]){
      int tmp = flood(1-x, out[x][y][j], y);
      if (tmp != -1) //Flood reached the end
        return tmp;
    }
  }
  return -1;
}

// starting at node (x, y), follow the back pointers and reverse each edge.
// Return the last node reached (i.e., the newly assigned left node)
inline int reverse(int x, int y){
  while (true) {
    int prev = back[x][y];
    if (prev == -1)       // Reached the unassigned node on the left
      break;
    out[x][y].push_back(prev);
    VECREMOVE(out[1-x][prev], y);
    x = 1-x; y = prev;
  }
  return y;
}

// Set visited to 0 and flood from unassigned left nodes.
inline void reset_flooding(int n){
  for(int i = 0; i < 2; i++)
    std::fill(visited[i].begin(), visited[i].end(), 0);

  for(int i = 0; i < n; i++)
    if(!used[i])
      flood(0, i, -1);
}

/*
  Add edges in order until k nodes can be matched.

  edges is a sorted vector of (dist, (left, right))

  Returns the index of the last edge added; this edge must appear.
 */
int getMinimalMaxEdgeInPerfectMatching(std::vector<Edge> edges, int n, int k){
  for(int i = 0; i < 2; i++) { //Clear the graph
    out[i].clear();
    out[i].resize(n);
  }
  std::fill(used.begin(), used.end(), 0);
  reset_flooding(n);

  int answer;
  for(answer = 0; answer < edges.size(); answer++){
    std::pair<int, int> e = edges[answer].second;
    out[0][e.first].push_back(e.second);
    //printf("Added edge: %d %d\n", e.first, e.second);
    if(visited[0][e.first] && !visited[1][e.second]) {
      int ans = flood(1, e.second, e.first);
      if (ans != -1){  //We made it to the end!
        if (--k == 0)
          break;
        int start = reverse(1, ans);
        used[start] = 1;
        reset_flooding(n);
      }
    }
  }
  // We must use edges[answer] to push k flow with minimal max edge.
  return answer;
}

#endif // ALGORITHM == MMDR_N4 || ALGORITHM == MMD_MSD2

////////////  End implementation of getMinimalMaxEdgeInPerfectMatching  ////////////////


////////////  Begin implementation of mmdr_dyna  ////////////////

#if ALGORITHM == MMDR_DYNA

std::vector<Edge> mmdr_dyna(Test t){
  int n = t.starts.size();
  std::vector<std::vector<Edge> > dists;
  std::vector<std::vector<Edge> > best;
  std::vector<int> prev;


  std::vector<std::vector<int> > sets;

  dists.resize(n);
  for(int i = 0; i < n; i++){
    dists[i].resize(n);
    for(int j = 0; j < n; j++) {
      dists[i][j] = std::make_pair(-getdist(t.starts[i], t.targets[j]),
				   std::make_pair(i, j));
    }
  }


  sets.resize(n+1);

  for(int i = 0; i < 1 << n; i++){
    int count = 0;
    for(int j = 0; j < n; j++)
      if (i & (1 << j))
        count++;
    sets[count].push_back(i);
  }

  best.resize(1<<n);
  prev.resize(1<<n);

  for(int k = 0; k < n; k++){
    for(int i = 0; i < n; i++){
      for(int j = 0; j < sets[k].size(); j++){
        if (sets[k][j] & (1 << i))
          continue;
        int s2 = sets[k][j] | (1 << i);
        std::vector<Edge> tmp = best[sets[k][j]];
        tmp.insert(std::lower_bound(tmp.begin(), tmp.end(), dists[i][k]),
                   dists[i][k]);
        if (best[s2].size() == 0 ||
            edgeVectorLessThan(best[s2], tmp))
          best[s2] = tmp;
      }
    }
  }
 
  std::vector<Edge> answer = best[sets[n][0]];

  return answer;
}

#endif // ALGORITHM == MMDR_DYNA

////////////  End implementation of mmdr_dyna  //////////////

////////////  Begin implementation of mmdr_n5  ////////////////

#if ALGORITHM == MMDR_N5

#define DATA_64WORDS_SIZE (MAXN*MAXN-1)/64+2

// -- Start functions for opeations on 64 bit values --
inline bool is_zero(const uint64 (&value)[DATA_64WORDS_SIZE]){
  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++)
    if (value[i] != 0) 
      return false;
  return true;
}

inline bool is_equal(const uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){
  if (valueA[DATA_64WORDS_SIZE-1] != valueB[DATA_64WORDS_SIZE-1]) {
    if (is_zero(valueA) && is_zero(valueB)) {
      return true;
      }
    return false;
  }

  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++) {
    if (valueA[i] != valueB[i])
      return false;
  }
  return true;
}

inline bool is_less_than(const uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){
  bool negA = valueA[DATA_64WORDS_SIZE-1];
  bool negB = valueB[DATA_64WORDS_SIZE-1];
  if (!negA && negB) {
    return false;
  } else if (negA && !negB) {
    return true;
  }

  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++) {
    if (valueA[i] == valueB[i])
      continue;
    return valueA[i] < valueB[i] ^ negA;
  }
  return false;
}

inline void set_max(uint64 (&value)[DATA_64WORDS_SIZE]){
  value[DATA_64WORDS_SIZE-1] = 0;
  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++) 
    value[i] = (uint64) -1;
}

inline void set_zero(uint64 (&value)[DATA_64WORDS_SIZE]){
  value[DATA_64WORDS_SIZE-1] = 0;

  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++) 
    value[i] = 0;
}

inline void set_bit(uint64 (&value)[DATA_64WORDS_SIZE], long bit){
  set_zero(value);
  uint64 val = ((uint64) 1) << std::abs(bit)%64;
  value[DATA_64WORDS_SIZE-1-std::abs(bit)/64-1] = val;
  if (bit < 0) {
    value[DATA_64WORDS_SIZE-1] = 1;
  }
}

inline void assign_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE])
{
  for (int i = 0; i < DATA_64WORDS_SIZE; i++) 
    valueA[i] = valueB[i];
}

void subtract_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]);

inline void add_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){ 
  //std::cout << "add_value\n";
  if (valueA[DATA_64WORDS_SIZE-1] != valueB[DATA_64WORDS_SIZE-1]) {
    uint64 temp[DATA_64WORDS_SIZE];
    assign_value(temp, valueB);
    temp[DATA_64WORDS_SIZE-1] = !temp[DATA_64WORDS_SIZE-1];
    subtract_value(valueA, temp);
    return;
  }

  int carry = 0;
  for (int i = DATA_64WORDS_SIZE-2; i >= 0; i--) { 
    valueA[i] += valueB[i] + carry;
    bool overflow = valueA[i] == valueB[i] && carry == 1;
    if (valueA[i] < valueB[i] || overflow)
      carry = 1;
    else
      carry = 0;
  }
}

inline void subtract_value(uint64 (&valueA)[DATA_64WORDS_SIZE], const uint64 (&valueB)[DATA_64WORDS_SIZE]){ 
  //std::cout << "subtract_value\n";
  if (is_equal(valueA, valueB)) {
    set_zero(valueA);
    return;
  }

  bool negA = valueA[DATA_64WORDS_SIZE-1];
  bool negB = valueB[DATA_64WORDS_SIZE-1];

  if (negA != negB) {
    uint64 temp[DATA_64WORDS_SIZE];
    assign_value(temp, valueB);
    temp[DATA_64WORDS_SIZE-1] = !temp[DATA_64WORDS_SIZE-1];
    add_value(valueA, temp);
    return;
  }

  if ((!negA && is_less_than(valueA, valueB)) ||
      (negA && is_less_than(valueB, valueA))) {
    uint64 temp[DATA_64WORDS_SIZE];
    assign_value(temp, valueB);
    subtract_value(temp, valueA);
    temp[DATA_64WORDS_SIZE-1] = !temp[DATA_64WORDS_SIZE-1];
    assign_value(valueA, temp);
    return;
  }
 

  uint64 negValueB[DATA_64WORDS_SIZE];
  for (int i = 0; i < DATA_64WORDS_SIZE-1; i++)
    negValueB[i] = ~valueB[i];
  negValueB[DATA_64WORDS_SIZE-1] = 0;
  uint64 one[DATA_64WORDS_SIZE];
  set_bit(one, 0);
  one[DATA_64WORDS_SIZE-1] = 0;
  add_value(negValueB, one);
  uint signA = valueA[DATA_64WORDS_SIZE-1];
  valueA[DATA_64WORDS_SIZE-1] = 0;
  add_value(valueA, negValueB);
  valueA[DATA_64WORDS_SIZE-1] = signA;
}
// -- End functions for operations on 64 bit values --

// Begin implementation of hungarian algorithm (large data) 

long newdist[MAXN][MAXN];

//Code for O(n^3) Hungarian algorithm from http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

//double cost[N][N];          //cost matrix
int n_large, max_match_large;        //n workers and n jobs
uint64 lx_large[N][DATA_64WORDS_SIZE], ly_large[N][DATA_64WORDS_SIZE];        //labels of X and Y parts
int xy_large[N];               //xy_large[x] - vertex that is matched with x,
int yx_large[N];               //yx_large[y] - vertex that is matched with y
bool S_large[N], T_large[N];         //sets S and T in algorithm
uint64 slack_large[N][DATA_64WORDS_SIZE];            //as in the algorithm description
int slackx_large[N];           //slackx[y] such a vertex, that
                         // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
int prev_large[N];             //array for memorizing alternating paths

void init_labels_large()
{
  memset(lx_large, 0, sizeof(lx_large));
  memset(ly_large, 0, sizeof(ly_large)); 
    for (int x = 0; x < n_large; x++)
      for (int y = 0; y < n_large; y++) {
	uint64 cost[DATA_64WORDS_SIZE];
	set_bit(cost, newdist[x][y]);
	  if (is_less_than(lx_large[x], cost)) {
	    assign_value(lx_large[x], cost);
	  }
	  //lx_large[x] = std::max(lx_large[x], newdist[x][y]);
      }
}

void update_labels_large()
{
  int x, y;
  uint64 delta[DATA_64WORDS_SIZE];
  set_max(delta);             //init delta as infinity
    for (y = 0; y < n_large; y++)            //calculate delta using slack
      if (!T_large[y]) {
	if (is_less_than(slack_large[y], delta)) {
	  assign_value(delta, slack_large[y]);
	  //delta = std::min(delta, slack_large[y]);
	}
      }
    for (x = 0; x < n_large; x++)            //update X labels
      if (S_large[x]) {
	subtract_value(lx_large[x], delta);
	//lx_large[x] -= delta;
      }
    for (y = 0; y < n_large; y++)            //update Y labels
      if (T_large[y]) {
	add_value(ly_large[y], delta);
	//ly_large[y] += delta;
      }
    for (y = 0; y < n_large; y++)            //update slack array
      if (!T_large[y]) {
	subtract_value(slack_large[y], delta);
	//slack_large[y] -= delta;
      }
}

void add_to_tree_large(int x, int prevx) 
//x - current vertex,prevx - vertex from X before x in the alternating path,
//so we add edges (prevx, xy_large[x]), (xy_large[x], x)
{
    S_large[x] = true;                    //add x to S
    prev_large[x] = prevx;                //we need this when augmenting
    for (int y = 0; y < n_large; y++) {    //update slacks, because we add new vertex to S
      uint64 temp[DATA_64WORDS_SIZE];
      assign_value(temp, lx_large[x]);
      add_value(temp, ly_large[y]);
      uint64 cost[DATA_64WORDS_SIZE];
      set_bit(cost,newdist[x][y]);
      subtract_value(temp, cost);
      if (is_less_than(temp, slack_large[y]))
      //if (lx_large[x] + ly_large[y] - newdist[x][y] < slack_large[y])
        {
	  assign_value(slack_large[y], temp);
	  //slack_large[y] = lx_large[x] + ly_large[y] - newdist[x][y];
	  slackx_large[y] = x;
        }
    }
}

bool augment_large()                         //main function of the algorithm
{
    if (max_match_large == n_large) return true;        //check wether matching is already perfect
    int x, y, root;                    //just counters and root vertex
    int q[N], wr = 0, rd = 0;          //q - queue for bfs, wr,rd - write and read
                                       //pos in queue
    memset(S_large, false, sizeof(S_large));       //init set S
    memset(T_large, false, sizeof(T_large));       //init set T
    memset(prev_large, -1, sizeof(prev_large));    //init set prev - for the alternating tree
    for (x = 0; x < n_large; x++)            //finding root of the tree
        if (xy_large[x] == -1)
        {
            q[wr++] = root = x;
            prev_large[x] = -2;
            S_large[x] = true;
            break;
        }

    for (y = 0; y < n_large; y++)            //initializing slack array
    {
      //slack_large[y] = lx_large[root] + ly_large[y] - newdist[root][y];
      assign_value(slack_large[y], lx_large[root]);
      add_value(slack_large[y], ly_large[y]);
      uint64 cost[DATA_64WORDS_SIZE];
      set_bit(cost, newdist[root][y]);
      subtract_value(slack_large[y], cost);
        slackx_large[y] = root;
    }

//second part of augment_large() function
    while (true)                                                        //main cycle
    {
        while (rd < wr)                                                 //building tree with bfs cycle
        {
            x = q[rd++];                                                //current vertex from X part
            for (y = 0; y < n_large; y++) {                                     //iterate through all edges in equality graph
	      uint64 temp[DATA_64WORDS_SIZE];
	      assign_value(temp, lx_large[x]);
	      add_value(temp, ly_large[y]);
	      uint64 cost[DATA_64WORDS_SIZE];
	      set_bit(cost, newdist[x][y]);
	      if (is_equal(cost, temp) && !T_large[y])
	      //if (newdist[x][y] == lx_large[x] + ly_large[y] &&  !T_large[y])
                {
                    if (yx_large[y] == -1) break;                             //an exposed vertex in Y found, so
                                                                        //augmenting path exists!
                    T_large[y] = true;                                        //else just add y to T,
                    q[wr++] = yx_large[y];                                    //add vertex yx_large[y], which is matched
                                                                        //with y, to the queue
                    add_to_tree_large(yx_large[y], x);                              //add edges (x,y) and (y,yx_large[y]) to the tree
                }
	    }
            if (y < n_large) break;                                           //augmenting path found!
        }
        if (y < n_large) break;                                               //augmenting path found!

        update_labels_large();                                                //augmenting path not found, so improve labeling
        wr = rd = 0;                
        for (y = 0; y < n_large; y++)        
        //in this cycle we add edges that were added to the equality graph as a
        //result of improving the labeling, we add edge (slackx[y], y) to the tree if
        //and only if !T[y] &&  slack[y] == 0, also with this edge we add another one
        //(y, yx_large[y]) or augment the matching, if y was exposed
	  if (!T_large[y] &&  is_zero(slack_large[y]))
            //if (!T_large[y] &&  slack_large[y] == 0)
            {
                if (yx_large[y] == -1)                                        //exposed vertex in Y found - augmenting path exists!
                {
                    x = slackx_large[y];
                    break;
                }
                else
                {
                    T_large[y] = true;                                        //else just add y to T,
                    if (!S_large[yx_large[y]])    
                    {
                        q[wr++] = yx_large[y];                                //add vertex yx_large[y], which is matched with
                                                                        //y, to the queue
                        add_to_tree_large(yx_large[y], slackx_large[y]);                  //and add edges (x,y) and (y,
                                                                        //yx_large[y]) to the tree
                    }
                }
            }
        if (y < n_large) break;                                               //augmenting path found!
    }

    if (y < n_large)                                                          //we found augmenting path!
    {
        max_match_large++;                                                    //increment matching
        //in this cycle we inverse edges along augmenting path
        for (int cx = x, cy = y, ty; cx != -2; cx = prev_large[cx], cy = ty)
        {
            ty = xy_large[cx];
            yx_large[cy] = cx;
            xy_large[cx] = cy;
        }
	return false;
        //augment_large();                                                      //recall function, go to step 1 of the algorithm
    }
    return true;
}//end of augment_large() function

void hungarian_large()
{
  //int ret = 0;                      //weight of the optimal matching
    n_large = N;
    max_match_large = 0;                    //number of vertices in current matching
    memset(xy_large, -1, sizeof(xy_large));    
    memset(yx_large, -1, sizeof(yx_large));
 
    init_labels_large();                    //step 0
    while (!augment_large()){;}                        //steps 1-3
    //for (int x = 0; x < n; x++)       //forming answer there
    //    ret += newdist[x][xy_large[x]];
    //return ret;
}

std::vector<Edge> mmdr_n5(Test t){
  int n = t.starts.size();
  std::vector<Edge> edges;
  std::vector<Edge> answer;
  
  // Create edges from all pairwise distances
  for(int i = 0; i < n; i++)
    for(int j = 0; j < n; j++)
      edges.push_back(std::make_pair(getdist(t.starts[i], t.targets[j]),
                                     std::make_pair(i, j)));
  std::sort(edges.begin(), edges.end());
 
  long lasti = -1;
  for(int i = 0; i < edges.size(); i++) {
    if (!i || edges[i].first != edges[lasti].first)
      lasti = i;
  
    newdist[edges[i].second.first][edges[i].second.second] = -lasti;
  }
  

  hungarian_large();

  for(int i = 0; i < n; i++){
    //printf("Got: %d %d %lf\n", i, h[i], getdist(t.starts[i], t.targets[h[i]]));
    answer.push_back(std::make_pair(getdist(t.starts[i], t.targets[xy_large[i]]),
                                    std::make_pair(i, xy_large[i])));
  }

  // answer now contains the correct answer.

  return answer;
}

#endif // ALGORITHM == MMDR_N5

////////////  End implementation of mmdr_n5  ////////////////


////////////  Begin implementation of mmdr_n4  ////////////////

#if ALGORITHM == MMDR_N4

// Begin implementation of hungarian with integer weights solution  ////////

// Code for O(n^3) Hungarian algorithm from http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

//#define N 55             //max number of vertices in one part
#define INF DBL_MAX    //just infinity

int cost_int[N][N];          //cost matrix

int n_int, max_match_int;        //n workers and n jobs

int lx_int[N], ly_int[N];        //labels of X and Y parts
int xy_int[N];               //xy[x] - vertex that is matched with x,
int yx_int[N];               //yx[y] - vertex that is matched with y
bool S_int[N], T_int[N];         //sets S and T in algorithm
int slack_int[N];            //as in the algorithm description
int slackx_int[N];           //slackx[y] such a vertex, that
                         // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
int prev_int[N];             //array for memorizing alternating paths

void init_labels_int()
{
    memset(lx_int, 0, sizeof(lx_int));
    memset(ly_int, 0, sizeof(ly_int));
    for (int x = 0; x < n_int; x++)
        for (int y = 0; y < n_int; y++)
	  lx_int[x] = std::max(lx_int[x], cost_int[x][y]);
}

void update_labels_int()
{
  int x, y;
  int delta = INT_MAX;             //init delta as infinity
    for (y = 0; y < n_int; y++)            //calculate delta using slack
        if (!T_int[y])
	  delta = std::min(delta, slack_int[y]);
    for (x = 0; x < n_int; x++)            //update X labels
        if (S_int[x]) lx_int[x] -= delta;
    for (y = 0; y < n_int; y++)            //update Y labels
        if (T_int[y]) ly_int[y] += delta;
    for (y = 0; y < n_int; y++)            //update slack_int array
        if (!T_int[y])
            slack_int[y] -= delta;
}

void add_to_tree_int(int x, int prevx) 
//x - current vertex,prevx - vertex from X before x in the alternating path,
//so we add edges (prevx, xy[x]), (xy[x], x)
{
    S_int[x] = true;                    //add x to S
    prev_int[x] = prevx;                //we need this when augmenting
    for (int y = 0; y < n_int; y++)    //update slacks, because we add new vertex to S
        if (lx_int[x] + ly_int[y] - cost_int[x][y] < slack_int[y])
        {
            slack_int[y] = lx_int[x] + ly_int[y] - cost_int[x][y];
            slackx_int[y] = x;
        }
}

bool augment_int()                         //main function of the algorithm
{
    if (max_match_int == n_int) return true;        //check wether matching is already perfect
    int x, y, root;                    //just counters and root vertex
    int q[N], wr = 0, rd = 0;          //q - queue for bfs, wr,rd - write and read
                                       //pos in queue
    memset(S_int, false, sizeof(S_int));       //init set S
    memset(T_int, false, sizeof(T_int));       //init set T
    memset(prev_int, -1, sizeof(prev_int));    //init set prev - for the alternating tree
    for (x = 0; x < n_int; x++)            //finding root of the tree
        if (xy_int[x] == -1)
        {
            q[wr++] = root = x;
            prev_int[x] = -2;
            S_int[x] = true;
            break;
        }

    for (y = 0; y < n_int; y++)            //initializing slack array
    {
        slack_int[y] = lx_int[root] + ly_int[y] - cost_int[root][y];
        slackx_int[y] = root;
    }

//second part of augment_int() function
    while (true)                                                        //main cycle
    {
        while (rd < wr)                                                 //building tree with bfs cycle
        {
            x = q[rd++];                                                //current vertex from X part
            for (y = 0; y < n_int; y++)                                     //iterate through all edges in equality graph
                if (cost_int[x][y] == lx_int[x] + ly_int[y] &&  !T_int[y])
                {
                    if (yx_int[y] == -1) break;                             //an exposed vertex in Y found, so
                                                                        //augmenting path exists!
                    T_int[y] = true;                                        //else just add y to T,
                    q[wr++] = yx_int[y];                                    //add vertex yx[y], which is matched
                                                                        //with y, to the queue
                    add_to_tree_int(yx_int[y], x);                              //add edges (x,y) and (y,yx[y]) to the tree
                }
            if (y < n_int) break;                                           //augmenting path found!
        }
        if (y < n_int) break;                                               //augmenting path found!

        update_labels_int();                                                //augmenting path not found, so improve labeling
        wr = rd = 0;                
        for (y = 0; y < n_int; y++)        
        //in this cycle we add edges that were added to the equality graph as a
        //result of improving the labeling, we add edge (slackx[y], y) to the tree if
        //and only if !T[y] &&  slack_int[y] == 0, also with this edge we add another one
        //(y, yx[y]) or augment the matching, if y was exposed
            if (!T_int[y] &&  slack_int[y] == 0)
            {
                if (yx_int[y] == -1)                                        //exposed vertex in Y found - augmenting path exists!
                {
                    x = slackx_int[y];
                    break;
                }
                else
                {
                    T_int[y] = true;                                        //else just add y to T,
                    if (!S_int[yx_int[y]])    
                    {
                        q[wr++] = yx_int[y];                                //add vertex yx[y], which is matched with
                                                                        //y, to the queue
                        add_to_tree_int(yx_int[y], slackx_int[y]);                  //and add edges (x,y) and (y,
                                                                        //yx[y]) to the tree
                    }
                }
            }
        if (y < n_int) break;                                               //augmenting path found!
    }

    if (y < n_int)                                                          //we found augmenting path!
    {
        max_match_int++;                                                    //increment matching
        //in this cycle we inverse edges along augmenting path
        for (int cx = x, cy = y, ty; cx != -2; cx = prev_int[cx], cy = ty)
        {
            ty = xy_int[cx];
            yx_int[cy] = cx;
            xy_int[cx] = cy;
        }
        //augment3();                                                      //recall function, go to step 1 of the algorithm
	return false;
    }
    return true;
}//end of augment3() function

int hungarian_int()
{
  int ret = 0;                      //weight of the optimal matching
    n_int = N;
    max_match_int = 0;                    //number of vertices in current matching
    memset(xy_int, -1, sizeof(xy_int));    
    memset(yx_int, -1, sizeof(yx_int));
 
    init_labels_int();                    //step 0
    while (!augment_int()){;}                        //steps 1-3
    for (int x = 0; x < n_int; x++)       //forming answer there
        ret += cost_int[x][xy_int[x]];
    return ret;
}

std::vector<Edge> mmdr_n4(Test t){
  int n = t.starts.size();
  std::vector<Edge> edges, edges_tmp;
  std::vector<Edge> answer;

  // Create edges from all pairwise distances
  for(int i = 0; i < n; i++)
    for(int j = 0; j < n; j++) {
      edges.push_back(std::make_pair(getdist(t.starts[i], t.targets[j]),
                                     std::make_pair(i, j)));
    }

  // Make the global variables in the floodfill have the right sizes
  for(int i = 0; i < 2; i++) {
    visited[i].resize(n);
    back[i].resize(n);
  }
  used.resize(n);

  int k = n;

  // Initialize all costs to large value
  // Don't use memset as memory might not be contiguous
  //memset(cost_int, -(n+1), sizeof(cost_int));
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N ; j++) {
      cost_int[i][j] = -(n+1);
    }
  }

  while (k > 0) {

    std::sort(edges.begin(), edges.end());

    // Call getMinimalMaxEdgeInPerfectMatching to get minimum maximal edge in a perfect matching.
    int choice = getMinimalMaxEdgeInPerfectMatching(edges, n, n);
    double max_edge_value = edges[choice].first;
    
    // Now remove (make very large) all edges that are greater than max_edge_value
    // Set cost of all max_edge_values to 1 and set cost of smaller edges to 0
    for(int i = 0; i < edges.size(); i++){
      if (edges[i].first < max_edge_value) {
	cost_int[edges[i].second.first][edges[i].second.second] = 0;
      } else if (edges[i].first == max_edge_value) {
	cost_int[edges[i].second.first][edges[i].second.second] = -1;
      } else {
	cost_int[edges[i].second.first][edges[i].second.second] = -(n+1);
      }
    }

    // Solve 0-1 sum minimization assignment problem
    int l = -hungarian_int();
    k -= l;

    if (k == 0) {
      // We are done!
      break;
    }

    // Now remove all edges that aren't tight edges and add max_edge_value 
    // edges to candidate edges for solution
    edges_tmp.clear();  
    for(int i = 0; i < edges.size(); i++) {
      Edge e = edges[i];
      if (lx_int[e.second.first] + ly_int[e.second.second] == cost_int[e.second.first][e.second.second]) {
	// We have a tight edge
	if (e.first == max_edge_value) {
	  // Set distance of edge to -1 so that we don't return it when finding the longest edge
	  // in a perfect matching and also so that we set its cost to 0 when minimizing 0-1 sum.
	  e.first = -1;  
	}
	edges_tmp.push_back(e);
      } else {
	// We are removing this edge from consideration
	cost_int[e.second.first][e.second.second] = -(n+1);
      }
    }
    edges = edges_tmp;
  }

  for (int i = 0; i < n; i++) {      //forming answer there
    answer.push_back(std::make_pair(getdist(t.starts[i], t.targets[xy_int[i]]),
                                    std::make_pair(i, xy_int[i])));
  }
  // answer now contains the correct answer.

  return answer;
}

#endif // ALGORTHM == MMDR_N4

////////////  End implementation of mmdr_n4  ////////////////

////////////  Begin implementation of mmd_msd2 solution  ////////

#if ALGORITHM == MMD_MSD2

// Code for O(n^3) Hungarian algorithm from http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

#define INF DBL_MAX    //just infinity

double cost[N][N];          //cost matrix
int n, max_match;        //n workers and n jobs
double lx[N], ly[N];        //labels of X and Y parts
int xy[N];               //xy[x] - vertex that is matched with x,
int yx[N];               //yx[y] - vertex that is matched with y
bool S[N], T[N];         //sets S and T in algorithm
double slack[N];            //as in the algorithm description
int slackx[N];           //slackx[y] such a vertex, that
                         // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
int prev[N];             //array for memorizing alternating paths

void init_labels()
{
    memset(lx, 0, sizeof(lx));
    memset(ly, 0, sizeof(ly));
    for (int x = 0; x < n; x++)
        for (int y = 0; y < n; y++)
	  lx[x] = std::max(lx[x], cost[x][y]);
}

void update_labels()
{
  int x, y;
  double delta = INF;             //init delta as infinity
    for (y = 0; y < n; y++)            //calculate delta using slack
        if (!T[y])
	  delta = std::min(delta, slack[y]);
    for (x = 0; x < n; x++)            //update X labels
        if (S[x]) lx[x] -= delta;
    for (y = 0; y < n; y++)            //update Y labels
        if (T[y]) ly[y] += delta;
    for (y = 0; y < n; y++)            //update slack array
        if (!T[y])
            slack[y] -= delta;
}

void add_to_tree(int x, int prevx) 
//x - current vertex,prevx - vertex from X before x in the alternating path,
//so we add edges (prevx, xy[x]), (xy[x], x)
{
    S[x] = true;                    //add x to S
    prev[x] = prevx;                //we need this when augmenting
    for (int y = 0; y < n; y++)    //update slacks, because we add new vertex to S
        if (lx[x] + ly[y] - cost[x][y] < slack[y])
        {
            slack[y] = lx[x] + ly[y] - cost[x][y];
            slackx[y] = x;
        }
}

bool augment()                         //main function of the algorithm
{
    if (max_match == n) return true;        //check wether matching is already perfect
    int x, y, root;                    //just counters and root vertex
    int q[N], wr = 0, rd = 0;          //q - queue for bfs, wr,rd - write and read
                                       //pos in queue
    memset(S, false, sizeof(S));       //init set S
    memset(T, false, sizeof(T));       //init set T
    memset(prev, -1, sizeof(prev));    //init set prev - for the alternating tree
    for (x = 0; x < n; x++)            //finding root of the tree
        if (xy[x] == -1)
        {
            q[wr++] = root = x;
            prev[x] = -2;
            S[x] = true;
            break;
        }

    for (y = 0; y < n; y++)            //initializing slack array
    {
        slack[y] = lx[root] + ly[y] - cost[root][y];
        slackx[y] = root;
    }

//second part of augment() function
    while (true)                                                        //main cycle
    {
        while (rd < wr)                                                 //building tree with bfs cycle
        {
            x = q[rd++];                                                //current vertex from X part
            for (y = 0; y < n; y++)                                     //iterate through all edges in equality graph
                if (cost[x][y] == lx[x] + ly[y] &&  !T[y])
                {
                    if (yx[y] == -1) break;                             //an exposed vertex in Y found, so
                                                                        //augmenting path exists!
                    T[y] = true;                                        //else just add y to T,
                    q[wr++] = yx[y];                                    //add vertex yx[y], which is matched
                                                                        //with y, to the queue
                    add_to_tree(yx[y], x);                              //add edges (x,y) and (y,yx[y]) to the tree
                }
            if (y < n) break;                                           //augmenting path found!
        }
        if (y < n) break;                                               //augmenting path found!

        update_labels();                                                //augmenting path not found, so improve labeling
        wr = rd = 0;                
        for (y = 0; y < n; y++)        
        //in this cycle we add edges that were added to the equality graph as a
        //result of improving the labeling, we add edge (slackx[y], y) to the tree if
        //and only if !T[y] &&  slack[y] == 0, also with this edge we add another one
        //(y, yx[y]) or augment the matching, if y was exposed
            if (!T[y] &&  slack[y] == 0)
            {
                if (yx[y] == -1)                                        //exposed vertex in Y found - augmenting path exists!
                {
                    x = slackx[y];
                    break;
                }
                else
                {
                    T[y] = true;                                        //else just add y to T,
                    if (!S[yx[y]])    
                    {
                        q[wr++] = yx[y];                                //add vertex yx[y], which is matched with
                                                                        //y, to the queue
                        add_to_tree(yx[y], slackx[y]);                  //and add edges (x,y) and (y,
                                                                        //yx[y]) to the tree
                    }
                }
            }
        if (y < n) break;                                               //augmenting path found!
    }

    if (y < n)                                                          //we found augmenting path!
    {
        max_match++;                                                    //increment matching
        //in this cycle we inverse edges along augmenting path
        for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty)
        {
            ty = xy[cx];
            yx[cy] = cx;
            xy[cx] = cy;
        }
        //augment();                                                      //recall function, go to step 1 of the algorithm
	return false;
    }
    return true;
}//end of augment() function

void hungarian()
{
  //int ret = 0;                      //weight of the optimal matching
    n = N;
    max_match = 0;                    //number of vertices in current matching
    memset(xy, -1, sizeof(xy));    
    memset(yx, -1, sizeof(yx));
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < N; j++) {
	cost[i][j] *= -1.0;
      }
    }
    init_labels();                    //step 0
    while (!augment()){;}                        //steps 1-3
    //for (int x = 0; x < n; x++)       //forming answer there
    //    ret += cost[x][xy[x]];
    //return ret;
}


std::vector<Edge> mmd_msd2(Test t){
  int n = t.starts.size();
  std::vector<Edge> edges;
  std::vector<Edge> answer;

  // Create edges from all pairwise distances squared
  for(int i = 0; i < n; i++)
    for(int j = 0; j < n; j++)
      edges.push_back(std::make_pair(pow(getdist(t.starts[i], t.targets[j]),2),
                                     std::make_pair(i, j)));
  std::sort(edges.begin(), edges.end());

  // Make the global variables in the floodfill have the right sizes
  for(int i = 0; i < 2; i++) {
    visited[i].resize(n);
    back[i].resize(n);
  }
  used.resize(n);

  // Call getMinimalMaxEdgeInPerfectMatching to get minimum maximal edge in a perfect mathcing.
  
  int choice = getMinimalMaxEdgeInPerfectMatching(edges, n, n);
  double max_edge_value = edges[choice].first;

  // Now remove (make very large) all edges that are greater than max_edge_value
  for(int i = 0; i < edges.size(); i++){
    if (edges[i].first <= max_edge_value) 
      cost[edges[i].second.first][edges[i].second.second] = edges[i].first;
    else
      cost[edges[i].second.first][edges[i].second.second] = max_edge_value*n+1;
  }

  hungarian();

  for(int i = 0; i < n; i++){
    //printf("Got: %d %d %lf\n", i, h2[i], getdist(t.starts[i], t.targets[h2[i]]));
    answer.push_back(std::make_pair(getdist(t.starts[i], t.targets[xy[i]]),
                                    std::make_pair(i, xy[i])));
  }

  // answer now contains the correct answer.

  return answer;
}

#endif // ALGORITHM == MMD_MSD2

////////////  End implementation of mmd_msd2  ////////////////


int main(){

  int repetitions = REPETITIONS;

  printf("Running %d repetitions...\n", repetitions);
  clock_t begin, end;
  begin = clock();
  srand(time(0));

  for(int iter = 0; iter < repetitions; iter++) {
    //std::cout << "Starting rep " << iter << "...\n";
    int n = N;
    Test t;
    for (int i = 0; i < n; i++) {
      t.starts.push_back(std::make_pair(rand()%(n*n), rand()%(n*n)));
    }
    for (int i = 0; i < n; i++) {
      t.targets.push_back(std::make_pair(rand()%(n*n), rand()%(n*n)));
    }
    
    SOLVER(t);
  }

  end = clock();
  double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
  printf("Done! %lf seconds (%lfms avg)\n", time_spent,
         time_spent * 1000 / repetitions);

  return 0;
}
