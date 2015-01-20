#include "matching.h"
#include <vector>
#include <iostream>
#include<fstream>
#include<cstdio>
#include<cmath>
#include<string>
#include<algorithm>
#include<climits>
#include<float.h>
#include<string.h>

#include <boost/cstdint.hpp>


#define MMD_MSD2 4
#define ALGORITHM MMD_MSD2


inline double Matching::getdist(const Point &a, const Point &b){
  return hypot(a.second.first-b.second.first, a.second.second-b.second.second);
}

// remove an element from a vector by value.
#define VECREMOVE(vec, v) (vec).erase(  \
              std::remove((vec).begin(), (vec).end(), (v)), (vec).end())

Matching::Matching(){nTrips = 0; nCars = 0;}

Matching::~Matching()
{

//	for(int i=0; i<edge_set.size(); i++)
//		delete edge_set[i];

//	delete[] lx;
/*	delete[] ly;
	delete[] xy;
	delete[] yx;
	delete[] T;
	delete[] S;
	delete[] slack;
	delete[] slackx;
	delete[] prev;

	for(int i=0; i<N; i++)
		delete[] cost[i];
	delete[] cost;
*/
}

void Matching::setDimensions(int nt, int nc)
{
	nTrips = nt; 
	nCars = nc;
}

/*void Matching::createEdge(int trp, int c, int dist)
{
	edge* e = new edge;
	e->car = c;
	e->trip = trp;
	e->distance = dist;
	edge_set.push_back(e);
}*/

void Matching::addCar(int x, int y, int id){t.starts.push_back(std::make_pair(id,std::make_pair(x, y)));}
void Matching::addTrip(int x, int y, int id){t.targets.push_back(std::make_pair(id,std::make_pair(x, y)));}

// k must be less than or equal to smaller of number of trips or cars
double* getKthSmallest(int k, std::vector<Point> list)
{
	double *result = new double[k];
	int *minIndex = new double[k];
	double *minValue = new double[k];	/*
	double dist;
	Point temp;
	// get Kth smallest ran k times
	for (int i=0; i<k; i++){

		for (int j=0; j<k; j++){
			minIndex[i] = j;
			minValue[i] = // distance from targets[i] to list[j]
			for (int l=j+1; l<list.size(); l++){
				// compute distance for list[l]
				if (list[l] < minValue[i]) {
					minIndex[i] = l;
					minValue[i] = // distance between targets[i] and list[l];
				}
			}
			// Swap in list
			temp = list[minIndex[i]];
			list[minIndex[i]] = list[j];
			list[j] = temp;
		}
		
	}*/
	delete[] minIndex;
	delete[] result;

	return minValue;
}

std::vector<Edge> Matching::findMatching()
{
	int n = t.starts.size();
	int m = t.targets.size();
	nCars = n;
	nTrips = m;
	int max[n];	
	double *kth;
	double dist;
	std::vector<Point> temp;

	if (n < m){

		// filter number of trips
		//kth = getKthSmallest(nCars, targets);

		for (int i=nCars; i<nCars + m-n; i++)
			t.starts.push_back(std::make_pair(i,std::make_pair(0,0)));

	} else {

		// filter number of cars
/*		kth = getKthSmallest(nTrips, starts);
		temp = starts;
		starts.clear();
		for (int i=0; i<nTrips; i++)
		{
			for (int j=0; j<nCars; j++)
			{
				dist = // distance between jth start and ith target
				if (dist <= kth[i])
					starts.push_back(temp[j]);
			}	
		}		

		n = starts.size();
		nCars = n;*/

		for (int i=nTrips; i<nTrips + n - m; i++)
			t.targets.push_back(std::make_pair(i,std::make_pair(0,0)));

	}

	//std::cout<<"num cars: "<<nCars<<std::endl;
	//std::cout <<"num trips: "<<nTrips<<std::endl;
	N = t.starts.size();
	return mmd_msd2(t);

} 


////////////  Begin implementation of getMinimalMaxEdgeInPerfectMatching  ////////////////

#if ALGORITHM == MMDR_N4 || ALGORITHM == MMD_MSD2

// We consider a bipartite graph with n nodes on the left and right.

// Global variables used in the floodfill.
// foo[0][i] corresponds to i'th node on left; foo[1][i] for the right.

//std::vector<std::vector<int> > out[2];  //adjacency list for each node
//std::vector<bool> visited[2];           //reached in floodfill
//std::vector<int> back[2];               //way to reach it
//std::vector<int> used;                  //whether a left node is used
//right nodes are used if and only if out[1][j].size() == 1.

// Floodfill from a node.
//  x in {0, 1}: left or right side
//  y in {0, ..., n-1}: node on side x
//  prev in {-1, 0, ..., n-1}: node on side 1-x that we came in on
//                             (-1 for unassigned node on left)
// Returns:
//  If it reaches an unassigned right node, the index of this node.
//  Otherwise, -1.
int Matching::flood(int x, int y, int prev){
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
inline int Matching::reverse(int x, int y){
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
inline void Matching::reset_flooding(int n){
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
int Matching::getMinimalMaxEdgeInPerfectMatching(std::vector<Edge> edges, int n, int k){
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

////////////  Begin implementation of mmd_msd2 solution  ////////

#if ALGORITHM == MMD_MSD2

// Code for O(n^3) Hungarian algorithm from http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm

#define INF DBL_MAX    //just infinity

//double cost[N][N];          //cost matrix
//int n, max_match;        //n workers and n jobs
//double lx[N], ly[N];        //labels of X and Y parts
//int xy[N];               //xy[x] - vertex that is matched with x,
//int yx[N];               //yx[y] - vertex that is matched with y
//bool S[N], T[N];         //sets S and T in algorithm
//double slack[N];            //as in the algorithm description
//int slackx[N];           //slackx[y] such a vertex, that
                         // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
//int prev[N];             //array for memorizing alternating paths

void Matching::init_labels()
{
    int n = N;
    memset(lx, 0, n*sizeof(*lx));
    memset(ly, 0, n*sizeof(*ly));
    for (int x = 0; x < n; x++)
        for (int y = 0; y < n; y++)
	  lx[x] = std::max(lx[x], cost[x][y]);
}

void Matching::update_labels()
{
  int x, y, n=N;
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

void Matching::add_to_tree(int x, int prevx) 
//x - current vertex,prevx - vertex from X before x in the alternating path,
//so we add edges (prevx, xy[x]), (xy[x], x)
{
    int n = N;
    S[x] = true;                    //add x to S
    prev[x] = prevx;                //we need this when augmenting
    for (int y = 0; y < n; y++)    //update slacks, because we add new vertex to S
        if (lx[x] + ly[y] - cost[x][y] < slack[y])
        {
            slack[y] = lx[x] + ly[y] - cost[x][y];
            slackx[y] = x;
        }
}

bool Matching::augment()                         //main function of the algorithm
{

    int n = N;
  //std::cout<<"n is "<<n<<std::endl;
    if (max_match == n) return true;        //check wether matching is already perfect
    int x, y, root;                    //just counters and root vertex
    int q[N], wr = 0, rd = 0;          //q - queue for bfs, wr,rd - write and read
                                       //pos in queue
    memset(S, false, n*sizeof(*S));       //init set S
    memset(T, false, n*sizeof(*T));       //init set T
    memset(prev, -1, n*sizeof(*prev));    //init set prev - for the alternating tree
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

void Matching::hungarian()
{
  //int ret = 0;                      //weight of the optimal matching
    int n = N;
    max_match = 0;                    //number of vertices in current matching
    memset(xy, -1, n*sizeof(*xy));    
    memset(yx, -1, n*sizeof(*yx));

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


std::vector<Edge> Matching::mmd_msd2(Test t){
  int n = N;
//std::cout << "N is: "<<N<<std::endl;
  cost = new double*[N];
  for (int i=0; i<N; i++)
	cost[i] = new double[N];
  lx = new double[N];
  ly = new double[N];        //labels of X and Y parts
  xy = new int[N];               //xy[x] - vertex that is matched with x,
  yx = new int[N];               //yx[y] - vertex that is matched with y
  S = new bool[N];
  T = new bool[N];         //sets S and T in algorithm
  slack = new double[N];            //as in the algorithm description
  slackx = new int[N];           //slackx[y] such a vertex, that
                         // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
  prev = new int[N];             //array for memorizing alternating paths

  
  std::vector<Edge> edges;
  std::vector<Edge> answer;

  // Create edges from all pairwise distances squared
  for(int i = 0; i < n; i++)
    for(int j = 0; j < n; j++)
      if ( t.starts[i].first >= nCars || t.targets[i].first >= nTrips){ // Is dummy location
	 edges.push_back(std::make_pair(0.0,
                                     std::make_pair(t.starts[i].first, t.targets[j].first)));

      } else {
	double dist = pow(getdist(t.starts[i], t.targets[j]),2);
        edges.push_back(std::make_pair(dist,
                                     std::make_pair(t.starts[i].first, t.targets[j].first)));
       
	}
 std::sort(edges.begin(), edges.end());
  if (n * n != edges.size())
	std::cout << "not enough edges"<<std::endl;

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
//  std::cout<<"algorithm ran"<<std::endl;
  for(int i = 0; i < n; i++){
    //printf("Got: %d %d %lf\n", i, h2[i], getdist(t.starts[i], t.targets[h2[i]]));
    if (t.starts[i].first < nCars && t.targets[xy[i]].first < nTrips)
      answer.push_back(std::make_pair(getdist(t.starts[i], t.targets[xy[i]]),
                                    std::make_pair(t.starts[i].first, t.targets[xy[i]].first)));
  }

	delete[] lx;
	delete[] ly;
        delete[] xy;
        delete[] yx;
        delete[] T;
        delete[] S;
        delete[] slack;
        delete[] slackx;
        delete[] prev;

        for(int i=0; i<N; i++)
                delete[] cost[i];
        delete[] cost;


  // answer now contains the correct answer.

  return answer;
}

#endif // ALGORITHM == MMD_MSD2

////////////  End implementation of mmd_msd2  ////////////////

/*
void Matching::initializeMatrix(vector<vector<int> > &matrix)
{
//cout << matrix.size() << endl;	
	int c,t, dist, max = 0;
	for (vector<edge*>::iterator it = edge_set.begin(); it != edge_set.end(); it++)
	{
		t = (*it)->trip;
		c = (*it)->car;
		dist = (*it)->distance;
		matrix[t][c] = dist;
		if (max < dist)
			max = dist;
	}

	for(t=0; t<matrix.size(); t++)
		for(c=0; c<matrix.size(); c++)
			if(matrix[t][c] == -1)
				matrix[t][c] = max;
}

std::vector<edge*> Matching::hungarianAlgorithm()
{

	vector<edge*> matching;
	vector<vector <int> > cost_matrix(nTrips,vector <int> (nCars,-1));
	vector<vector <int> > assignment;

//	for(int i=0; i<nTrips; i++)
//		cost_matrix[i] = vector<int>(nCars,-1);

	initializeMatrix(cost_matrix);

	Hungarian hungarian(cost_matrix, nTrips, nCars, HUNGARIAN_MODE_MINIMIZE_COST) ;
	cout << "Cost: "<< endl;
	hungarian.print_cost();
	hungarian.solve();
	cout << "Assignment: "<<endl;
	hungarian.print_assignment();
	assignment = hungarian.assignment();
	cout << assignment.size() << " size" <<endl;
	cout << "cost: "<<hungarian.cost()<<endl;
	for (int i=0; i< nTrips; i++){

		for (int j=0; j< nCars; j++){

			if (assignment[i][j] == HUNGARIAN_ASSIGNED){
				cout << "in here"<<endl;
				for (vector<edge*>::iterator it = edge_set.begin(); it != edge_set.end(); it++){

					if((*it)->trip == i && (*it)->car == j){matching.push_back((*it)); break;}

				}
			}			
		}
	}
//	hungarian.print_cost();
	return matching;
}
*/
