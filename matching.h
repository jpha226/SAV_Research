#ifndef SCRAM_H
#define SCRAM_H


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

struct edge{

	int car;
	int trip;
	int distance;

};

typedef std::pair<int, std::pair<int, int> > Point;

// Edge looks like (dist, (left node, right node))
// dist, (carId, tripId)
typedef std::pair<double, std::pair<int, int> > Edge;

struct Test{
  std::vector<Point> starts;
  std::vector<Point> targets;
};


class Matching{



	public: 
		Matching();
		Matching(int num_cars, int num_trips);
		~Matching();
		void createEdge(int c, int trp, int dist);
		void addCar(int x, int y, int c);
		void addTrip(int x, int y, int t);
		std::vector<Edge> findMatching();	
		void initializeMatrix(std::vector<std::vector<int> > &matrix);
		std::vector<edge*> hungarianAlgorithm();
		void setDimensions(int nt, int nc);
		int flood(int x, int y, int prev);
		inline int reverse(int x, int y);
		inline void reset_flooding(int n);
		int getMinimalMaxEdgeInPerfectMatching(std::vector<Edge> edges, int n, int k);
		void init_labels();
		void update_labels();
		void add_to_tree(int x, int prevx);
		bool augment();
		void hungarian();
		std::vector<Edge> mmd_msd2(Test t);
		inline double getdist(const Point &a, const Point &b);


	private: 
		std::vector<edge*> edge_set;
		int nTrips;
		int nCars;
		int N;
		Test t;
		
		// vars for minmaxedge
		std::vector<std::vector<int> > out[2];  //adjacency list for each node
		std::vector<bool> visited[2];           //reached in floodfill
		std::vector<int> back[2];               //way to reach it
		std::vector<int> used;                  //whether a left node is used

		//mmd_ globals
		double** cost;
		int max_match;
		double* lx;
		double* ly;
		int *xy;
		int *yx;
		bool *S;
		bool *T;
		double *slack;
		int *slackx;
		int *prev;
};







#endif
