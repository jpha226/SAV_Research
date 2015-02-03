#include<time.h>
#include "matching.h"
#include<iostream>
#include <cstdlib>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

using namespace std;


int main(int argc, char* argv[]){

	Matching matching;
	srand(atoi(argv[3]));
	int n = atoi(argv[1]);
	int m = atoi(argv[2]);
//	cout << argv[1] << endl;
	for (int i=0; i< n; i++){
		
		matching.addCar(rand()%40,rand()%40,i);

	}
	for (int i=0; i< m; i++){
		matching.addTrip(rand() % 40,rand() % 40, i, -1);
	}

	int x = rand() % 40;
	int y = rand() % 40;
	int offset = 1;
	matching.addTrip(x,y, m, offset);
	matching.addCar(x+offset,y,n);

	matching.setDimensions(n+1,m+1);
	
	clock_t t1,t2;
	t1 = clock();
	vector<Edge> result = matching.findMatching();
	t2 = clock();
	int carIndex, trip;
	 for (vector<Edge>::iterator it = result.begin(); it != result.end(); it++)
        {
                carIndex = (*it).second.first;
                trip = (*it).second.second;
		if (trip == m)
			cout << "Distance: " << (*it).first <<endl;
	}

	float diff = ((float)t2) - ((float)t1);
	float seconds = diff / CLOCKS_PER_SEC;
	cout << "Time: "<<seconds << endl;

	cout << "Matching size: "<<result.size()<<endl;
	return 0;
}
