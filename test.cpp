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

	if (argc != 4)
		return 0;

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

	int x;
	int y;
	int offset[] = {1,2,5,1,3,5,2,4,2,1};
	int nr = 10;	


	for ( int i = 0; i < nr; i++){
		x = rand() % 40;
		y = rand() % 40;	
		matching.addTrip(x,y, m+i, offset[i]);
		matching.addCar(x+offset[i],y,n);
	}

	matching.setDimensions(n+nr,m+nr);
	
	clock_t t1,t2;
	t1 = clock();
	vector<Edge> result = matching.findMatching();
	t2 = clock();
	int carIndex, trip;
	 for (vector<Edge>::iterator it = result.begin(); it != result.end(); it++)
        {
                carIndex = (*it).second.first;
                trip = (*it).second.second;

		for ( int i=0; i<nr; i++)
			if (trip == m+i)
				cout << "Distance for " << trip << ": " << (*it).first << " from "<< offset[trip - m]<<endl;
	}

	float diff = ((float)t2) - ((float)t1);
	float seconds = diff / CLOCKS_PER_SEC;
	cout << "Time: "<<seconds << endl;

	cout << "Matching size: "<<result.size()<<endl;
	return 0;
}
