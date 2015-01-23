#include<time.h>
#include "matching.h"
#include<iostream>
#include <cstdlib>
#include <math.h>
#include <stdlib.h>


using namespace std;


int main(){

	Matching matching;
	srand(0);
	int n = 1600;
	int m = 1000;

	for (int i=0; i< n; i++){
		
		matching.addCar(rand()%(n*n),rand()%(n*n),i);

	}
	for (int i=0; i< m; i++){
		matching.addTrip(rand()%(n*n),rand()%(n*n),i);
	}

	matching.setDimensions(n,m);
	
	clock_t t1,t2;
	t1 = clock();
	matching.findMatching();
	t2 = clock();
	
	float diff = ((float)t2) - ((float)t1);
	float seconds = diff / CLOCKS_PER_SEC;
	cout << "Time: "<<seconds << endl;


	return 0;
}
