#include "simulator.h"
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <sstream>




using namespace std;

int main(int argc, char *argv[]){

	Simulator* simulator;
	int xMax = 40;
	int yMax = 40;
	int input;

	if (argc <= 1)
		simulator = new Simulator(xMax,yMax);
	else{
		int arg_index = 0;
		while (arg_index < argc)
		{
			if (strcmp (argv[arg_index],"-s") == 0){
				xMax = strtod(argv[arg_index + 1],NULL);
				yMax = xMax;
			}
			arg_index++;
		}
		simulator = new Simulator(xMax, yMax);
		arg_index = 0;
		// Second pass for additional arguments if present
		while (arg_index < argc)
		{
			arg_index++;
		}
	}

	simulator->runSimulation();

	return 0;
}
