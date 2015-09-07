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
	int fleet_size;
	int input;
	int seed = 0;
	char* inputFile = "input.txt";	

	if (argc <= 1)
//		simulator = new Simulator(1000000,0, inputFile);
		cout << "Usage: ./simulator --fleet-size [int] --seed [int] --input [string]" << endl;
	else{
		int arg_index = 0;
		while (arg_index < argc)
		{
			if (strcmp (argv[arg_index],"--fleet-size") == 0){
				fleet_size = strtod(argv[arg_index + 1],NULL);
			}
			else if (strcmp (argv[arg_index],"--seed") == 0) {
				seed = strtod(argv[arg_index + 1], NULL);
			}
			else if (strcmp (argv[arg_index],"--input") == 0) {
				inputFile = argv[arg_index + 1];
			}
			arg_index++;
		}
		simulator = new Simulator(fleet_size, seed, inputFile);
		arg_index = 0;
		// Second pass for additional arguments if present
		while (arg_index < argc)
		{
			arg_index++;
		}
		simulator -> runSimulation();
	}
	return 0;
}
