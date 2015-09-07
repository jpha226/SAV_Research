/*
Program: AV Car Share
Author: Daniel J. Fagnant
Organization: University of Texas at Austin
Revision Date: 5/21/2012
Description: AVCarShare is an agent-based program that simulates autonomous vehicle (AV) car sharing in an urban area.
  The program operates by generating trips throughout a single day in a grid-based zonal system.
  The program then runs a warm start.  In this, each trip looks for an existing shared AV in the same grid cell.
  If one is found, the traveler reserves that AV.  If not, the traveler looks for another nearby shared AV.
  If a shared AV still cannot be found, a new shared AV is generated.
  Once all trips have been assigned, the AVs move and the program moves on to the next time step.
  After the day is complete, all AVs are returned to their original start locations and the program is re-run.
  There are two optimization goals: 1. minimize the number of required AVs & 2. minimize unoccupied AV driving
Inputs: AVCarShareInput.txt should be located in the same directory as the executable program.
  The AVCarShareInput.txt file should have the following items:
  maxDist (integer), the maximum trip distance, in either the x or y direction (i.e. xMaxDist = yMaxDist = maxDist)
  outerRate (integer), the average trip generation rate for the outermost grid coordinates
  innerRate (integer), the average trip generation rate for the innermost grid coordinates
  reportProcs (0/1), boolean flag for reporting internal processes during steps
  saveRate (integer), the time interval at which the program will save data
  readFile (0/1), boolean flag to determine whether to start midway through a prior run
  readName (char*), name of the file to be read
  Each of these items should be on separate lines and formatted like "xMax=10" "maxTrav=4", etc.
  For input file comments, begin lines with the # character.
Outputs: This program outputs the number of shared AVs needed (N) to serve T trips, the shares of occupied travel time (OTT)
  and unoccupied travel time (UTT), and the maximum percentage of vehicles that were in use (MAXUSE) at any given time.
*/


// Libraries, structs, typedefs, constants, & function headers ******************************************************************************

#include <iostream>
#include <cstdlib>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <time.h>
#include "matching.h"
#include <sstream>
#include "simulator.h"

/*
#define LARGE 400
#define SMALL 40
#define GREEDY 0
#define SCRAM 1
#define SAV 2
#define SAEV 3
#define MERGE 4
#define SEPARATE 5
#define CONSTANT 6
#define VARY 7
#define REASSIGN 0


#define SIZE SMALL // 40 x 40 or 400 x 400 (Changes how trip generation rates are handled)
#define ALGORITHM GREEDY // Matching is done with either the original greedy approach or SCRAM
#define SPEED CONSTANT
#define SIMULATOR SAV // Sets car ranges and fuel times for either electric or gas vehicles
#define WAIT SEPARATE // Refers to giving all unmatched trip equal priority or separate
#define CHARGE_IN_PLACE true
#define SUPPLY_PRICING false
#define CHARGE_PRICING false
#define REALLOCATION_ON true
*/

/****
* The original simulator can be run by defining SIZE as SMALL, ALGORITHM as GREEDY, and SIMULATOR as SAV and WAIT as SEPARATE.
* The upgraded simulator for Donna Chen's research is ran as SIZE LARGE and SIMULATOR SAEV
* Experiments for the matching algorithm change the algorithm value
****/

using namespace std;


// Begin Program **************************************************************************************************
Simulator::Simulator(int fleet_size, int seed, char* inputFile)
{
	// Grid variables
	xMax = 40;
	yMax = 40;
	numZonesL = 5;
	numZonesS = 10;
	zoneSizeL = xMax / numZonesL;
	zoneSizeS = xMax / numZonesS;
	nearDist = 5.0; // These are wrong
	innerDist = 10.0; // These are wrong
	outerDist = 20.0; // These are wrong

	// Simulation variables
	maxNumRuns = 50;
	numWarmRuns = 20;

	// Range and fuel variables
	carRange = 1600;
	refuelTime = 2;
	refuelDist = 0;
	rejectLimit = 1000000;
	rangePercentToRefuel = 1.0;
	cellSize=0.25;

	fleetSizeLimit = fleet_size;
	limitFleetSize = false;

	rseed = seed;

	// Pricing variables
	base_price = 0.85 * cellSize;
	saev_vott = 0.35;
//	string inputFile = "input.txt";
        loadParameters(inputFile);
          
	cout << "Seed: " << rseed << endl;
	base_price += cellSize;
	cout << xMax << " " << yMax << endl;
	// Initialize grid structures for the map
	CarMx = new vector<Car>*[xMax];
	ChStMx = new vector<Station>*[xMax];
	cellChargeCount = new vector<int>*[xMax];
	cellChargeTime = new vector<int>*[xMax];
	for (int x=0; x<xMax; x++)
	{
		CarMx[x] = new vector<Car> [yMax];
		ChStMx[x] = new vector<Station> [yMax];
		cellChargeCount[x] = new vector<int> [yMax];
		cellChargeTime[x] = new vector<int> [yMax];
	}

	zoneSharesL = new double*[numZonesL];
	for(int x=0; x<numZonesL; x++)
		zoneSharesL[x] = new double [numZonesL];
	zoneSizeL = xMax / numZonesL;

	zoneSharesS = new double*[numZonesS];
	for(int x=0; x<numZonesS; x++)
		zoneSharesS[x] = new double [numZonesS];
	zoneSizeS = xMax / numZonesS;


	setTripDist();
	loadTripRateData("AustinTripRates.csv","AustinCellMap.csv");	
}

Simulator::Simulator(){ Simulator(10000000, 0, "input.txt");}

Simulator::~Simulator()
{
	for (int x=0; x<xMax; x++)
	{
		delete [] CarMx[x];
		delete [] ChStMx[x];
		delete [] cellChargeCount[x];
		delete [] cellChargeTime[x];
	}

	for (int x=0; x<numZonesL; x++)
		delete [] zoneSharesL[x];

	for (int x=0; x<numZonesS; x++)
		delete [] zoneSharesS[x];

	delete [] CarMx;
	delete [] ChStMx;
	delete [] cellChargeCount;
	delete [] cellChargeTime;
	delete [] zoneSharesL;
	delete [] zoneSharesS;
}

void Simulator::runSimulation()
{

	randomSeed = -1;
//	if (argc > 1)
//		randomSeed = atoi(argv[1]);
	    
//	strcpy(zName, "Warm Zones.txt");
	clock_t t1,t2,tot1,tot2;
	float time_diff, seconds;
	tot1 = clock();

	if (!canRefuelAnywhere){ // Charging stations only need to be generated if we are charging in place

		// Generate stations until convergence
		int prevMaxAvailStations = 1000;
		maxAvailStations = 0;
		int iter = 1;
		cout << abs(prevMaxAvailStations - maxAvailStations)/(1.0*prevMaxAvailStations) << endl;
		while ((abs(prevMaxAvailStations - maxAvailStations)/(1.0*prevMaxAvailStations) > 0.01) && iter < 100){

			prevMaxAvailStations = maxAvailStations;
		        initVars(iter, true, false);
		        findDistWeight();
			if (!error)
		        {
				if (!readFile){
		                    generateTrips(true);
				    //writeTripsToFile();
				}
				if (!readFile || wStart) {
					cout << "Run " << iter  << endl;
					runSharedAV(true,false,iter,false);//timeTripCounts, maxTrav, maxTravCongested, dwLookup, zoneSharesL, zoneSharesS, maxCarUse, maxCarOcc, totDist,
// 						unoccDist, waitT, reportProcs, saveRate, true, false, maxAvailCars, readFile, startIter, unservedT, waitCount, 
//						hotStarts, coldStarts, numRuns, iter, false);
					placeInitCars();//timeTripCounts, maxCarUse, maxCarOcc, totDist, unoccDist, waitT, dwLookup, reportProcs, hotStarts, coldStarts);
					cout << "Run " << iter + 1 << endl;
					cout << "Cars: " << maxAvailCars << endl;
					cout << "Stations: " << maxAvailStations << endl;
					cout << "Prev Stations: " << prevMaxAvailStations << endl;
				}
			}
			iter++;
			}
	}
        // Warm start for getting the number of vehicles
	for (int i = 1; i <= numWarmRuns; i++)
	{
		initVars(i,true,true);
		findDistWeight();

        	if (!error)
	        {
        		if (!readFile)
            		{
                		generateTrips (true);
	        	}
	        	if (!readFile || wStart) // run sharedAV if not restoring a previous run or if continuing a previous run still on warm start
            		{
				cout << "About to run program: "<< i << endl;
		        	t1 = clock();
				runSharedAV (true,false,i,true);//timeTripCounts,  maxTrav, maxTravCongested,  dwLookup, zoneSharesL, zoneSharesS, maxCarUse, maxCarOcc, totDist,
//					unoccDist, waitT, reportProcs, saveRate, true, false, maxAvailCars, readFile, startIter, unservedT, waitCount,
//					 hotStarts, coldStarts, numRuns, i, true);
         
				placeInitCars ();//timeTripCounts, maxCarUse, maxCarOcc, totDist, unoccDist, waitT, dwLookup, reportProcs, hotStarts,
					 //coldStarts);
				t2 = clock();
            		 }
			 time_diff = ((float)t2 - (float)t1);
			 seconds = time_diff / CLOCKS_PER_SEC;
	        	 cout << "Warm Run " << i << " completed. Time: " << seconds << endl;
			 nCars = 0; nStations = 0;
			 for(int x=0; x<xMax; x++){
				for(int y=0; y<yMax; y++){
					nCars += CarMx[x][y].size();
					nStations += ChStMx[x][y].size();
					if(ChStMx[x][y].size() > 1)
						cout << "problem w/ num charge stations"<<endl;
				}
	    		  }
			  cout << "nCars is "<<nCars <<"\nnStations is "<<nStations<<endl;
        	}	
    	}

    	// now we have the right # of vehicles, Do warm runs until we generate the right amount
    	distWt = netDistWt / numWarmRuns;
    	maxAvailCars = maxAvailCars / numWarmRuns;
    	maxAvailStations = maxAvailStations / numWarmRuns;
    	nCars = 0;
    	nStations = 0;
    	for (int i = 2; i < 100 && nCars < maxAvailCars; i++)
    	{
        	initVars (i,true,true);
	        cout << "Max avail cars is " << maxAvailCars << endl << endl;

        	cout << "Possible last warm start before running scenario" << endl;

	        generateTrips (true);
        	runSharedAV (true,true,i,true);// timeTripCounts, maxTrav, maxTravCongested,  dwLookup, zoneSharesL, zoneSharesS, maxCarUse, maxCarOcc, totDist, unoccDist,
//                     waitT, reportProcs, saveRate, true, true, maxAvailCars, readFile, startIter, unservedT, waitCount, hotStarts, coldStarts, numRuns, i, true);
        	placeInitCars ();//timeTripCounts, maxCarUse, maxCarOcc, totDist, unoccDist, waitT, dwLookup, reportProcs, hotStarts, coldStarts);

	        nCars = 0;
		nStations = 0;
	        for (int x = 0; x < xMax; x++)
        	{
	        	for (int y = 0; y < yMax; y++)
	            	{
        		        nCars = nCars + CarMx[x][y].size();
				nStations = nStations + ChStMx[x][y].size();
            		}
        	}
		cout << "Just generated: " <<nCars << endl;
    	}

    	cout << "nCars is " << nCars << endl;
	cout<< "nStations is "<<nStations << endl;
	//strcpy(zName, "Zones.txt");

    	// run the program
    	for (int i = 1; i <= numRuns; i++)
    	{
		initVars(i,false,true);
		if (randomSeed == -1)
			cout << "Run: "<<i<<" seed: "<< random_seeds[i-1] <<endl;
		else
			cout << "Run: "<<i<<" seed: "<< randomSeed <<endl;
	
	        placeInitCars ();//timeTripCounts, maxCarUse, maxCarOcc, totDist, unoccDist, waitT, dwLookup, reportProcs, hotStarts, coldStarts);
        	generateTrips (false);
		t1 = clock();
        	runSharedAV (false,false,i,true);// timeTripCounts,  maxTrav, maxTravCongested,  dwLookup, zoneSharesL, zoneSharesS, maxCarUse, maxCarOcc, totDist, unoccDist, waitT,
//                             reportProcs, saveRate, false, false, maxAvailCars, readFile, startIter, unservedT, waitCount, hotStarts, coldStarts, numRuns, i, true);
		t2 = clock();
		time_diff = ((float)t2 - (float)t1);
		seconds = time_diff / CLOCKS_PER_SEC;
		if (false)//matchAlgorithm == SCRAM)
		        reportMatchingResults();
		else{
			//printParameters();
			reportResults (i);// timeTripCounts,  maxCarUse, maxCarOcc, totDist, unoccDist, waitT, unservedT, waitCount, hotStarts, coldStarts,
                       // totDistRun, totUnoccDistRun, totCarsRun, totTripsRun, totHSRun, totCSRun, totWaitTRun, totUnservedTRun, totWaitCountRun,
                       // totUnusedRun, totUnoccRun, totAvgWait, totAvgTripDist, numRuns, i); 
		}
		cout << "Completion time: " <<seconds << endl;
    }
    
    	if (numRuns > 1)
    	{
        	reportFinalResults(totDistRun, totUnoccDistRun, totCarsRun, totTripsRun, totHSRun, totCSRun, totWaitTRun, totUnservedTRun, totWaitCountRun,
                           totUnusedRun, totUnoccRun, totAvgWait, totAvgTripDist, numRuns);
    	}

    	tot2 = clock();
	time_diff = ((float)tot2 - (float)tot1);
	seconds = time_diff / CLOCKS_PER_SEC;
    	cout << "Total time: " << seconds;
}

//****Major Functions called from Main*****************************************************************************************************************

// initializes all variables
void Simulator::initVars (int runNum, bool warmStart, bool checkStationDistance)
{
    double inputVal;
    char comment;
    char* varStr;
    char* valStr;
    char instring [80];

    error = false;
    wStart = true;
//    outerRate = -1;
//    innerRate = -1;
//    nearRate = -1;
//    exurbanRate = -1;
//    maxTrav = -1;
    

    for(int t = 0; t < 288; t++)
    {
        maxCarUse[t] = 0;
        maxCarOcc[t] = 0;
    }

    revenue = 0;
    totDist = 0;
    unoccDist = 0;
    reallocDist = 0;
    waitT = 0;
    reportProcs = false;
    saveRate = 0;
    startIter = 0;
//    readFile = false;
    hotStarts = 0;
    coldStarts = 0;
    tripsReassigned = 0;

    maxCarInUse = 0;
    maxCarCharging = 0;
    maxCarNoUse = 0;

    maxInUseCharge = 0;
    maxChargingCharge = 0;
    maxNoUseCharge = 0;

    if (runNum == 1)
    {
	if (warmStart)
		prev_avail_cars = 0;
	else
	        prev_avail_cars = nCars;
	totTripsReassigned = 0;
        totDistRun = 0;
        totUnoccDistRun = 0;
	totUnoccChargeDistRun = 0;
	totNumChargeRun = 0;
	totChargeTimeRun = 0;
        totCarsRun = 0;
        totTripsRun = 0;
        totHSRun = 0;
        totCSRun = 0;
        totWaitTRun = 0;
        totUnservedTRun = 0;
        totUnusedRun = 0;
        totUnoccRun = 0;
        totAvgWait = 0;
        totAvgTripDist = 0;
        totWaitCOV = 0;
        totTripDistCOV = 0;
        totCarTripsCOV = 0;
        netDistWt = 0;
        maxAvailCars = 0;

	totMaxCarInUse = 0;
	totMaxCarNoUse = 0;
	totMaxCarCharging = 0;	

	totMaxInUseCharge = 0;
	totMaxNoUseCharge = 0;
	totMaxChargingCharge = 0;

        for (int i = 0; i < 6; i++)
        {
            totWaitCountRun[i] = 0;
	   // waitList[i].clear();
        }

        for (int i = 0; i < numRuns; i++)
        {
            totDistRunCOV.push_back(0);
            totUnoccDistRunCOV.push_back(0);
	    totUnoccChargeDistRunCOV.push_back(0);
	    totChargeTimeRunCOV.push_back(0);
	    totNumChargeRunCOV.push_back(0);
            totCarsRunCOV.push_back(0);
            totTripsRunCOV.push_back(0);
            totHSRunCOV.push_back(0);
            totCSRunCOV.push_back(0);
            totWaitTRunCOV.push_back(0);
            totUnservedTRunCOV.push_back(0);
            totWaitCountRunCOV.push_back(0);
            totUnusedRunCOV.push_back(0);
            totUnoccRunCOV.push_back(0);
            totAvgWaitCOV.push_back(0);
            totAvgTripDistCOV.push_back(0);
            totStartsPerTripCOV.push_back(0);
            totAvgTripsPerCarCOV.push_back(0);

	    totMaxCarInUseCOV.push_back(0);
	    totMaxCarChargingCOV.push_back(0);
	    totMaxCarNoUseCOV.push_back(0);

            totMaxInUseChargeCOV.push_back(0);
            totMaxChargingChargeCOV.push_back(0);
            totMaxNoUseChargeCOV.push_back(0);


            totPctMaxWaitFiveCOV.push_back(0);
            totPctInducedTCOV.push_back(0);
            totPctMaxInUseCOV.push_back(0);
            totPctMaxOccCOV.push_back(0);
            totPctColdShareCOV.push_back(0);
	    totRevenueCOV.push_back(0);
	    totReallocDistRunCOV.push_back(0);
	    totCongestTimeRunCOV.push_back(0);
        }
    }
/*
    FILE* inputfile;
    inputfile = fopen("input.txt", "r");


    while (!feof(inputfile))
    {
        fgets(instring, 80, inputfile);

        comment = instring[0];
        if (comment != '#')
        {
            varStr = strtok(instring, "=");
            valStr = strtok(NULL, "\0");

            if (strcmp (varStr, "outerRate") == 0) {
                inputVal = strtod(valStr, NULL);
                outerRate = (int) inputVal;
            } else if (strcmp (varStr, "innerRate") == 0) {
                inputVal = strtod(valStr, NULL);
                innerRate = (int) inputVal;
            } else if (strcmp (varStr, "nearRate") == 0) {
                inputVal = strtod(valStr, NULL);
                nearRate = (int) inputVal;
	    } else if (strcmp (varStr, "exurbanRate") == 0) {
		inputVal = strtod(valStr, NULL);
		exurbanRate = (int) inputVal;
            } else if (strcmp (varStr, "maxTrav") == 0) {
                inputVal = strtod(valStr, NULL);
                maxTrav = (int) inputVal;
            } else if (strcmp (varStr, "maxTravC") == 0) {
                inputVal = strtod(valStr, NULL);
                maxTravCongested = (int) inputVal;
            } else if (strcmp (varStr, "reportProcs") == 0) {
                inputVal = strtod(valStr, NULL);
                reportProcs = (bool) inputVal;
            } else if (strcmp (varStr, "saveRate") == 0) {
                inputVal = strtod(valStr, NULL);
                saveRate = (int) inputVal;
            } else if (strcmp (varStr, "readFile") == 0) {
                inputVal = strtod(valStr, NULL);
                readFile = (bool) inputVal;
            } else if (strcmp (varStr, "readName") == 0) {
                strcpy (readName, valStr);
            } else if (strcmp (varStr, "rseed") == 0) {
                inputVal = strtod(valStr, NULL);
                rseed = (int) inputVal;
            } else if (strcmp (varStr, "numRuns") == 0) {
                inputVal = strtod(valStr, NULL);
                numRuns = (int) inputVal;
            }
        }
    }

    if (warmStart)
      prev_avail_cars = 0;

    fclose(inputfile);
*/
    if (runNum == 1)
    {
	cout <<"Seed"<<endl;
	srand(rseed);
	if (!warmStart){
		random_seeds.clear();
		for (int i = 0; i < numRuns; i++)
		{
			random_seeds.push_back(rand());
		}
	}

	if (!warmStart)
	{
		for(int x=0; x<xMax; x++)
			for(int y=0; y<yMax; y++){
				cellChargeCount[x][y].clear();
				cellChargeTime[x][y].clear();
				//chargeCongestionCount[x][y].clear();
			}
	
	
	}
	for (int x=0; x<xMax; x++){
		for (int y=0; y<yMax; y++){
			for (int i=0; i< numRuns; i++){
				cellChargeCount[x][y].push_back(0);
				cellChargeTime[x][y].push_back(0);
				//chargeCongestionCount[x][y].push_back(0);
			}
		}
	}
	
    }

    if (!warmStart)
	srand(random_seeds[runNum]);
    if(!warmStart && runNum ==1)
	srand(rseed);

    //fclose(inputfile);

    if (readFile)
    {
        restoreStatus( timeTripCounts,  maxTrav,  maxCarUse, totDist, unoccDist, reportProcs, saveRate, wStart, startIter, readName, error, wStart, startIter);
    } else {

        if (outerRate < 0 || innerRate < 0 || maxTrav <= 0 || saveRate < 0)
        {
            cout << "Input file read error! At least one of outerRate, innerRate, maxTrav, or saveRate not defined or invalid entry." << endl;
	    cout << "OR: "<<outerRate<<" IR: "<<innerRate<< " mT: "<<maxTrav<<" sR: "<<saveRate<<endl;
            cin >> maxTrav; // dummy input
            error = true;
        }

        for (int t = 0; t < 288; t++)
        {
            	TTMx[t].clear();
		timeTripCounts[t] = 0;
	}

        if (warmStart)
        {
            for (int x = 0; x < xMax; x++)
            {
                for (int y = 0; y < yMax; y++)
                {
		    if (canRefuelAnywhere || checkStationDistance)
	                    CarMx[x][y].clear();
		    if (!canRefuelAnywhere)
		    {
			CarMx[x][y].clear();
			//ChStMx[x][y].clear();	
			//Car c;
			//c.x = x;
			//c.y = y;
			//genNewStation(c);
		    }
                }
            }
        }
	
        setZoneSharesS (zoneSharesS, outerRate, innerRate, nearRate, exurbanRate, numZonesS, zoneSizeS);
	setZoneSharesL (zoneSharesL, outerRate, innerRate, nearRate, exurbanRate, numZonesL, zoneSizeL);
        setStartTimes (startTimes);
        
	if (xMax == 400)
		setTripDistLarge ();
	else
		setTripDist();
	setPersonalTripVOTT();
	setPersonalTripProbability();
	setBusinessTripVOTT();
	setBusinessTripProbability();
	setDwTimes (dwLookup);

    }
    if (!canRefuelAnywhere){
      for(int x=0; x<xMax; x++){
	for(int y=0; y<yMax; y++){
		if(ChStMx[x][y].size() > 0){
			ChStMx[x][y][0].chargeTime = 0;
			ChStMx[x][y][0].unoccupiedDist = 0;
			ChStMx[x][y][0].numCharges = 0;
			ChStMx[x][y][0].congestTime = 0;
		}	
	}
      }
    }

    return;
}

bool Simulator::loadParameters(char* input)
{
   FILE* inputfile;
   inputfile = fopen(input, "r");
   if (inputfile == NULL){
     cout << "Could not open "<<input<<endl;
     return false;
   }
   double inputVal = -1.0;
   char* varStr;
   char* valStr;
   char instring [80];
   //rseed = 0;
   readName = new char[20];
   char comment;

    while (!feof(inputfile))
    {
        fgets(instring, 80, inputfile);
        comment = instring[0];
        if (comment != '#' && comment != '\n')
        {
            varStr = strtok(instring, "=");
            valStr = strtok(NULL, "\0");

	    if (strcmp (varStr, "xMax") == 0) {
		inputVal = strtod(valStr, NULL);
		xMax = 4 * (int) inputVal;
	    } else if (strcmp (varStr, "yMax") == 0) {
		inputVal = strtod(valStr, NULL);
		yMax = 4 * (int) inputVal;
	    } else if (strcmp (varStr, "numZonesL") == 0) {
		inputVal = strtod(valStr, NULL);
		numZonesL = (int) inputVal;
	    } else if (strcmp (varStr, "numZonesS") == 0) {
		inputVal = strtod(valStr, NULL);
		numZonesS = (int) inputVal;
            } else if (strcmp (varStr, "outerRate") == 0) {
                inputVal = strtod(valStr, NULL);
                outerRate = (int) inputVal;
            } else if (strcmp (varStr, "innerRate") == 0) {
                inputVal = strtod(valStr, NULL);
                innerRate = (int) inputVal;
            } else if (strcmp (varStr, "nearRate") == 0) {
                inputVal = strtod(valStr, NULL);
                nearRate = (int) inputVal;
            } else if (strcmp (varStr, "exurbanRate") == 0) {
                inputVal = strtod(valStr, NULL);
                exurbanRate = (int) inputVal;
            } else if (strcmp (varStr, "maxTrav") == 0) {
                inputVal = strtod(valStr, NULL);
                maxTrav = (int) inputVal;
            } else if (strcmp (varStr, "maxTravC") == 0) {
                inputVal = strtod(valStr, NULL);
                maxTravCongested = (int) inputVal;
            } else if (strcmp (varStr, "reportProcs") == 0) {
                inputVal = strtod(valStr, NULL);
                reportProcs = (bool) inputVal;
            } else if (strcmp (varStr, "saveRate") == 0) {
                inputVal = strtod(valStr, NULL);
                saveRate = (int) inputVal;
            } else if (strcmp (varStr, "readFile") == 0) {
                inputVal = strtod(valStr, NULL);
                readFile = (bool) inputVal;
            } else if (strcmp (varStr, "readName") == 0) {
                strcpy (readName, valStr);
            } else if (strcmp (varStr, "rseed") == 0) {
                inputVal = strtod(valStr, NULL);
                rseed = (int) inputVal;
            } else if (strcmp (varStr, "numRuns") == 0) {
                inputVal = strtod(valStr, NULL);
                numRuns = (int) inputVal;
            } else if (strcmp (varStr, "businessVottDistributionFile") == 0) {
                strcpy (businessVottDistributionFile, valStr);
            } else if (strcmp (varStr, "personalVottDistributionFile") == 0) {
                strcpy (personalVottDistributionFile, valStr);
            } else if (strcmp (varStr, "personalTripProb") == 0) {
                inputVal = strtod(valStr, NULL);
                personalTripProb = (double) inputVal;
            } else if (strcmp (varStr, "transitPrice") == 0) {
                inputVal = strtod(valStr, NULL);
                transitPrice = (double) inputVal;
            } else if (strcmp (varStr, "ridePriceScheme") == 0) {
                inputVal = strtod(valStr, NULL);
                ridePriceScheme = (int) inputVal;
            } else if (strcmp (varStr, "rideBasePrice") == 0) {
                inputVal = strtod(valStr, NULL);
                rideBasePrice = (double) inputVal;
            } else if (strcmp (varStr, "matchAlgorithm") == 0) {
                inputVal = strtod(valStr, NULL);
                matchAlgorithm = (int) inputVal;
            } else if (strcmp (varStr, "tripPool") == 0) {
                inputVal = strtod(valStr, NULL);
                tripPool = (bool) inputVal;
            } else if (strcmp (varStr, "carSpeed") == 0) {
                inputVal = strtod(valStr, NULL);
                carSpeed = (int) inputVal;
            } else if (strcmp (varStr, "reassignTrips") == 0) {
                inputVal = strtod(valStr, NULL);
                reassignTrips = (bool) inputVal;
            } else if (strcmp (varStr, "reallocationOn") == 0) {
                inputVal = strtod(valStr, NULL);
                reallocationOn = (bool) inputVal;
            } else if (strcmp (varStr, "canRefuelAnywhere") == 0) {
                inputVal = strtod(valStr, NULL);
                canRefuelAnywhere = (bool) inputVal;
            } else if (strcmp (varStr, "nearDist") == 0) {
                inputVal = strtod(valStr, NULL);
                nearDist = (int) 4 * inputVal;
            } else if (strcmp (varStr, "innerDist") == 0) {
                inputVal = strtod(valStr, NULL);
                innerDist = (int) 4 * inputVal;
            } else if (strcmp (varStr, "outerDist") == 0) {
                inputVal = strtod(valStr, NULL);
                outerDist = (int) 4 * inputVal;
            } else if (strcmp (varStr, "carRange") == 0) {
                inputVal = strtod(valStr, NULL);
                carRange = (int) 4 * inputVal;
            } else if (strcmp (varStr, "refuelTime") == 0) {
                inputVal = strtod(valStr, NULL);
                refuelTime = (int) (inputVal / 5);
            } else if (strcmp (varStr, "numRejectsToRefuel") == 0) {
                inputVal = strtod(valStr, NULL);
                numRejectsToRefuel = (int) inputVal;
            } else if (strcmp (varStr, "rangePercentToRefuel") == 0) {
                inputVal = strtod(valStr, NULL);
                rangePercentToRefuel = (double) inputVal;
            } else if (strcmp (varStr, "tripDistDistributionFile") == 0) {
                strcpy(tripDistDistributionFile, valStr);
            } else if (strcmp (varStr, "useCityTripData") == 0) {
                inputVal = strtod(valStr, NULL);
                useCityTripData = (bool) inputVal;
            } else if (strcmp (varStr, "cellSize") == 0) {
                inputVal = strtod(valStr, NULL);
                cellSize = (float) inputVal;
            } else if (strcmp (varStr, "limitGreedySearch") == 0) {
	        inputVal = strtod(valStr,NULL);
                limitGreedySearch = (bool) inputVal;
	    } else if (strcmp (varStr, "limitFleetSize") == 0) {
		inputVal = strtod(valStr,NULL);
		limitFleetSize = (bool) inputVal;
	    }

        }
    }
    return true;
}

void Simulator::printParameters()
{
	cout << "xMax: " <<xMax << endl;
	cout << "yMax: " <<yMax << endl;
	cout << "numZonesL: "<<numZonesL<<endl;
	cout<<"numZonesS: "<<numZonesS<<endl;
	cout<<"outerRate: "<<outerRate<<endl;
	cout<<"innerRate: "<<innerRate<<endl;
	cout<<"nearRate: "<<nearRate<<endl;
	cout<<"exurbanRate: "<<exurbanRate<<endl;
	cout<<"maxTrav: "<<maxTrav<<endl;
	cout<<"maxTravC: "<<maxTravCongested<<endl;
	cout<<"reportProcs: "<<reportProcs<<endl;
	cout<<"saveRate: "<<saveRate<<endl;
	cout<<"readFile: "<<readFile<<endl;
	cout<<"rseed: "<<rseed<<endl;
	cout<<"readName: "<<readName<<endl;
	cout<<"numRuns: "<<numRuns<<endl;
	cout<<"businessVottDistributionFile: "<<businessVottDistributionFile<<endl;
	cout<<"pVDF: "<<personalVottDistributionFile<<endl;
	cout<<"personalTripProb: "<<personalTripProb<<endl;
	cout <<"transitPrice: "<<transitPrice<<endl;
	cout <<"ridePriceScheme: "<<ridePriceScheme<<endl;
	cout<<"rideBasePrice: "<<rideBasePrice<<endl;
	cout<<"matchAlgorithm: "<<matchAlgorithm<<endl;
	cout<<"tripPool: "<<tripPool<<endl;
	cout<<"carSpeed: "<<carSpeed<<endl;
	cout<<"reassignTrips: "<<reassignTrips<<endl;
	cout<<"reallocationOn: "<<reallocationOn<<endl;
	cout<<"canRefuelAnywhere: "<<canRefuelAnywhere<<endl;
	cout<<"nearDist: "<<nearDist<<endl;
	cout<<"innerDist: "<<innerDist<<endl;
	cout<<"outerDist: "<<outerDist<<endl;
	cout<<"carRange: "<<carRange<<endl;
	cout<<"refuelTime: "<<refuelTime<<endl;
	cout<<"numRejectsToRefuel: "<<numRejectsToRefuel<<endl;
	cout<<"rangePercentToRefuel: "<<rangePercentToRefuel<<endl;
	cout<<"useModeChoice: "<<useModeChoice<<endl;
	cout<<"tDDF: "<<tripDistDistributionFile<<endl;
	cout<<"useCityTripData: "<<useCityTripData<<endl;

}

void Simulator::loadTripRateData(char* rateData, char* cellMap)
{

    ifstream infile(rateData);
    ifstream mapFile(cellMap);
    string line = "";
    getline(infile, line);
    numTripOrigins = (int)strtod(line.c_str(),NULL);
    int row = 0;
    int col = 0;
    int id,x,y;
    char comma;

    cityTripRates = new double*[numTripOrigins];
    xMap = new int[numTripOrigins];
    yMap = new int[numTripOrigins];
    for (int i=0; i<numTripOrigins; i++)
      cityTripRates[i] = new double[numTripOrigins];

    getline(infile, line); // skip the header line

    while ( getline(infile, line)) {
      stringstream strstr(line);
      string word = "";
      col = 0;
      while ( getline(strstr,word,',')) {
        if (col == 0){
          col++;
          continue;
        } else {
          cityTripRates[row][col - 1] = (double) strtod(word.c_str(),NULL);
          col++;
        }
      }
      row ++;
    }

    // Skip first two lines
    getline(mapFile,line);
    getline(mapFile,line);

    while (mapFile >> id >> comma >> x >> comma >> y) {

      xMap[id - 1] = x;
      xMap[id - 1] = y;

    }
    mapFile.close();
    infile.close();

}

// determines weights for AM-PM trip balancing
void Simulator::findDistWeight()
{

    double tripDiffCL, tripDiffCR;

    double Llimit = 0;
    double Rlimit = 1;

    distWt = 1;

    tripDiffCL = countDiff (outerRate, innerRate, nearRate, exurbanRate, startTimes, tripDist,  timeTripCounts, ((Llimit * 2) + Rlimit) / 3, reportProcs);
    tripDiffCR = countDiff (outerRate, innerRate, nearRate, exurbanRate, startTimes, tripDist,  timeTripCounts, (Llimit + (Rlimit * 2)) / 3, reportProcs);

    // run 20 iterations, to get a sense of overall balancing
    for (int i = 0; i < 20 && abs(int(tripDiffCL)) > 10 && abs(int(tripDiffCR)) > 10; i++)
    {
        tripDiffCL = countDiff (outerRate, innerRate, nearRate, exurbanRate, startTimes, tripDist,  timeTripCounts, ((Llimit * 2) + Rlimit) / 3, reportProcs);
        tripDiffCR = countDiff (outerRate, innerRate, nearRate, exurbanRate, startTimes, tripDist,  timeTripCounts, (Llimit + (Rlimit * 2)) / 3, reportProcs);

        if (tripDiffCR > tripDiffCL)
        {
            // inconclusive result
            Rlimit = Rlimit;
        } else if (abs(int(tripDiffCL)) < abs(int(tripDiffCR))) {
            Rlimit = (Llimit + (Rlimit * 2)) / 3;
        } else {
            Llimit = ((Llimit * 2) + Rlimit) / 3;
        }

    }

    if (abs(int(tripDiffCL)) <= 10)
    {
        distWt = ((Llimit * 2) + Rlimit) / 3;
    } else if (abs(int(tripDiffCR)) <= 10) {
        distWt = (Llimit + (Rlimit * 2)) / 3;
    } else {
        distWt = (Rlimit + Llimit) / 2;
    }

    netDistWt = netDistWt + distWt;

    if (reportProcs)
    {
        cout << "Dist L = " << tripDiffCL << " Dist R = " << tripDiffCR << endl;
        cout << "Final wt " << distWt << endl;
    }

    return;
}

// generates all trips to be undertaken in a given day
void Simulator::generateTrips (bool warmStart)
{

    if (useCityTripData) {
      generateTripsWithData(warmStart);
      return;
    }

    int time, newX, newY;
    double rate;
    int zones [xMax][yMax];
    int destZones [xMax][yMax];
    double r, dWt;
    int totTrips = 0;

    vector <int> xvect;
    vector <int> yvect;
    vector <int>::iterator itx;
    vector <int>::iterator ity;

    ofstream ZoneFile;

    // randomize x and y ordering

    for (int x = 0; x < xMax; x++)
    {
        xvect.push_back(x);
    }

    for (int y = 0; y < yMax; y++)
    {
        yvect.push_back(y);
    }

    random_shuffle(xvect.begin(), xvect.end());
    random_shuffle(yvect.begin(), yvect.end());

    if (reportProcs)
    {
        for (itx = xvect.begin(); itx != xvect.end(); ++itx)
        {
            cout << *itx << " ";
        }
        cout << endl << endl;
    }


    r = sqrt (pow(double(xMax - 1) / 2,2) + pow(double(yMax - 1) / 2,2));  //radius from the city center
    Trip nTrip;

    // generate trips in each zone

    for (int xd = 0; xd < xMax; xd++)
    {
        for (int yd = 0; yd < yMax; yd++)
        {
            rate = getRate (xd, yd, r, double(xMax - 1) / 2, double(yMax - 1) / 2, outerRate, innerRate, nearRate, exurbanRate);
            zones[xd][yd] =  genPoisson (rate);
        }
    }
   

    // assign departure times to each trip
int sum = 0;
int count =0; // for averaging
int bus = 0;
int redrawnCt = 0;
    for (itx = xvect.begin(); itx != xvect.end(); ++itx)
    {
	    for (ity = yvect.begin(); ity != yvect.end(); ++ity)
        {
            for (int t = 0; t < zones[*itx][*ity]; t++)
            {
                time = getStartTime(startTimes);

                // greater attraction to the center in the morning
                if (time > 120)
                {
                    dWt = distWt;
                } else {
                    dWt = 0.5;
                }

                getDest (*itx, *ity, newX, newY, tripDist, dWt);
		if (warmStart&&!warmStart){
			bool redrawn = false;
	
			while ( abs(*itx - newX) + abs(*ity - newY) > 50 * (1.0/cellSize))
			{
				if(redrawn)
					redrawnCt ++;
			redrawn = true;
				getDest(*itx, *ity, newX, newY, tripDist, dWt);
	
			}
		}

		double type = rand();
		type /= RAND_MAX;
		double VOTT;
		if (type < 0.1866)
			nTrip.isBusiness = true;
		else
			nTrip.isBusiness = false;

                nTrip.startTime = time;
                nTrip.startX = *itx;
                nTrip.startY = *ity;
                nTrip.endX = newX;
                nTrip.endY = newY;
                nTrip.carlink = false;
                nTrip.returnHome = false;
                nTrip.waitTime = 0;
                nTrip.tripDist = (abs(nTrip.startY - nTrip.endY) + abs(nTrip.startX - nTrip.endX));
                nTrip.tripDist = nTrip.tripDist / (1.0/cellSize);
                nTrip.waitPtr = NULL;
		nTrip.price = 0.0;
		//double saev_wait = 2.5;
		//getTripTravelMode(&nTrip,saev_wait);
		sum += nTrip.tripDist; // in miles
		count++;	
                
                placeInTTM ( nTrip, timeTripCounts, time);
		if(nTrip.startX >= xMax)
			cout << "exceeded x boundary"<<endl;
            }
        }
    }
//double avg = 1.0 * sum;
//	cout << "Average: "<< avg / count << endl;
//    cout << "prob of transit: " << (bus * 1.0) / count << endl;
    double avgDist2 = 0;
    double numTrips2 = 0;

    for (int t = 0; t < 288; t++)
    {

        for (int trp = 0; trp < TTMx[t].size(); trp++)
        {
            avgDist2 = avgDist2 + TTMx[t][trp].tripDist;
        }
        numTrips2 = numTrips2 + TTMx[t].size();
    }

    avgDist2 = avgDist2 / numTrips2;

    /*if (printZ)
    //{

//        strcpy(zName,"tempZones.txt\0");
        for (int xd = 0; xd < xMax; xd++)
        {
            for (int yd = 0; yd < yMax; yd++)
            {
                destZones[xd][yd] = 0;
            }
        }


        for (int t = 0; t < 288; t++)
        {
            for (int c = 0; c < TTMx[t].size(); c++)
            {
                destZones[TTMx[t][c].endX][TTMx[t][c].endY]++;
            }
        }

        ZoneFile.open(zName);
        printZones(ZoneFile, zones, destZones);
        ZoneFile.close();

    }*/

    for (int tt = 0; tt < 288; tt++)
    {
        if (reportProcs)
        {
            cout << tt << " " << TTMx[tt].size() << endl;
        }
        totTrips = totTrips + TTMx[tt].size();
    }

    if (reportProcs)
    {
        cout << totTrips << " total trips created" << endl << endl;
    }

    //std::cout << "Generated " <<totTrips << " trips with " << redrawnCt << " redrawn" << std::endl;
    return;

}

void Simulator::generateTripsWithData(bool warmStart)
{
    double rate;
    int totTrips = 0;
    int tempTripCount = 0;
    int time;
    Trip nTrip;    

    for (int c1=0; c1<numTripOrigins; c1++) { // Change TC
      for (int c2=0; c2<numTripOrigins; c2++) { // Change TC

        tempTripCount = genPoisson (cityTripRates[c1][c2]); // Add argument to genPoisson
        for (int i=0; i<tempTripCount; i++) {

          time = getStartTime(startTimes);
          nTrip.startTime = time;
          nTrip.startX = xMap[c1];
          nTrip.startY = yMap[c1];
          nTrip.endX = xMap[c2];
          nTrip.endY = yMap[c2];
          nTrip.carlink = false;
          nTrip.returnHome = false;
          nTrip.waitTime = 0;
          nTrip.tripDist = (abs(nTrip.startY - nTrip.endY) + abs(nTrip.startX - nTrip.endX));
          nTrip.tripDist = nTrip.tripDist / (1.0/cellSize); // convert to miles
          nTrip.waitPtr = NULL;
          nTrip.price = 0.0;

          placeInTTM (nTrip, timeTripCounts, time);
        }
      }
    }

    for (int tt = 0; tt < 288; tt++)
    {
        if (reportProcs)
        {
            cout << tt << " " << TTMx[tt].size() << endl;
        }
        totTrips = totTrips + TTMx[tt].size();
    }

    std::cout << "Generated: " << totTrips << endl;

}

void Simulator::getTripTravelMode(Trip* trip,double saev_wait,double tripdemand_b, double tripdemand, double carsupply_b, double carsupply, bool warmStart)
{
	double VOTT;
	if (trip->isBusiness)
		VOTT = getBusinessVOTT();
	else
		VOTT = getPersonalVOTT();
	int time = trip->startTime;
	double trav_speed = 133.0;
	if (((time >= 84) && (time <= 96)) || ((time >= 192) && (time <= 222)))
		trav_speed = 123.0;
	double V_pv = -1 * VOTT * (trip->tripDist * (1.0/cellSize) / trav_speed) - 0.152 * (trip->tripDist * (1.0/cellSize));
	if (!trip->isBusiness)
	{
		double distToCen = sqrt (pow((trip->endX - xMax/2),2) + pow((trip->endY - yMax/2),2));
		if (distToCen < innerDist)
			V_pv -= 4;
		else if (distToCen < nearDist)
			V_pv -= 2;
	}
	double t_ao, t_ad; // time access origin and access destination
        double distToCenOrigin, distToCenDest;
        distToCenOrigin = sqrt ( pow((trip->startX - xMax/2),2) + pow(trip->startY - yMax/2,2));
        distToCenDest = sqrt ( pow((trip->endX - xMax/2),2) + pow((trip->endY - yMax/2),2));
        t_ao = getAccessTime(distToCenOrigin);
        t_ad = getAccessTime(distToCenDest);
        double wait = getWait(distToCenOrigin);
	double saev_price = base_price;//0.2125;
	double saev_multiplier = (tripdemand_b / tripdemand) * (carsupply / carsupply_b);
	if (saev_multiplier > 10.0)
	  saev_multiplier = 2.0;
	else if (saev_multiplier < 0.1)
	  saev_multiplier = 0.5;
	else
	  saev_multiplier = 1.0;
        trip->sb = carsupply_b;
	trip->sB = carsupply;
	trip->db = tripdemand_b;
	trip->dB = tripdemand;
	int chargeDistance = 0;
	if (!warmStart){
		if (SUPPLY_PRICING)
		  saev_price *= saev_multiplier;
		if (CHARGE_PRICING)
	          chargeDistance = nearestStationDistance(trip->endX,trip->endY); // in grid cells. Trip distance is in miles so multiply by 4.
	}

        double V_transit = -2.0 * VOTT * (t_ao + t_ad) - VOTT * (trip->tripDist * (1.0/cellSize) / 100.0) - 2.0;
	trip->price = saev_price * (trip->tripDist*(1.0/cellSize) + chargeDistance);
        double V_saev = -2.0 * VOTT * saev_wait - saev_vott * VOTT * (trip->tripDist * (1.0/cellSize) / trav_speed) - trip->price;
        double prob_pv = exp(V_pv) / (exp(V_pv) + exp(V_transit) + exp(V_saev));
        double prob_tr = exp(V_transit) / (exp(V_pv) + exp(V_transit) + exp(V_saev));

        double mode = rand();
        mode /= RAND_MAX;

        if (mode < prob_tr){
        	trip->modeOfTransit = 0;
        }
        else if (mode < prob_tr + prob_pv){
                trip->modeOfTransit = 1;
        }
        else{
                trip->modeOfTransit = 2;
        }
        trip->VOTT = VOTT;

	trip->V_tr = V_transit;
	trip->V_pv = V_pv;

}

void Simulator::writeTripsToFile()
{
	static bool done = false;

	if(done)
		return;
	done = true;
	ofstream outfile;
	outfile.open(mode_output);//"modeChoiceStats_supply_pricing.csv");
	int i = 0;
	for(int t=0; t<288; t++)
	{
		//cout << TTMx[t].size() << endl;
		for (int trp=0; trp<TTMx[t].size(); trp++)
		{
			if (i % 500 == 0){
				Trip trip = TTMx[t][trp];
				if(trip.isBusiness)
					outfile << "Business,";
				else
					outfile << "Personal,";
			        if (((t >= 84) && (t <= 96)) || ((t >= 192) && (t <= 222)))
					outfile <<"Peak,";
				else
					outfile <<"Off Peak,";

	               		double distToCenOrigin, distToCenDest;
	         	       distToCenOrigin = sqrt ( pow((trip.startX - xMax/2),2) + pow(trip.startY - yMax/2,2));
         			distToCenDest = sqrt ( pow((trip.endX - xMax/2),2) + pow((trip.endY - yMax/2),2));
				outfile << distToCenOrigin<<",";
				outfile << distToCenDest << ",";
				outfile << trip.VOTT<<",";
				outfile << trip.tripDist * (1.0/cellSize) << ",";
				outfile << "PV Util: "<< trip.V_pv<<",";
				outfile << "Trans Util: "<<trip.V_tr<<",";
				outfile << "supplyb: " << trip.sb<<",";
				outfile << "supplyB: "<< trip.sB<<",";
				outfile << "demandb: " << trip.db <<",";
				outfile << "demandB: " << trip.dB <<",";
				if (trip.modeOfTransit == 0)
					outfile << "Public Transit"<<endl;
				else if (trip.modeOfTransit == 1)
					outfile << "Private Vehicle" <<endl;
				 else{ outfile << "SAEV" << endl;}
			}
			i++;
		}
	}
	outfile.close();
}

double Simulator::getWait(double dist)
{
	if (dist < innerDist)
		return 0.0;
	else if (dist < nearDist)
		return 0.05;
	else if (dist < outerDist)
		return 0.1;
	else
		return 0.15;

}

double Simulator::getPersonalVOTT()
{
	double vot = rand();
	vot /= RAND_MAX;
	int index = 0;
	while (personalTripProbability[index] < vot){index++;}
	return personalTripVOTT[index - 1];
}

double Simulator::getBusinessVOTT()
{
	double vot = rand();
	vot /= RAND_MAX;
	int index = 0;
	while (businessTripProbability[index] < vot){index++;}
	return businessTripVOTT[index];

}

double Simulator::getAccessTime(double dist)
{
	if (dist < innerDist)
		return 0.05;
	else if (dist < nearDist)
		return 0.15;
	else if (dist < outerDist)
		return 0.35;
	else
		return 1.0;

}

//Runs the main thrust of the program - pairs all trips with vehicles and moves travelers from origin to destination
void Simulator::runSharedAV (bool warmStart, bool lastWarm, int iter, bool checkStationDistance)// int* timeTripCounts, int maxTrav, int maxTravCongested,  double dwLookup [][288], double **zoneSharesL, double **zoneSharesS, double* maxCarUse, double* maxCarOcc, int& totDist, int& unoccDist,
// 		  int& waitT, bool reportProcs, int saveRate, bool warmStart, bool lastWarm, long& maxAvailCars, bool& readFile, int startIter, int& unservedT, 
//		  int* waitCount,  int& hotStarts, int& coldStarts, int nRuns, int iter, bool checkStationDistance)
{
    Car nCar;
    int cn;
    int newCarCt = 0;
    int tempMinUnused;
    int tempCarOcc;
    int actCarNum;
    int passCarNum;
    int saveCtr = saveRate;
    int t, startT, trav, carCt, totNumCars;
    vector<Trip> waitList[6];
    vector<Trip> mergedTripList;
//    Trip waitList [WaitListSize][6]; // wait list
    int waitListI[6]; // wait list index

    int nw, ne, sw, se;


    int zoneGen[numZonesL][numZonesL]; //xmas, ymax
    int waitZones[numZonesL][numZonesL]; //xmax,ymax
    int **waitZonesTL = new int*[numZonesL];//[numZonesL];
    int waitZonesTS[numZonesS][numZonesS];
    double **netZoneBalance = new double*[numZonesL];
    int tripO[numZonesL][numZonesL];
    int tripD[numZonesL][numZonesL];
    int cardDirectLZ[numZonesL];
    int cardDirect[numZonesL];
    int cardDirect2[numZonesL];

    for(int x=0; x<numZonesL; x++){

	netZoneBalance[x] = new double [numZonesL];
	waitZonesTL[x] = new int [numZonesL];
    }

    int xRandOrd[xMax];
    int yRandOrd[yMax];

    int trackX = -1;
    int trackY = -1;
    int trackC = -1;
    bool found = false;

    nw = 0;
    ne = 0;
    sw = 0;
    se = 0;

    unservedT = 0;

    for (int i = 0; i < 6; i++)
    {
        waitCount[i] = 0;
    }

    for (int w = 0; w < 6; w++)
    {
        waitListI[w] = 0;
    }
    
    for (int x = 0; x < numZonesL; x++)
    {
        for (int y = 0; y < numZonesL; y++)
        {
            zoneGen[x][y] = 0;
            waitZones[x][y] = 0;
            netZoneBalance[x][y] = 0;
            tripO[x][y] = 0;
            tripD[x][y] = 0;
        }
        cardDirectLZ[x] = 0;
        cardDirect[x] = 0;
        cardDirect2[x] = 0;
    }


// if loading from previous run start at the last time cycle iteration completed, else start at time = 0
    if (readFile)
    {
        startT = startIter;
        readFile = false;
    } else {
        startT = 0;
    }



    // Init blocks for pricing
    double tripDemand[numZonesL][numZonesL];
    double carsAvailable[numZonesL][numZonesL];
    double numTrips = 0;
    for (int x=0; x<xMax; x++){
      for (int y =0; y<yMax; y++){
        tripDemand[x/zoneSizeL][y/zoneSizeL] += getRate(x,y,-1,xMax/2, yMax/2, innerRate, nearRate, outerRate, exurbanRate);
      }
    }

    for (int x=0; x<numZonesL; x++)
      for (int y=0; y<numZonesL; y++)
        carsAvailable[x][y] = 0;

    prev_avail_cars = 0;
    for (int x = 0; x < xMax; x++) {
      for ( int y = 0; y < yMax; y++) {
        for (int c=0; c<CarMx[x][y].size(); c++) {
          if (!CarMx[x][y][c].inUse) {
            carsAvailable[x / zoneSizeL][y / zoneSizeL]++;
	    prev_avail_cars++;
	  }
        }
      }
    }


    for (t = 0; t<288; t++)
      numTrips += TTMx[t].size();


    for (t = startT; t < 288; t++)
    {
//	if (!warmStart)
//	  cout << "Time of day: "<<t<<" " <<endl;	
        carCt = 0;
        for (int xc = 0; xc < xMax; xc++)
        {
            for (int yc = 0; yc < yMax; yc++)
            {
                carCt = carCt + CarMx[xc][yc].size();//numCars[xc][yc];
            }
        }
//	cout << "Num Cars: " << carCt << endl;
 /*       if (nRuns == 1)
        {
            cout << "T = " << t << ", trips = " << timeTripCounts[t] << ", numCars = " << carCt << ", Delay = ";
            for (int w = 0; w < 6; w++)
            {
                cout << waitListI[w] << ",";
            }
            cout << endl;
        } */

        // set vehicle to track
        if (t == 0 && !warmStart)
        {
            trackX = 17;
            trackY = 23;
            trackC = 0;

            for (int x = trackX; x < xMax && !found; x++)
            {
                if (CarMx[x][trackY].size() > 0)
                {
                    found = true;
                }
            }

            reportProcs = false;
        }

// save current status, if selected
        if (saveRate > 0)
        {
            saveCtr--;
            if (saveCtr == 0)
            {
                saveCtr = saveRate;
                saveStatus( timeTripCounts,  maxTrav,  maxCarUse, totDist, unoccDist, reportProcs, saveRate, warmStart, t);
            }
        }

        // assume congested speeds between 7-8 AM and 4-6:30 PM
        if (((t >= 84) && (t <= 96)) || ((t >= 192) && (t <= 222)))
        {
            trav = maxTravCongested;
        } else {
            trav = maxTrav;
        }
	
// determine number of cars currently active & currently waiting
        if (reportProcs)
        {
            actCarNum = 0;
            passCarNum = 0;

            for (int x = 0; x < xMax; x++)
            {
                for (int y = 0; y < yMax; y++)
                {
                    for (int c = 0; c < CarMx[x][y].size(); c++)
                    {
                        if (CarMx[x][y][c].inUse == true)
                        {
                            actCarNum++;
                        } else {
                            passCarNum++;
                        }
                    }
                }
            }
            cout << actCarNum << " active cars and " << passCarNum << " waiting cars" << endl << endl;
            cout << "Finding cars" << endl;
        }


	// Determine travel mode
	int saev_ct = 0;
	int trans_ct = 0;
	int pv_ct = 0;
	for (int trp = 0; trp < TTMx[t].size(); trp++)
	{
		if (useModeChoice) {
			double tripdemand_b = tripDemand[TTMx[t][trp].startX/zoneSizeL][TTMx[t][trp].startY/zoneSizeL];
			double carsupply_b = carsAvailable[TTMx[t][trp].startX/zoneSizeL][TTMx[t][trp].startY/zoneSizeL];
			getTripTravelMode(&TTMx[t][trp],2.5/60.0,tripdemand_b, numTrips, carsupply_b, (double)prev_avail_cars, warmStart);
		} else {
			TTMx[t][trp].modeOfTransit = 2;
		}
		if (TTMx[t][trp].modeOfTransit == 0)
			trans_ct ++;
		else if (TTMx[t][trp].modeOfTransit == 1)
			pv_ct ++;
		else
			saev_ct ++;
	}
//	if(!warmStart)
//		cout << "Time of day: " << t << "\nSAEV: " << saev_ct << "\nTransit: " << trans_ct << "\nPV: "<< pv_ct << endl;
/*	for (int w = 5; w >=0; w--){
		for (int i=0; i<waitList[w].size(); i++){
			getTripTravelMode(waitList[w][i].waitPtr,(w+1)*5 + 2.5);
			waitList[w][i].modeOfTransit = waitList[w][i].waitPtr->modeOfTransit;
			if ( waitList[w][i].modeOfTransit != waitList[w][i].waitPtr->modeOfTransit)
				cout << "Mode not matching" << endl;
		}
	}
*/
	// MERGE HERE
	if (tripPool == MERGE && !warmStart){

		mergedTripList.clear();

		for ( int w = 5; w >= 0; w--){
			for (int i=0; i<waitList[w].size(); i++)
				mergedTripList.push_back(waitList[w][i]);
		}
		for (int i=0; i<TTMx[t].size(); i++){
			mergedTripList.push_back(TTMx[t][i]);
			mergedTripList[mergedTripList.size() - 1].waitPtr = &TTMx[t][i];
		}

		if (matchAlgorithm == GREEDY)
			matchTripsToCarsGreedy(mergedTripList, t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts, iter, checkStationDistance,warmStart);
		else if (matchAlgorithm == HUNGARIAN || matchAlgorithm == SCRAM)
			matchTripsToCarsScram(mergedTripList, t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts);
		else
                        matchTripsToCarsDecentralized(mergedTripList, t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts, iter, checkStationDistance,warmStart);


		// Clear waitList
		for (int w=0; w <6; w++){

			waitList[w].clear();
			waitListI[w] = 0;
		}

		// initialize wait zones
                for (int x = 0; x < numZonesL; x++)
                {
                        for (int y = 0; y < numZonesL; y++)
                        {
                                waitZonesTL[x][y] = 0;
                        }
                }

                for (int x = 0; x < numZonesS; x++)
                {
                        for (int y = 0; y < numZonesS; y++)
                        {
                                waitZonesTS[x][y] = 0;
                    }
                }

                totNumCars = 0;
                for (int x = 0; x < xMax; x++)
                {
                    for (int y = 0; y < yMax; y++)
                        {
                        totNumCars = totNumCars + CarMx[x][y].size();
                        }
                }

		// Add Trips to waitList
		for (int i=0; i < mergedTripList.size(); i++)
		{
			if (mergedTripList[i].carlink == false)
			{
				mergedTripList[i].waitTime += 5;
				if (t - mergedTripList[i].startTime < 6){
					waitList[t - mergedTripList[i].startTime].push_back(mergedTripList[i]);
					waitListI[t - mergedTripList[i].startTime]++;
					
					   // move car to the next wait time interval
	                                //waitList[w][i].waitTime = waitList[w][i].waitTime + 5;
        	                        waitZonesTL[mergedTripList[i].startX / zoneSizeL][mergedTripList[i].startY / zoneSizeL]++;
                	                waitZonesTS[mergedTripList[i].startX / zoneSizeS][mergedTripList[i].startY / zoneSizeS]++;

				}
				else
					unservedT++;
			}
		}

	} else { // IF WE ARE NOT MERGING WAITLIST	
//		cout << "matching wait lists: "<< TTMx[t].size() << endl;

        	for (int w = 5; w>= 0; w--)
        	{
			if (matchAlgorithm == GREEDY || warmStart)
	                	matchTripsToCarsGreedy(waitList[w], t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts, iter, checkStationDistance, warmStart);
			else if (matchAlgorithm == DECENTRALIZED)
                                matchTripsToCarsDecentralized(waitList[w], t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts, iter, checkStationDistance, warmStart);
                	else
				matchTripsToCarsScram(waitList[w], t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts);
	        }
//		cout << "matched" << endl;
		// initialize wait zones
        	for (int x = 0; x < numZonesL; x++)
        	{
            		for (int y = 0; y < numZonesL; y++)
            		{
                		waitZonesTL[x][y] = 0;
            		}
        	}

        	for (int x = 0; x < numZonesS; x++)
        	{
            		for (int y = 0; y < numZonesS; y++)
            		{
	                	waitZonesTS[x][y] = 0;
        	    }
        	}

	        totNumCars = 0;
        	for (int x = 0; x < xMax; x++)
	        {
        	    for (int y = 0; y < yMax; y++)
            		{
                	totNumCars = totNumCars + CarMx[x][y].size();
            		}
        	}

		// for all unserved waitlisted trips, move them to the next waitlist period.  Those waiting longer than 30 minutes are unserved.
//		cout << " Moving waitlists around"<<endl;
	        for (int w = 5; w >= 0; w--)
        	{
           		waitCount[w] = waitCount[w] + waitListI[w];

	            for (int i = 0; i < waitListI[w]; i++)
        	    {
                	if (waitList[w][i].carlink == false && waitList[w][i].modeOfTransit == 2)
	                {

//                    if (warmStart && totNumCars < 1500)
	                    if (((warmStart && !lastWarm) || (lastWarm && totNumCars < maxAvailCars)) && totNumCars < fleetSizeLimit)
        	            {
                        // generate a new car
                	        newCarCt ++;
                        	nCar = genNewCar (waitList[w][i]);
//				if (nCar.gas < abs(waitList[w][i].endX - nCar.x) + abs(waitList[w][i].endY - nCar.y))
//					cout << "new car not enough" << endl;	
//				if (waitList[w][i].waitPtr != NULL)
				//waitList[w][i].waitPtr->carlink = true;
//				if(t==4)
//					cout << "hey"<<endl;
	                        cn = CarMx[nCar.x][nCar.y].size();
        	                CarMx [nCar.x][nCar.y].push_back(nCar);
				CarMx[nCar.x][nCar.y][cn].currTrip = waitList[w][i].waitPtr;				
                	        zoneGen[nCar.x / zoneSizeL][nCar.y / zoneSizeL]++;

                        	totNumCars++;
                    	     }

	                    else if (w == 5)
        	            {
                	        unservedT++;
	                    } else {
        	                // move car to the next wait time interval
		                if (useModeChoice) {
		                        double tripdemand_b = tripDemand[waitList[w][i].startX/zoneSizeL][waitList[w][i].startY/zoneSizeL];
                		        double carsupply_b = carsAvailable[waitList[w][i].startX/zoneSizeL][waitList[w][i].startY/zoneSizeL];
		                        getTripTravelMode(waitList[w][i].waitPtr,((w+2)*5 + 2.5)/60.0,tripdemand_b, numTrips, carsupply_b, (double)prev_avail_cars, warmStart);
                		} else {
		                        waitList[w][i].waitPtr->modeOfTransit = 2;
                		}

				waitList[w][i].modeOfTransit = waitList[w][i].waitPtr->modeOfTransit;
				if (waitList[w][i].modeOfTransit == 2)
				{
	                	        waitList[w][i].waitTime = waitList[w][i].waitTime + 5;
        	                	waitZonesTL[waitList[w][i].startX / zoneSizeL][waitList[w][i].startY / zoneSizeL]++;
	        	                waitZonesTS[waitList[w][i].startX / zoneSizeS][waitList[w][i].startY / zoneSizeS]++;
        	                //waitList[waitListI[w+1]][w+1] = waitList[i][w];
                		        waitList[w+1].push_back(waitList[w][i]);
					waitListI[w+1]++;
				}
                    	    }

                	  }
            		}
	            waitListI[w] = 0;
		    waitList[w].clear();
		  }
	
		if (matchAlgorithm == GREEDY || warmStart)
			matchTripsToCarsGreedy(TTMx[t], t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts, iter, checkStationDistance, warmStart);
		else
			matchTripsToCarsScram(TTMx[t], t, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts);
	
	
// for each trip that hasn't found a car, create a new one if in warm start or put on the wait list if running normal run
        for (int trp = 0; trp < timeTripCounts[t]; trp++)
        {
            if (TTMx[t][trp].carlink == false)// && TTMx[t][trp].modeOfTransit == 2)
            {
                /*if (warmStart)
                {
                    if (reportProcs)
                    {
                        cout << "New Car for trip " << trp << " " << TTMx[trp][t].startX << "," << TTMx[trp][t].startY << endl;
                    }
                    newCarCt ++;
                    nCar = genNewCar (TTMx[t][trp]);
                    CarMx[nCar.x][nCar.y].push_back(nCar);
		    //cn = numCars [nCar.x][nCar.y] ++;
                    //CarMx [cn][nCar.x][nCar.y] = nCar;

                    zoneGen[nCar.x / zoneSizeL][nCar.y / zoneSizeL]++;
		
                } else {*/ 
                  if (reportProcs)
                    {
                        cout << "Time " << waitList[waitListI[0]][0].startTime << " start " << waitList[waitListI[0]][0].startX << "," << waitList[waitListI[0]][0].startY << endl;
                    }


	                if (useModeChoice) {
        	                double tripdemand_b = tripDemand[TTMx[t][trp].startX/zoneSizeL][TTMx[t][trp].startY/zoneSizeL];
                	        double carsupply_b = carsAvailable[TTMx[t][trp].startX/zoneSizeL][TTMx[t][trp].startY/zoneSizeL];
                        	getTripTravelMode(&TTMx[t][trp],7.5/60.0,tripdemand_b, numTrips, carsupply_b, (double)prev_avail_cars, warmStart);
	                } else {
        	                TTMx[t][trp].modeOfTransit = 2;
                	}

			if (TTMx[t][trp].modeOfTransit == 2){
                                waitZonesTL[TTMx[t][trp].startX / zoneSizeL][TTMx[t][trp].startY / zoneSizeL]++;
	                        waitZonesTS[TTMx[t][trp].startX / zoneSizeS][TTMx[t][trp].startY / zoneSizeS]++;
		    		TTMx[t][trp].waitTime = 5;
				waitList[0].push_back(TTMx[t][trp]);
	        		waitList[0][waitListI[0]].waitPtr = &TTMx[t][trp];
	                	waitListI[0]++;
			}
         	}
	     
           }
	} // End SEPARATE part here

	// Assign unused cars that rejected trips
	for (int x=0; x<xMax; x++){
		for (int y=0; y<yMax; y++){
			for (int c=0; c<CarMx[x][y].size(); c++){
				if (CarMx[x][y][c].numRejects > numRejectsToRefuel && !CarMx[x][y][c].inUse && CarMx[x][y][c].gas < (rangePercentToRefuel * carRange))
				{
//                                        CarMx[x][y][c].refuel = refuelTime;
                                        CarMx[x][y][c].destX = x;
                                        CarMx[x][y][c].destY = y;
                                        CarMx[x][y][c].pickupX = -1;
                                        CarMx[x][y][c].pickupY = -1;
                                        CarMx[x][y][c].currTrip = NULL;
//                                        if (CHARGE_IN_PLACE)
//                                                cellChargeCount[x][y][r] ++;
                                        CarMx[x][y][c].inUse = true;
					CarMx[x][y][c].needFuel = true;

				}
			}
		}
	}


        // update cumulative waitZones balance with single iteration waitZonesT
        for (int x = 0; x < numZonesL; x++)
        {
            for (int y = 0; y < numZonesL; y++)
            {
                waitZones[x][y] = waitZones[x][y] + waitZonesTL[x][y];
            }
        }

        if (reportProcs)
//        if (waitListI[0] > 0 && !warmStart)
        {
            // print the matrix of waiting passengers who have not been served
            showWaitCars(t, waitList, waitListI);
        }


//        if (!warmStart)
//        {
//            reportProcs = false;
//        }
	if (!canRefuelAnywhere){
//		cout << "Assign refuel" << endl;
		// Assign cars that need to refuel to a charging station
		for (int x =0; x<xMax; x++)
		{
			for (int y=0; y<yMax; y++)
			{
				for (int c =0; c<CarMx[x][y].size(); c++)
				{
					if(CarMx[x][y][c].needFuel){//refuel > 0){
						if (carSpeed == VARY)
							trav = getCarTrav(x,y,t);

						for (int d=0; d<trav && CarMx[x][y][c].stationLink == false; d++)
							findNearestStation(&CarMx[x][y][c],d,trav); // This function needs to assign station to car

						if (warmStart && (CarMx[x][y][c].stationLink == false) && !checkStationDistance){
								//cout << "generating station"<<endl;
								genNewStation(CarMx[x][y][c]);
								CarMx[x][y][c].destX = x;
								CarMx[x][y][c].destY = y;
								CarMx[x][y][c].pickupX = -1;
								CarMx[x][y][c].pickupY = -1;
								CarMx[x][y][c].stationLink = true;
						}
						if(!warmStart && CarMx[x][y][c].stationLink == false)
							CarMx[x][y][c].stuck = true;
					}
				}
			}
		}
	}


        int inUse = 0;
	int charging = 0;
	int unused = 0;
	int inUse_max_charge = 0;
	int charging_max_charge = 0;
	int unused_max_charge = 0;
        for (int x=0; x<xMax; x++){
                for(int y=0; y<yMax; y++){
                        for( int c=0; c<CarMx[x][y].size(); c++){

                                if (CarMx[x][y][c].inUse && !CarMx[x][y][c].needFuel){//refuel == 0){
					inUse ++;
					inUse_max_charge ++;
				}
				if (CarMx[x][y][c].inUse && CarMx[x][y][c].needFuel){//refuel > 0){
					charging++;
					charging_max_charge++;
				}
				if (CarMx[x][y][c].inUse == false){
					unused++;
					unused_max_charge++;
				}
                        }
                }
        }

	if (inUse > maxCarInUse){
		maxCarInUse = inUse;
		maxCarCharging = charging;
		maxCarNoUse = unused;
	}
	if (charging_max_charge > maxChargingCharge){
		maxInUseCharge = inUse_max_charge;
		maxChargingCharge = charging_max_charge;
		maxNoUseCharge = unused_max_charge;
	}

        //prev_avail_cars = unused;

//	if (charging > maxCarCharging){ maxCarCharging = charging;}
//	if (unused > maxCarNoUse){ maxCarNoUse = unused;}

//	cout << "move cars" << endl;
	// move all cars that are in use
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].inUse && !CarMx[x][y][c].moved)
                    {
			if (carSpeed == VARY)
				trav = getCarTrav(x,y,t);
			moveCar (x, y, c, t, trav, totDist, unoccDist, waitT, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                 trackX, trackY, trackC,iter);
                        c--;
                    }
                }
            }
        }
	
        tempCarOcc = 0;

// update max # of occupied cars
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
		    if (canRefuelAnywhere){
	            	if (CarMx[x][y][c].moved == false)
        	        {
                	        tempCarOcc++;
                    	}
		    }
		    else{
		    	if (CarMx[x][y][c].moved == false && CarMx[x][y][c].needFuel){
				tempCarOcc++;
			}
		    }
                }
            }
        }


// Should .moved be true for occupied cars?

        maxCarOcc[t] = maxCarOcc[t] + tempCarOcc;

		

//        if (tempCarOcc < maxCarOcc)
//        {
//            maxCarOcc = tempCarOcc;

        


// **************** vehicle reallocation portion ****************************************************************

// reallocate vehicles based on zones

//  cout << "realloc" << endl;
	if (reallocationOn){

          reallocVehsLZones (timeTripCounts, dwLookup, zoneSharesL, t, reportProcs, totDist, unoccDist, hotStarts, coldStarts, trav,
                           netZoneBalance, cardDirectLZ, waitZonesTL, numZonesL, zoneSizeL, trackX, trackY, trackC,iter);

          randOrdering(xRandOrd, xMax);
          randOrdering(yRandOrd, yMax);

          // shift vehicles two spaces, if current space has 3 or my vehicles and nearby space is unoccupied with no neighbors
          for (int x = 0; x < xMax; x++)
          {
            for (int y = 0; y < yMax; y++)
            {
                reallocVehs2(xRandOrd[x], yRandOrd[y],  timeTripCounts, dwLookup, reportProcs, totDist, unoccDist, hotStarts, coldStarts,
                             cardDirect2, trackX, trackY, trackC, iter);
            }
          }

          randOrdering(xRandOrd, xMax);
          randOrdering(yRandOrd, yMax);

          // shift vehicles one space if difference between current space and new space is more than 2
          for (int x = 0; x < xMax; x++)
          {
              for (int y = 0; y < yMax; y++)
              {
                  reallocVehs(xRandOrd[x], yRandOrd[y],  timeTripCounts, dwLookup, reportProcs, totDist, unoccDist, hotStarts, coldStarts, cardDirect, trackX, trackY, trackC, iter);
              }
          }

	}

        tempMinUnused = 0;

// reset movement for all cars
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].moved == false)
                    {
                        tempMinUnused++;
                        CarMx[x][y][c].tElapsed++;
                    } else {
                        CarMx[x][y][c].moved = false;
                    }
                }
            }
        }
	//cout << "refuel cars" << endl;
// refuel all cars that are low on fuel
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {

		int cellCt = 0;
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].needFuel)//CarMx[x][y][c].refuel > 0) // need to refuel and at a station
                    {
			if (canRefuelAnywhere){

                        	CarMx[x][y][c].refuel--;
	                        CarMx[x][y][c].gas = carRange;
				cellChargeTime[x][y][iter] ++;
                	        if (CarMx[x][y][c].refuel <= 0) // could possibly be zero
	                        {
        	                    CarMx[x][y][c].inUse = false;
				    CarMx[x][y][c].numRejects = 0;
				    CarMx[x][y][c].stationLink = false;
				    CarMx[x][y][c].needFuel = false;
                        	}
	                } else if (!canRefuelAnywhere && ChStMx[x][y].size() > 0){
				cellCt++;
				ChStMx[x][y][0].chargeTime++;
				CarMx[x][y][c].refuel--;
				CarMx[x][y][c].gas = carRange;
				cellChargeTime[x][y][iter] ++;
				if (CarMx[x][y][c].refuel <= 0) // could possibly be zero when starting here
				{
					//cout << "Car done charging" << endl;
					CarMx[x][y][c].inUse = false;
					CarMx[x][y][c].numRejects = 0;
					CarMx[x][y][c].stationLink = false;
					CarMx[x][y][c].needFuel = false;
				}
		        }
		    }
		  }
                
		if (!canRefuelAnywhere && cellCt > 0)
			if (cellCt > ChStMx[x][y][0].congestTime)
				ChStMx[x][y][0].congestTime = cellCt;	
            }
        }

	for (int x=0; x<numZonesL; x++)
	  for(int y=0; y<numZonesL; y++)
	    carsAvailable[x][y] = 0;

	prev_avail_cars = 0;
        for (int x = 0; x < xMax; x++) {
          for ( int y = 0; y < yMax; y++) {
            for (int c=0; c<CarMx[x][y].size(); c++) {
              if (!CarMx[x][y][c].inUse){
                carsAvailable[x / zoneSizeL][y / zoneSizeL]++;
		prev_avail_cars++;
	      }
            }
          }
        }


        maxCarUse[t] = maxCarUse[t] + tempMinUnused;
//        if (tempMinUnused < maxCarUse)
//        {
//            maxCarUse = tempMinUnused;
//        }
/*	if (t == 200 && !warmStart && (iter % 10 == 0 || iter ==1)){
		cout << "Writing heat map" << endl;
		writeHeatMap(iter,t);
	}
*/
    } // END MAIN LOOP

    //runSummary(warmStart, lastWarm, zoneGen, waitZones, netZoneBalance, tripO, tripD, cardDirectLZ, cardDirect, cardDirect2,  timeTripCounts);

    if (warmStart && !lastWarm)
    {
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                maxAvailCars = maxAvailCars + CarMx[x][y].size();
            }
        }
    }
	maxAvailStations = 0;
	if (warmStart && !lastWarm && !canRefuelAnywhere){
		for (int x = 0; x < xMax; x++){
			for (int y=0; y<yMax; y++){
				maxAvailStations = maxAvailStations + ChStMx[x][y].size();
			}
		}
	}

	// Go through waitList and increment unserved for trips still on it, also these trips do not count against waitTime
	for(int i=0; i<6; i++){
		for (int j=0; j<waitList[i].size(); j++)
		{
			if (waitList[i][j].carlink == false){ // unserved
				waitList[i][j].waitPtr->waitTime = 0;
				unservedT++;
				if (waitList[i][j].modeOfTransit != 2)
					cout << "Should have to choose saev to be here" << endl;
			}
		}
	}

	// Calculate true wait times
	for(int i=0; i<6; i++)
		waitCount[i]=0;
	int index;
	for (int t=0; t<288; t++){
//		cout << TTMx[t].size() << endl;
		for(int trp = 0; trp<TTMx[t].size(); trp++){
			index = ((int)TTMx[t][trp].waitTime)/5 - 1;
			if (index >= 0)
				waitCount[index]++;
		}
	}
	waitCount[4] += waitCount[5];
	waitCount[3] += waitCount[4];
	waitCount[2] += waitCount[3];
	waitCount[1] += waitCount[2];
	waitCount[0] += waitCount[1];
	if (!warmStart)
		writeTripsToFile();
    return;

}

// resets all variables for the daily run, now that we know how many cars there will be
void Simulator::placeInitCars ()//int* timeTripCounts, double* maxCarUse, double* maxCarOcc, int& totDist,
//                    int& unoccDist, int& waitT, double dwLookup [][288], bool reportProcs, int& hotStarts, int& coldStarts)
{
//    int dx, dy;


//    maxCarUse = 20000;
//    maxCarOcc = 20000;
    totDist = 0;
    unoccDist = 0;
    reallocDist = 0;
    waitT = 0;

/*
    // return all cars to their initial position
    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            for (int c = 0; c < numCars[x][y]; c++)
            {
                dx = CarMx[c][x][y].startX;
                dy = CarMx[c][x][y].startY;
                CarMx[c][x][y].returnHome = false;
                move(CarMx, numCars, x, y, dx, dy, c, 0, dwLookup, TTMx, timeTripCounts, reportProcs, hotStarts, coldStarts);
            }
        }
    }

*/

    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {

		if (!CarMx[x][y][c].inUse){ // cars that aren't assigned or charging
	                CarMx[x][y][c].inUse = false;
	                CarMx[x][y][c].pickupX = -1;
        	        CarMx[x][y][c].pickupY = -1;
	                CarMx[x][y][c].destX = CarMx[x][y][c].x;
        	        CarMx[x][y][c].destY = CarMx[x][y][c].y;
                	CarMx[x][y][c].returnHome = false;
	                CarMx[x][y][c].tripCt = 0;
			CarMx[x][y][c].numRejects = 0;
			CarMx[x][y][c].currTrip = NULL;
			CarMx[x][y][c].stationLink = false;
		}else{
			CarMx[x][y][c].tripCt=0;
		}	
			CarMx[x][y][c].stuck = false;
//		if (RUN == RESET)
//		{
//			CarMx[x][y][c].gas = carRange;
//			CarMx[x][y][c].refuel = 0;
//		}
            }
        }
    }

    // all trips need to be serviced for next iteration
    for (int t = 0; t < 288; t++)
    {
        TTMx[t].clear();
	timeTripCounts[t] = 0;
    }


    // reset vehicle starts
    hotStarts = 0;
    coldStarts = 0;

    if (reportProcs)
    {
        cout << "Beginning next iteration" << endl << endl;
    }

    return;
}

//
void Simulator::reportResults (int runNum)//int* timeTripCounts, double* maxCarUse, double* maxCarOcc, int totDist, int unoccDist, int waitT, int unservedT, int* waitCount, int hotStarts, int coldStarts, long& totDistRun, long& totUnoccDistRun, long& totCarsRun, long& totTripsRun, long& totHSRun, long& totCSRun, long& totWaitTRun, long& totUnservedTRun, long* totWaitCountRun, int totUnusedRun, int totUnoccRun, double& totAvgWait, double totAvgTripDist, int nRuns, int runNum)
{
    int numCars = 0;
    int nTrips = 0;
    int nStations = 0;
    int maxTripGen = 0;
    char dummyStr[20];

    double waitCOV = 0;
    double tripDistCOV = 0;
    double carTripsCOV = 0;
    double avgWait = 0;
    double avgDist = 0;
    double avgTrips = 0;

    double totWaitCount = 0;
    double maxCarUseT = 20000;
    double maxCarOccT = 20000;

    int servedT = 0;

/*    int totR = 0;
    int totU = 0;
    double avgsx = 0;
    double avgsy = 0;
    int rmov = 0;
    int umov = 0;
*/



    // determine num cars
    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            numCars = numCars + CarMx[x][y].size();
	    nStations = nStations + ChStMx[x][y].size();
        }
    }

/*

    int tdist[36];
    int disttrav = 0;
    for (int i = 0; i < 36; i++)
    {
        tdist[i] = 0;
    }


*/

/*    for (int j = 0; j < 36; j++)
    {
        cout << j + 1 << " " << tdist[j] << endl;
    }



    for (int tm = 0; tm < 288; tm++)
    {
        for (int trp = 0; trp < timeTripCounts[tm]; trp++)
        {
            totR = totR + TTMx[trp][tm].endX - TTMx[trp][tm].startX;
            totU = totU + TTMx[trp][tm].endY - TTMx[trp][tm].startY;
            avgsx = avgsx + TTMx[trp][tm].startX;
            avgsy = avgsy + TTMx[trp][tm].startY;
            if (TTMx[trp][tm].endX > TTMx[trp][tm].startX)
            {
                rmov = rmov + 1;
            } else if (TTMx[trp][tm].endX < TTMx[trp][tm].startX) {
                rmov = rmov - 1;
            }
            if (TTMx[trp][tm].endY > TTMx[trp][tm].startY)
            {
                umov = umov + 1;
            } else if (TTMx[trp][tm].endY < TTMx[trp][tm].startY) {
                umov = umov - 1;
            }
        }
    }

    avgsx = avgsx / nTrips;
    avgsy = avgsy / nTrips;

    cout << endl << "TotR: " << totR << " TotU: " << totU << " avgsx: " << avgsx << " avgsy: " << avgsy << " rmov: " << rmov << " umov: " << umov << endl << endl;
*/

    // get coefficient of variation
    for (int t = 0; t < 288; t++)
    {

	nTrips = nTrips + TTMx[t].size();
	
	if (TTMx[t].size() > maxTripGen)
		maxTripGen = TTMx[t].size();

        for (int trp = 0; trp < TTMx[t].size(); trp++)
        {
            if (TTMx[t][trp].carlink){
	      avgWait = avgWait + TTMx[t][trp].waitTime;
	      servedT++;
              avgDist = avgDist + TTMx[t][trp].tripDist;
	    }
	    if (TTMx[t][trp].modeOfTransit != 2)
	    {
		double r = rand() / RAND_MAX;
		if (r > 0.22)
			nTrips++;

	    }
        }
    }


    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                avgTrips = avgTrips + CarMx[x][y][c].tripCt;
            }
        }
    }

    // Get Charge Stats
    double chargeDist=0.0;
    double chargeTime=0.0;
    double chargeCount=0.0;
    double congestTime=0.0;
    for (int x=0; x<xMax; x++){
	for(int y=0; y<yMax; y++){
		if(ChStMx[x][y].size() > 0){
			chargeDist += ChStMx[x][y][0].unoccupiedDist;
			chargeTime += ChStMx[x][y][0].chargeTime;
			chargeCount += ChStMx[x][y][0].numCharges;
			congestTime += ChStMx[x][y][0].congestTime;
		}
	}
     }


    avgWait = avgWait / servedT;//nTrips;
    avgDist = avgDist / servedT;//nTrips;
    avgTrips = avgTrips / numCars;

    for (int t = 0; t < 288; t++)
    {
        for (int trp = 0; trp < timeTripCounts[t]; trp++)
        {
            waitCOV = waitCOV + pow(TTMx[t][trp].waitTime - avgWait, 2);
            tripDistCOV = tripDistCOV + pow(TTMx[t][trp].tripDist - avgDist, 2);
        }
    }
    int numStuck = 0;
    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                carTripsCOV = carTripsCOV + pow(CarMx[x][y][c].tripCt - avgDist, 2);
		if (CarMx[x][y][c].stuck)
			numStuck++;
            }
        }
    }

    // output the number of cars in use and number of cars occupied by time of day to file, then determine the highest use times
    writeMaxCarUse(maxCarUse, maxCarOcc);

    for (int t = 0; t < 288; t++)
    {
        if (maxCarOccT > maxCarOcc[t])
        {
            maxCarOccT = maxCarOcc[t];
        }
        if (maxCarUseT > maxCarUse[t])
        {
            maxCarUseT = maxCarUse[t];
        }
    }


    // coefficient of variation, std. deviation divided by the mean
    waitCOV = sqrt(waitCOV / servedT) / avgWait;
    tripDistCOV = sqrt(tripDistCOV / servedT) / avgDist;
    carTripsCOV = sqrt(carTripsCOV / nTrips) / avgTrips;

    cout << "Total number of trips " << nTrips << endl;
    cout << "Total number of served trips "<<servedT<<endl;
    cout << "Total number of unserved trips " << unservedT << endl;
    cout << "Total number of cars " << numCars << endl;
    cout << "Total stuck cars at end of run " << numStuck << endl;
    cout << "Total number of stations "<< nStations << endl;
//    cout << "5-minute traveler wait time intervals elapsed " << waitCount << endl;
//    cout << "Total number of trips reassigned "<< tripsReassigned << endl;
    cout << "Average wait time " << avgWait << endl;
    cout << "Total miles traveled " << totDist * cellSize << endl;
    cout << "Total unoccupied miles traveled " << unoccDist * cellSize << endl;
    cout << "Total reallocation miles traveled " << reallocDist * cellSize << endl;
    cout << "Total unoccupied miles to charge " << chargeDist * cellSize << endl;
    cout << "Total Charges "<< chargeCount << endl;
    cout << "Total Charge Time " << chargeTime << endl;
    cout << "Average Congestion Time per Station " << congestTime / nStations << endl;
    cout << "Average trip miles " << avgDist << endl;
//    cout << "Maximum number of trips starting during any 5 minute period " << maxTripGen << endl;
    cout << "Maximum number of cars in use during any 5 minute period " << maxCarUseT << endl;
    cout << "Maximum number of cars occupied during any 5 minute period " << maxCarOccT << endl;
    cout << "SAEV with charging station:" << endl;
    cout << "Maximum number of cars in use during max use 5 minute period " << maxCarInUse << endl;
    cout << "Maximum number of cars charging in max use 5 minute period " << maxCarCharging << endl;
    cout << "Maximum number of cars unused or reallocated in max use 5 minute period " << maxCarNoUse<< endl;
    cout << "Maximum number of cars in use during max charging 5 minute period " << maxInUseCharge << endl;
    cout << "Maximum number of cars charging in max charging 5 minute period " << maxChargingCharge << endl;
    cout << "Maximum number of cars unused or reallocated in max charging 5 minute period " << maxNoUseCharge<< endl;
    cout << "Total Revenue " << revenue << endl;
    cout << "Total # of hot starts " << hotStarts << endl;
    cout << "Total # of cold starts " << coldStarts << endl << endl;

    for (int i = 0; i < 6; i++)
    {
        cout << "Wait " << (i+1) * 5 << " " << waitCount[i] << endl;
        totWaitCount = totWaitCount + waitCount[i];
    }


    cout << "Wait time COV: " << waitCOV << endl;
    cout << "Trip Distance COV: " << tripDistCOV << endl;
    cout << "Trips per car COV: " << carTripsCOV << endl << endl;

    if (numRuns == 1)
    {
        cout << "Press any key and enter to exit." << endl;
    //    cin >> dummyStr;
    }
    totRevenueRun += revenue;
    totTripsRun = totTripsRun + nTrips;
    totUnservedTRun = totUnservedTRun + unservedT;
    totCarsRun = totCarsRun + numCars;
    totWaitTRun = totWaitTRun + waitT;
    totHSRun =  totHSRun + hotStarts;
    totCSRun = totCSRun + coldStarts;
    totDistRun = totDistRun + totDist;
    totUnoccDistRun = totUnoccDistRun + unoccDist;
    totReallocDistRun += reallocDist;
    totUnoccChargeDistRun = totUnoccChargeDistRun + chargeDist;
    totChargeTimeRun += chargeTime;
    totCongestTimeRun += congestTime;
    totNumChargeRun += chargeCount;
    totUnusedRun = totUnusedRun + maxCarUseT;
    totUnoccRun = totUnoccRun + maxCarOccT;
    totAvgWait = totAvgWait + avgWait;
    totAvgTripDist = totAvgTripDist + avgDist;
    totWaitCOV = totWaitCOV + waitCOV;
    totTripDistCOV = totTripDistCOV + tripDistCOV;
    totCarTripsCOV = totCarTripsCOV + carTripsCOV;

    totMaxCarInUse = totMaxCarInUse + maxCarInUse;
    totMaxCarCharging = totMaxCarCharging + maxCarCharging;
    totMaxCarNoUse = totMaxCarNoUse + maxCarNoUse;
    totMaxInUseCharge += maxInUseCharge;
    totMaxChargingCharge += maxChargingCharge;
    totMaxNoUseCharge += maxNoUseCharge;

    for (int i = 0; i < 6; i++)
    {
        totWaitCountRun[i] = totWaitCountRun[i] + waitCount[i];
    }

    totMaxCarInUseCOV[runNum-1] = maxCarInUse;
    totMaxCarChargingCOV[runNum-1] = maxCarCharging;
    totMaxCarNoUseCOV[runNum-1] = maxCarNoUse;

    totMaxInUseChargeCOV[runNum-1] = maxInUseCharge;
    totMaxChargingChargeCOV[runNum-1] = maxChargingCharge;
    totMaxNoUseChargeCOV[runNum-1] = maxNoUseCharge;

    totRevenueCOV [runNum - 1] = revenue;

    totDistRunCOV[runNum-1] = totDist;
    totUnoccDistRunCOV[runNum-1] = unoccDist;
    totReallocDistRunCOV[runNum-1] = reallocDist;
    totUnoccChargeDistRunCOV[runNum-1] = chargeDist;
    totChargeTimeRunCOV[runNum-1] = chargeTime;
    totNumChargeRunCOV[runNum-1] = chargeCount;
    totCongestTimeRunCOV[runNum-1] = congestTime;
    totCarsRunCOV[runNum-1] = numCars;
    totTripsRunCOV[runNum-1] = nTrips;
    totHSRunCOV[runNum-1] = hotStarts;
    totCSRunCOV[runNum-1] = coldStarts;
    totWaitTRunCOV[runNum-1] = waitT;
    totUnservedTRunCOV[runNum-1] = unservedT;
    totWaitCountRunCOV[runNum-1] = waitCount[0] + waitCount[1] + waitCount[2] + waitCount[3] + waitCount[4] + waitCount[5];
    totUnusedRunCOV[runNum-1] = maxCarUseT;
    totUnoccRunCOV[runNum-1] = maxCarOccT;
    totAvgWaitCOV[runNum-1] = avgWait;
    totAvgTripDistCOV[runNum-1] = avgDist;
    totStartsPerTripCOV[runNum-1] = double(hotStarts + coldStarts) / double(nTrips);
    totAvgTripsPerCarCOV[runNum-1] = double(nTrips) / double(numCars);

    totPctMaxWaitFiveCOV[runNum-1] = totWaitCount / double(servedT);
    totPctInducedTCOV[runNum-1] = double (unoccDist) / double(totDist);
    totPctMaxInUseCOV[runNum-1] = 1 - double (maxCarUseT) / double (numCars);
    totPctMaxOccCOV[runNum-1] = 1 - double (maxCarOccT) / double (numCars);
    totPctColdShareCOV[runNum-1] = double (coldStarts) / double (coldStarts + hotStarts);

    return;
}

void Simulator::reportFinalResults (long totDistRun, long totUnoccDistRun, long totCarsRun, long totTripsRun, long totHSRun, long totCSRun, long totWaitTRun,
                         long totUnservedTRun, long* totWaitCountRun, long totUnusedRun, long totUnoccRun, double totAvgWait, double totAvgTripDist,int numRuns){
    char dummyStr[20];
    double totDistRunCOVF, totUnoccDistRunCOVF, totReallocDistRunCOVF, totUnoccChargeDistRunCOVF, totCongestTimeRunCOVF, totCarsRunCOVF, totTripsRunCOVF, totHSRunCOVF, totCSRunCOVF, totWaitTRunCOVF, totUnservedTRunCOVF, totChargeTimeRunCOVF, totNumChargeRunCOVF,
           totWaitCountRunCOVF, totUnusedRunCOVF, totUnoccRunCOVF, totAvgWaitCOVF, totAvgTripDistCOVF, totStartsPerTripCOVF, totAvgTripsPerCarCOVF,
           totPctMaxWaitFiveCOVF, totPctInducedTCOVF, totPctMaxInUseCOVF, totPctMaxOccCOVF, totPctColdShareCOVF, totMaxCarInUseCOVF, totMaxCarChargingCOVF, totMaxCarNoUseCOVF, totMaxInUseChargeCOVF, totMaxNoUseChargeCOVF, totMaxChargingChargeCOVF, totRevenueCOVF;

    findCOV (totRevenueCOVF, totRevenueCOV);
    findCOV (totMaxCarInUseCOVF, totMaxCarInUseCOV);
    findCOV (totMaxCarChargingCOVF, totMaxCarChargingCOV);
    findCOV (totMaxCarNoUseCOVF, totMaxCarNoUseCOV);
    findCOV (totMaxInUseChargeCOVF, totMaxInUseChargeCOV);
    findCOV (totMaxChargingChargeCOVF, totMaxChargingChargeCOV);
    findCOV (totMaxNoUseChargeCOVF, totMaxNoUseChargeCOV);

    findCOV (totDistRunCOVF, totDistRunCOV);
    findCOV (totUnoccDistRunCOVF, totUnoccDistRunCOV);
    findCOV (totReallocDistRunCOVF, totReallocDistRunCOV);
    findCOV(totUnoccChargeDistRunCOVF, totUnoccDistRunCOV);
    findCOV(totCongestTimeRunCOVF, totCongestTimeRunCOV);
    findCOV(totChargeTimeRunCOVF, totChargeTimeRunCOV);
    findCOV(totNumChargeRunCOVF, totNumChargeRunCOV);
    findCOV (totCarsRunCOVF, totCarsRunCOV);
    findCOV (totTripsRunCOVF, totTripsRunCOV);
    findCOV (totHSRunCOVF, totHSRunCOV);
    findCOV (totCSRunCOVF, totCSRunCOV);
    findCOV (totWaitTRunCOVF, totWaitTRunCOV);
    findCOV (totUnservedTRunCOVF, totUnservedTRunCOV);
    findCOV (totWaitCountRunCOVF, totWaitCountRunCOV);
    findCOV (totUnusedRunCOVF, totUnusedRunCOV);
    findCOV (totUnoccRunCOVF, totUnoccRunCOV);
    findCOV (totAvgWaitCOVF, totAvgWaitCOV);
    findCOV (totAvgTripDistCOVF, totAvgTripDistCOV);
    findCOV (totStartsPerTripCOVF, totStartsPerTripCOV);
    findCOV (totAvgTripsPerCarCOVF, totAvgTripsPerCarCOV);

    findCOV (totPctMaxWaitFiveCOVF, totPctMaxWaitFiveCOV);
    findCOV (totPctInducedTCOVF, totPctInducedTCOV);
    findCOV (totPctMaxInUseCOVF, totPctMaxInUseCOV);
    findCOV (totPctMaxOccCOVF, totPctMaxOccCOV);
    findCOV (totPctColdShareCOVF, totPctColdShareCOV);


    cout << endl << endl << "****************************************" << endl << endl;

    totRevenueRun = totRevenueRun / numRuns;
    totTripsRun = totTripsRun / numRuns;
    totCarsRun = totCarsRun / numRuns;
    totWaitTRun = totWaitTRun / numRuns;
    totHSRun =  totHSRun / numRuns;
    totCSRun = totCSRun / numRuns;
    totDistRun = totDistRun / numRuns;
    totUnoccDistRun = totUnoccDistRun / numRuns;
    totUnoccChargeDistRun = totUnoccChargeDistRun / numRuns;
    totReallocDistRun /= (1.0*numRuns);
    totCongestTimeRun = totCongestTimeRun / numRuns;
    totNumChargeRun = totNumChargeRun / numRuns;
    totChargeTimeRun = totChargeTimeRun / numRuns;
    totUnoccRun = totUnoccRun / numRuns;
    totUnusedRun = totUnusedRun / numRuns;
    totAvgWait = totAvgWait / numRuns;
    totAvgTripDist = totAvgTripDist / numRuns;
    totWaitCOV = totWaitCOV / numRuns;
    totTripDistCOV = totTripDistCOV / numRuns;
    totCarTripsCOV = totCarTripsCOV / numRuns;

    totMaxCarInUse /= numRuns;
    totMaxCarCharging /= numRuns;
    totMaxCarNoUse /= numRuns;
    totMaxInUseCharge /= numRuns;
    totMaxChargingCharge /= numRuns;
    totMaxNoUseCharge /= numRuns;

    for (int i = 0; i < 6; i++)
    {
        totWaitCountRun[i] = totWaitCountRun[i] / numRuns;
    }

    cout << "Average number of trips " << totTripsRun << "           COV: " << totTripsRunCOVF << "\tsd: "<<totTripsRunCOVF * totTripsRun<< endl;
    cout << "Total number of unserved trips " << totUnservedTRun << "        COV: " << totUnservedTRunCOVF << "\tsd: "<<totUnservedTRun * totUnservedTRunCOVF<< endl;
    cout << "Average number of cars " << totCarsRun << "             COV: " << totCarsRunCOVF << endl;
//    cout << "5-minute wait intervals elapsed " << totWaitCountRun << "    COV: " << totWaitCountRunCOVF << endl;
    cout << "Average wait time " << totAvgWait << "              COV: " << totAvgWaitCOVF  << "\tsd: "<<totAvgWait * totAvgWaitCOVF<< endl;
//    cout << "Maximum number of trips starting during any 5 minute period " << maxTripGen << endl;
    cout << "Average number of reassigned trips: " << totTripsReassigned / (1.0 * numRuns)<< endl;
    cout << "Average total miles traveled " << totDistRun * cellSize << "     COV: " << totDistRunCOVF << "\tsd: "<<(totDistRun * cellSize)* totDistRunCOVF<< endl;
    cout << "Average total unocc mi traveled  " << totUnoccDistRun * cellSize << "  COV: " << totUnoccDistRunCOVF << "\tsd: "<<(totUnoccDistRun * cellSize) * totUnoccDistRunCOVF<< endl;
    cout << "Average reallocation mi traveled " << totReallocDistRun * cellSize << " COV: " << totReallocDistRunCOVF << endl;
    cout << "Average miles to charge " << totUnoccChargeDistRun * cellSize << "   COV: " << totUnoccChargeDistRunCOVF << endl;
    cout << "Average charge time " << totChargeTimeRun << "   COV: " << totChargeTimeRunCOVF << endl;
    cout << "Average number of charges " << totNumChargeRun << "   COV: " << totNumChargeRunCOVF << endl;
    cout << "Average charge congestion times per run " << totCongestTimeRun << "   COV: " << totCongestTimeRunCOVF << endl;
    cout << "Average trip distance " << totAvgTripDist << "           COV: " << totAvgTripDistCOVF << "\tsd: "<<totAvgTripDist * totAvgTripDistCOVF << endl;
    cout << "Average min number unused cars " << totUnusedRun << "       COV: " << totUnusedRunCOVF << endl;
    cout << "Average min number unoccupied cars " << totUnoccRun << "  COV: " << totUnoccRunCOVF << endl;
    cout << "Average Revenue " << totRevenueRun << "  COV: " << totRevenueCOVF << endl;
    cout << "Average # of hot starts " << totHSRun << "           COV: " << totHSRunCOVF << endl;
    cout << "Average # of cold starts " << totCSRun << "           COV: " << totCSRunCOVF << endl << endl;

    for (int i = 0; i < 6; i++)
    {
        cout << "Wait count " << (i+1) * 5 << " elapsed " << totWaitCountRun[i] << "    COV: " << totWaitCountRunCOVF << endl;
    }

    cout << "Average wait time COV within run: " << totWaitCOV << endl;
    cout << "Average trip distance COV within run: " << totTripDistCOV << endl;
    cout << "Average trips per car COV within run: " << totCarTripsCOV << endl << endl;

    cout << "Average trips per car COV: " << totAvgTripsPerCarCOVF << endl;
    cout << "Average starts per trip COV: " << totStartsPerTripCOVF << endl << endl;

    cout << "% waiting 5 minutes or more COV " << totPctMaxWaitFiveCOVF << endl;
    cout << "% induced travel COV " << totPctInducedTCOVF << endl;
    cout << "% max cars in use COV " << totPctMaxInUseCOVF << endl;
    cout << "% max cars occupied COV " << totPctMaxOccCOVF << endl;
    cout << "% share cold start COV " << totPctColdShareCOVF << endl << endl;

    cout << "For runs with charging stations: "<<endl;
    cout << "Average # of cars in use during max 5 minute period: " << totMaxCarInUse << "\tCOV: " <<totMaxCarInUseCOVF << endl;
    cout << "Average # of cars charging during max 5 minute period: " << totMaxCarCharging << "\tCOV: "<<totMaxCarChargingCOVF << endl;
    cout << "Average # of cars unused or reallocated during max 5 minute period: " <<totMaxCarNoUse << "\tCOV: "<<totMaxCarNoUseCOVF << endl;
    cout << "Average # of cars in use during max charging 5 minute period: " << totMaxInUseCharge << "\tCOV: " <<totMaxInUseChargeCOVF << endl;
    cout << "Average # of cars charging during max charging 5 minute period: " << totMaxChargingCharge << "\tCOV: "<<totMaxChargingChargeCOVF << endl;
    cout << "Average # of cars unused or reallocated during max charging 5 minute period: " <<totMaxNoUseCharge << "\tCOV: "<<totMaxNoUseChargeCOVF << endl;


    cout << "Press any key and enter to exit." << endl;
    //cin >> dummyStr;

    return;
}


//****Functions called from InitVars*************************************************************************************************************

//cumulative distribution of trip start times in 5 minute increments
void Simulator::setStartTimes(double startTimes [288])
{

// NHTS

    startTimes	[	0	]	=	0.000496758	;
    startTimes	[	1	]	=	0.000908696	;
    startTimes	[	2	]	=	0.001191581	;
    startTimes	[	3	]	=	0.001386444	;
    startTimes	[	4	]	=	0.001519484	;
    startTimes	[	5	]	=	0.001728657	;
    startTimes	[	6	]	=	0.002015685	;
    startTimes	[	7	]	=	0.002197047	;
    startTimes	[	8	]	=	0.002313675	;
    startTimes	[	9	]	=	0.002351144	;
    startTimes	[	10	]	=	0.002418503	;
    startTimes	[	11	]	=	0.002507331	;
    startTimes	[	12	]	=	0.002688577	;
    startTimes	[	13	]	=	0.002818928	;
    startTimes	[	14	]	=	0.002898469	;
    startTimes	[	15	]	=	0.002956507	;
    startTimes	[	16	]	=	0.002992458	;
    startTimes	[	17	]	=	0.003060655	;
    startTimes	[	18	]	=	0.003174915	;
    startTimes	[	19	]	=	0.003241152	;
    startTimes	[	20	]	=	0.003286805	;
    startTimes	[	21	]	=	0.003330903	;
    startTimes	[	22	]	=	0.003393218	;
    startTimes	[	23	]	=	0.003483016	;
    startTimes	[	24	]	=	0.003642754	;
    startTimes	[	25	]	=	0.003749291	;
    startTimes	[	26	]	=	0.003801638	;
    startTimes	[	27	]	=	0.00381983	;
    startTimes	[	28	]	=	0.003854756	;
    startTimes	[	29	]	=	0.003865312	;
    startTimes	[	30	]	=	0.003919426	;
    startTimes	[	31	]	=	0.003941839	;
    startTimes	[	32	]	=	0.004008882	;
    startTimes	[	33	]	=	0.004022319	;
    startTimes	[	34	]	=	0.004034089	;
    startTimes	[	35	]	=	0.004077566	;
    startTimes	[	36	]	=	0.004147899	;
    startTimes	[	37	]	=	0.004182138	;
    startTimes	[	38	]	=	0.004218813	;
    startTimes	[	39	]	=	0.004257465	;
    startTimes	[	40	]	=	0.004288959	;
    startTimes	[	41	]	=	0.004340915	;
    startTimes	[	42	]	=	0.004410815	;
    startTimes	[	43	]	=	0.004457169	;
    startTimes	[	44	]	=	0.004478865	;
    startTimes	[	45	]	=	0.004596235	;
    startTimes	[	46	]	=	0.004777589	;
    startTimes	[	47	]	=	0.005023828	;
    startTimes	[	48	]	=	0.005358602	;
    startTimes	[	49	]	=	0.005626749	;
    startTimes	[	50	]	=	0.005820178	;
    startTimes	[	51	]	=	0.005998657	;
    startTimes	[	52	]	=	0.006221562	;
    startTimes	[	53	]	=	0.006548951	;
    startTimes	[	54	]	=	0.006999344	;
    startTimes	[	55	]	=	0.007331702	;
    startTimes	[	56	]	=	0.007583646	;
    startTimes	[	57	]	=	0.007995908	;
    startTimes	[	58	]	=	0.008440343	;
    startTimes	[	59	]	=	0.008999632	;
    startTimes	[	60	]	=	0.009787859	;
    startTimes	[	61	]	=	0.010322346	;
    startTimes	[	62	]	=	0.010776652	;
    startTimes	[	63	]	=	0.011266608	;
    startTimes	[	64	]	=	0.012080861	;
    startTimes	[	65	]	=	0.013100789	;
    startTimes	[	66	]	=	0.014444454	;
    startTimes	[	67	]	=	0.015542875	;
    startTimes	[	68	]	=	0.01640177	;
    startTimes	[	69	]	=	0.017519476	;
    startTimes	[	70	]	=	0.018773712	;
    startTimes	[	71	]	=	0.020367969	;
    startTimes	[	72	]	=	0.022373843	;
    startTimes	[	73	]	=	0.023888801	;
    startTimes	[	74	]	=	0.025126808	;
    startTimes	[	75	]	=	0.026512297	;
    startTimes	[	76	]	=	0.028593341	;
    startTimes	[	77	]	=	0.03125661	;
    startTimes	[	78	]	=	0.034696331	;
    startTimes	[	79	]	=	0.037446682	;
    startTimes	[	80	]	=	0.039664343	;
    startTimes	[	81	]	=	0.043089771	;
    startTimes	[	82	]	=	0.046949554	;
    startTimes	[	83	]	=	0.051589447	;
    startTimes	[	84	]	=	0.057509736	;
    startTimes	[	85	]	=	0.062185788	;
    startTimes	[	86	]	=	0.066093945	;
    startTimes	[	87	]	=	0.070540811	;
    startTimes	[	88	]	=	0.076368334	;
    startTimes	[	89	]	=	0.08353644	;
    startTimes	[	90	]	=	0.092350791	;
    startTimes	[	91	]	=	0.099435829	;
    startTimes	[	92	]	=	0.105253981	;
    startTimes	[	93	]	=	0.111565231	;
    startTimes	[	94	]	=	0.117280789	;
    startTimes	[	95	]	=	0.12375771	;
    startTimes	[	96	]	=	0.131951126	;
    startTimes	[	97	]	=	0.138320279	;
    startTimes	[	98	]	=	0.143025745	;
    startTimes	[	99	]	=	0.147532629	;
    startTimes	[	100	]	=	0.152018243	;
    startTimes	[	101	]	=	0.157651138	;
    startTimes	[	102	]	=	0.165138727	;
    startTimes	[	103	]	=	0.170716928	;
    startTimes	[	104	]	=	0.175158365	;
    startTimes	[	105	]	=	0.180615845	;
    startTimes	[	106	]	=	0.186466016	;
    startTimes	[	107	]	=	0.193960503	;
    startTimes	[	108	]	=	0.203763387	;
    startTimes	[	109	]	=	0.211315462	;
    startTimes	[	110	]	=	0.216701084	;
    startTimes	[	111	]	=	0.221660004	;
    startTimes	[	112	]	=	0.226553519	;
    startTimes	[	113	]	=	0.232978581	;
    startTimes	[	114	]	=	0.241402823	;
    startTimes	[	115	]	=	0.247661262	;
    startTimes	[	116	]	=	0.252308068	;
    startTimes	[	117	]	=	0.257928256	;
    startTimes	[	118	]	=	0.264365681	;
    startTimes	[	119	]	=	0.273073451	;
    startTimes	[	120	]	=	0.28459767	;
    startTimes	[	121	]	=	0.293216915	;
    startTimes	[	122	]	=	0.299100152	;
    startTimes	[	123	]	=	0.304368913	;
    startTimes	[	124	]	=	0.30959424	;
    startTimes	[	125	]	=	0.31630327	;
    startTimes	[	126	]	=	0.325219692	;
    startTimes	[	127	]	=	0.331804434	;
    startTimes	[	128	]	=	0.33668053	;
    startTimes	[	129	]	=	0.342293069	;
    startTimes	[	130	]	=	0.34865862	;
    startTimes	[	131	]	=	0.357525444	;
    startTimes	[	132	]	=	0.36942925	;
    startTimes	[	133	]	=	0.378146747	;
    startTimes	[	134	]	=	0.384222124	;
    startTimes	[	135	]	=	0.389505255	;
    startTimes	[	136	]	=	0.394680065	;
    startTimes	[	137	]	=	0.401569804	;
    startTimes	[	138	]	=	0.411389469	;
    startTimes	[	139	]	=	0.418569998	;
    startTimes	[	140	]	=	0.423760859	;
    startTimes	[	141	]	=	0.429895139	;
    startTimes	[	142	]	=	0.43632823	;
    startTimes	[	143	]	=	0.445140864	;
    startTimes	[	144	]	=	0.456901256	;
    startTimes	[	145	]	=	0.465804608	;
    startTimes	[	146	]	=	0.472261721	;
    startTimes	[	147	]	=	0.478079887	;
    startTimes	[	148	]	=	0.483357046	;
    startTimes	[	149	]	=	0.490109763	;
    startTimes	[	150	]	=	0.4994786	;
    startTimes	[	151	]	=	0.506256948	;
    startTimes	[	152	]	=	0.511019024	;
    startTimes	[	153	]	=	0.515969111	;
    startTimes	[	154	]	=	0.521425085	;
    startTimes	[	155	]	=	0.528767247	;
    startTimes	[	156	]	=	0.539261295	;
    startTimes	[	157	]	=	0.546811498	;
    startTimes	[	158	]	=	0.552037809	;
    startTimes	[	159	]	=	0.556700306	;
    startTimes	[	160	]	=	0.560770498	;
    startTimes	[	161	]	=	0.566867809	;
    startTimes	[	162	]	=	0.575273967	;
    startTimes	[	163	]	=	0.581449478	;
    startTimes	[	164	]	=	0.585407792	;
    startTimes	[	165	]	=	0.589683306	;
    startTimes	[	166	]	=	0.594924848	;
    startTimes	[	167	]	=	0.602129322	;
    startTimes	[	168	]	=	0.612673969	;
    startTimes	[	169	]	=	0.6200676	;
    startTimes	[	170	]	=	0.625122606	;
    startTimes	[	171	]	=	0.629248523	;
    startTimes	[	172	]	=	0.633198937	;
    startTimes	[	173	]	=	0.638744685	;
    startTimes	[	174	]	=	0.646964033	;
    startTimes	[	175	]	=	0.652630681	;
    startTimes	[	176	]	=	0.656554821	;
    startTimes	[	177	]	=	0.661372853	;
    startTimes	[	178	]	=	0.666797241	;
    startTimes	[	179	]	=	0.674608894	;
    startTimes	[	180	]	=	0.6858384	;
    startTimes	[	181	]	=	0.693864406	;
    startTimes	[	182	]	=	0.699350674	;
    startTimes	[	183	]	=	0.70394128	;
    startTimes	[	184	]	=	0.707815445	;
    startTimes	[	185	]	=	0.713744134	;
    startTimes	[	186	]	=	0.722381334	;
    startTimes	[	187	]	=	0.728262784	;
    startTimes	[	188	]	=	0.732410303	;
    startTimes	[	189	]	=	0.736713891	;
    startTimes	[	190	]	=	0.741780789	;
    startTimes	[	191	]	=	0.749389491	;
    startTimes	[	192	]	=	0.760228427	;
    startTimes	[	193	]	=	0.76798386	;
    startTimes	[	194	]	=	0.773208489	;
    startTimes	[	195	]	=	0.777147255	;
    startTimes	[	196	]	=	0.781018918	;
    startTimes	[	197	]	=	0.786719701	;
    startTimes	[	198	]	=	0.795138428	;
    startTimes	[	199	]	=	0.801012717	;
    startTimes	[	200	]	=	0.804507659	;
    startTimes	[	201	]	=	0.8087472	;
    startTimes	[	202	]	=	0.81392503	;
    startTimes	[	203	]	=	0.821896407	;
    startTimes	[	204	]	=	0.833165234	;
    startTimes	[	205	]	=	0.841247788	;
    startTimes	[	206	]	=	0.846920765	;
    startTimes	[	207	]	=	0.851312169	;
    startTimes	[	208	]	=	0.854492023	;
    startTimes	[	209	]	=	0.859235764	;
    startTimes	[	210	]	=	0.866516261	;
    startTimes	[	211	]	=	0.87159234	;
    startTimes	[	212	]	=	0.874622881	;
    startTimes	[	213	]	=	0.877857985	;
    startTimes	[	214	]	=	0.881181332	;
    startTimes	[	215	]	=	0.886857698	;
    startTimes	[	216	]	=	0.895042567	;
    startTimes	[	217	]	=	0.900860278	;
    startTimes	[	218	]	=	0.904483768	;
    startTimes	[	219	]	=	0.907457374	;
    startTimes	[	220	]	=	0.909918727	;
    startTimes	[	221	]	=	0.913657917	;
    startTimes	[	222	]	=	0.919597958	;
    startTimes	[	223	]	=	0.923553488	;
    startTimes	[	224	]	=	0.925886715	;
    startTimes	[	225	]	=	0.927813028	;
    startTimes	[	226	]	=	0.929737845	;
    startTimes	[	227	]	=	0.933265557	;
    startTimes	[	228	]	=	0.93883006	;
    startTimes	[	229	]	=	0.942503414	;
    startTimes	[	230	]	=	0.944801091	;
    startTimes	[	231	]	=	0.94654082	;
    startTimes	[	232	]	=	0.947711109	;
    startTimes	[	233	]	=	0.949884476	;
    startTimes	[	234	]	=	0.953545033	;
    startTimes	[	235	]	=	0.956091778	;
    startTimes	[	236	]	=	0.957513069	;
    startTimes	[	237	]	=	0.958580262	;
    startTimes	[	238	]	=	0.959842889	;
    startTimes	[	239	]	=	0.962360601	;
    startTimes	[	240	]	=	0.966535101	;
    startTimes	[	241	]	=	0.969378337	;
    startTimes	[	242	]	=	0.97093064	;
    startTimes	[	243	]	=	0.972142528	;
    startTimes	[	244	]	=	0.972902949	;
    startTimes	[	245	]	=	0.974461602	;
    startTimes	[	246	]	=	0.977142899	;
    startTimes	[	247	]	=	0.978958916	;
    startTimes	[	248	]	=	0.979929876	;
    startTimes	[	249	]	=	0.980702053	;
    startTimes	[	250	]	=	0.981508427	;
    startTimes	[	251	]	=	0.983161874	;
    startTimes	[	252	]	=	0.985881626	;
    startTimes	[	253	]	=	0.987615292	;
    startTimes	[	254	]	=	0.988462239	;
    startTimes	[	255	]	=	0.988899256	;
    startTimes	[	256	]	=	0.988999163	;
    startTimes	[	257	]	=	0.989634042	;
    startTimes	[	258	]	=	0.990872905	;
    startTimes	[	259	]	=	0.991625719	;
    startTimes	[	260	]	=	0.991990908	;
    startTimes	[	261	]	=	0.992136064	;
    startTimes	[	262	]	=	0.992330746	;
    startTimes	[	263	]	=	0.992950545	;
    startTimes	[	264	]	=	0.994295841	;
    startTimes	[	265	]	=	0.995075997	;
    startTimes	[	266	]	=	0.99539396	;
    startTimes	[	267	]	=	0.995571128	;
    startTimes	[	268	]	=	0.995654711	;
    startTimes	[	269	]	=	0.995937263	;
    startTimes	[	270	]	=	0.996595248	;
    startTimes	[	271	]	=	0.996964806	;
    startTimes	[	272	]	=	0.997066766	;
    startTimes	[	273	]	=	0.997079821	;
    startTimes	[	274	]	=	0.997149795	;
    startTimes	[	275	]	=	0.997519848	;
    startTimes	[	276	]	=	0.998246095	;
    startTimes	[	277	]	=	0.998673298	;
    startTimes	[	278	]	=	0.998910772	;
    startTimes	[	279	]	=	0.999018897	;
    startTimes	[	280	]	=	0.999052551	;
    startTimes	[	281	]	=	0.999187415	;
    startTimes	[	282	]	=	0.999510452	;
    startTimes	[	283	]	=	0.999714046	;
    startTimes	[	284	]	=	0.999800488	;
    startTimes	[	285	]	=	0.999857822	;
    startTimes	[	286	]	=	0.999957729	;
    startTimes	[	287	]	=	1	;



// Seattle

/*
    startTimes	[	0	]	=	0.000183461	;
    startTimes	[	1	]	=	0.000596574	;
    startTimes	[	2	]	=	0.000871927	;
    startTimes	[	3	]	=	0.001090345	;
    startTimes	[	4	]	=	0.001222614	;
    startTimes	[	5	]	=	0.001430235	;
    startTimes	[	6	]	=	0.001747832	;
    startTimes	[	7	]	=	0.001958558	;
    startTimes	[	8	]	=	0.00208014	;
    startTimes	[	9	]	=	0.00211278	;
    startTimes	[	10	]	=	0.002151691	;
    startTimes	[	11	]	=	0.002284651	;
    startTimes	[	12	]	=	0.002473998	;
    startTimes	[	13	]	=	0.002605708	;
    startTimes	[	14	]	=	0.002698029	;
    startTimes	[	15	]	=	0.002762203	;
    startTimes	[	16	]	=	0.002799451	;
    startTimes	[	17	]	=	0.00288155	;
    startTimes	[	18	]	=	0.002979462	;
    startTimes	[	19	]	=	0.003022862	;
    startTimes	[	20	]	=	0.003082744	;
    startTimes	[	21	]	=	0.003109741	;
    startTimes	[	22	]	=	0.003182533	;
    startTimes	[	23	]	=	0.003286579	;
    startTimes	[	24	]	=	0.003444201	;
    startTimes	[	25	]	=	0.00352496	;
    startTimes	[	26	]	=	0.003577972	;
    startTimes	[	27	]	=	0.003610522	;
    startTimes	[	28	]	=	0.003613255	;
    startTimes	[	29	]	=	0.003620196	;
    startTimes	[	30	]	=	0.003682573	;
    startTimes	[	31	]	=	0.003696662	;
    startTimes	[	32	]	=	0.003727752	;
    startTimes	[	33	]	=	0.003744146	;
    startTimes	[	34	]	=	0.003761942	;
    startTimes	[	35	]	=	0.003812491	;
    startTimes	[	36	]	=	0.003950531	;
    startTimes	[	37	]	=	0.004084371	;
    startTimes	[	38	]	=	0.004217336	;
    startTimes	[	39	]	=	0.004384278	;
    startTimes	[	40	]	=	0.004543568	;
    startTimes	[	41	]	=	0.004749186	;
    startTimes	[	42	]	=	0.005046083	;
    startTimes	[	43	]	=	0.00532993	;
    startTimes	[	44	]	=	0.005580792	;
    startTimes	[	45	]	=	0.005803891	;
    startTimes	[	46	]	=	0.005994353	;
    startTimes	[	47	]	=	0.006259453	;
    startTimes	[	48	]	=	0.006589168	;
    startTimes	[	49	]	=	0.006998403	;
    startTimes	[	50	]	=	0.007423965	;
    startTimes	[	51	]	=	0.007920648	;
    startTimes	[	52	]	=	0.008520013	;
    startTimes	[	53	]	=	0.009286647	;
    startTimes	[	54	]	=	0.010159155	;
    startTimes	[	55	]	=	0.011205492	;
    startTimes	[	56	]	=	0.012326014	;
    startTimes	[	57	]	=	0.013560625	;
    startTimes	[	58	]	=	0.014858033	;
    startTimes	[	59	]	=	0.016434251	;
    startTimes	[	60	]	=	0.01822464	;
    startTimes	[	61	]	=	0.020305092	;
    startTimes	[	62	]	=	0.022416624	;
    startTimes	[	63	]	=	0.02463804	;
    startTimes	[	64	]	=	0.027024085	;
    startTimes	[	65	]	=	0.030006752	;
    startTimes	[	66	]	=	0.033561092	;
    startTimes	[	67	]	=	0.03710062	;
    startTimes	[	68	]	=	0.040929505	;
    startTimes	[	69	]	=	0.04481586	;
    startTimes	[	70	]	=	0.04862358	;
    startTimes	[	71	]	=	0.052884651	;
    startTimes	[	72	]	=	0.057567476	;
    startTimes	[	73	]	=	0.062513107	;
    startTimes	[	74	]	=	0.067622096	;
    startTimes	[	75	]	=	0.073071824	;
    startTimes	[	76	]	=	0.078687872	;
    startTimes	[	77	]	=	0.085052914	;
    startTimes	[	78	]	=	0.092086807	;
    startTimes	[	79	]	=	0.099337401	;
    startTimes	[	80	]	=	0.106889548	;
    startTimes	[	81	]	=	0.114265795	;
    startTimes	[	82	]	=	0.121200514	;
    startTimes	[	83	]	=	0.128924208	;
    startTimes	[	84	]	=	0.137004701	;
    startTimes	[	85	]	=	0.145567947	;
    startTimes	[	86	]	=	0.15451886	;
    startTimes	[	87	]	=	0.163415375	;
    startTimes	[	88	]	=	0.172420881	;
    startTimes	[	89	]	=	0.181978516	;
    startTimes	[	90	]	=	0.191986474	;
    startTimes	[	91	]	=	0.201954298	;
    startTimes	[	92	]	=	0.211682193	;
    startTimes	[	93	]	=	0.221045494	;
    startTimes	[	94	]	=	0.229708656	;
    startTimes	[	95	]	=	0.237886731	;
    startTimes	[	96	]	=	0.24522962	;
    startTimes	[	97	]	=	0.252635693	;
    startTimes	[	98	]	=	0.259786872	;
    startTimes	[	99	]	=	0.26648229	;
    startTimes	[	100	]	=	0.272748695	;
    startTimes	[	101	]	=	0.278781643	;
    startTimes	[	102	]	=	0.284526536	;
    startTimes	[	103	]	=	0.290673049	;
    startTimes	[	104	]	=	0.296917168	;
    startTimes	[	105	]	=	0.303028972	;
    startTimes	[	106	]	=	0.309174682	;
    startTimes	[	107	]	=	0.314829732	;
    startTimes	[	108	]	=	0.319960391	;
    startTimes	[	109	]	=	0.325212349	;
    startTimes	[	110	]	=	0.33051277	;
    startTimes	[	111	]	=	0.335469414	;
    startTimes	[	112	]	=	0.339952666	;
    startTimes	[	113	]	=	0.34446612	;
    startTimes	[	114	]	=	0.348853696	;
    startTimes	[	115	]	=	0.353164849	;
    startTimes	[	116	]	=	0.357294919	;
    startTimes	[	117	]	=	0.361296683	;
    startTimes	[	118	]	=	0.364812529	;
    startTimes	[	119	]	=	0.368518461	;
    startTimes	[	120	]	=	0.372196916	;
    startTimes	[	121	]	=	0.375850288	;
    startTimes	[	122	]	=	0.379380212	;
    startTimes	[	123	]	=	0.382935793	;
    startTimes	[	124	]	=	0.386465073	;
    startTimes	[	125	]	=	0.390242535	;
    startTimes	[	126	]	=	0.394311021	;
    startTimes	[	127	]	=	0.398183887	;
    startTimes	[	128	]	=	0.401869394	;
    startTimes	[	129	]	=	0.405345993	;
    startTimes	[	130	]	=	0.40854369	;
    startTimes	[	131	]	=	0.412241575	;
    startTimes	[	132	]	=	0.416387881	;
    startTimes	[	133	]	=	0.420702342	;
    startTimes	[	134	]	=	0.425078713	;
    startTimes	[	135	]	=	0.429062509	;
    startTimes	[	136	]	=	0.432881023	;
    startTimes	[	137	]	=	0.436684357	;
    startTimes	[	138	]	=	0.440924075	;
    startTimes	[	139	]	=	0.445442816	;
    startTimes	[	140	]	=	0.449891652	;
    startTimes	[	141	]	=	0.454289863	;
    startTimes	[	142	]	=	0.458535971	;
    startTimes	[	143	]	=	0.462743029	;
    startTimes	[	144	]	=	0.466779917	;
    startTimes	[	145	]	=	0.471024597	;
    startTimes	[	146	]	=	0.475213431	;
    startTimes	[	147	]	=	0.479209109	;
    startTimes	[	148	]	=	0.482993894	;
    startTimes	[	149	]	=	0.487104523	;
    startTimes	[	150	]	=	0.491321655	;
    startTimes	[	151	]	=	0.495780369	;
    startTimes	[	152	]	=	0.500718262	;
    startTimes	[	153	]	=	0.505258992	;
    startTimes	[	154	]	=	0.509582056	;
    startTimes	[	155	]	=	0.513335699	;
    startTimes	[	156	]	=	0.516498085	;
    startTimes	[	157	]	=	0.519743524	;
    startTimes	[	158	]	=	0.522881118	;
    startTimes	[	159	]	=	0.525853311	;
    startTimes	[	160	]	=	0.528706576	;
    startTimes	[	161	]	=	0.531866253	;
    startTimes	[	162	]	=	0.53527258	;
    startTimes	[	163	]	=	0.538836004	;
    startTimes	[	164	]	=	0.542282294	;
    startTimes	[	165	]	=	0.545655948	;
    startTimes	[	166	]	=	0.549149988	;
    startTimes	[	167	]	=	0.552752026	;
    startTimes	[	168	]	=	0.556659825	;
    startTimes	[	169	]	=	0.560683431	;
    startTimes	[	170	]	=	0.56507933	;
    startTimes	[	171	]	=	0.569427806	;
    startTimes	[	172	]	=	0.574011366	;
    startTimes	[	173	]	=	0.579049365	;
    startTimes	[	174	]	=	0.584865759	;
    startTimes	[	175	]	=	0.590900169	;
    startTimes	[	176	]	=	0.596885483	;
    startTimes	[	177	]	=	0.602587824	;
    startTimes	[	178	]	=	0.60802705	;
    startTimes	[	179	]	=	0.613665778	;
    startTimes	[	180	]	=	0.619246768	;
    startTimes	[	181	]	=	0.624957036	;
    startTimes	[	182	]	=	0.63062579	;
    startTimes	[	183	]	=	0.636163869	;
    startTimes	[	184	]	=	0.641501526	;
    startTimes	[	185	]	=	0.647327841	;
    startTimes	[	186	]	=	0.653862443	;
    startTimes	[	187	]	=	0.660243616	;
    startTimes	[	188	]	=	0.667142344	;
    startTimes	[	189	]	=	0.673725281	;
    startTimes	[	190	]	=	0.680195068	;
    startTimes	[	191	]	=	0.687004055	;
    startTimes	[	192	]	=	0.694293224	;
    startTimes	[	193	]	=	0.701990609	;
    startTimes	[	194	]	=	0.709712031	;
    startTimes	[	195	]	=	0.716851798	;
    startTimes	[	196	]	=	0.723732206	;
    startTimes	[	197	]	=	0.731567437	;
    startTimes	[	198	]	=	0.740713409	;
    startTimes	[	199	]	=	0.749923008	;
    startTimes	[	200	]	=	0.759181918	;
    startTimes	[	201	]	=	0.767746298	;
    startTimes	[	202	]	=	0.776000297	;
    startTimes	[	203	]	=	0.78526507	;
    startTimes	[	204	]	=	0.795614381	;
    startTimes	[	205	]	=	0.806143344	;
    startTimes	[	206	]	=	0.816922379	;
    startTimes	[	207	]	=	0.826801214	;
    startTimes	[	208	]	=	0.835866439	;
    startTimes	[	209	]	=	0.844901356	;
    startTimes	[	210	]	=	0.853442364	;
    startTimes	[	211	]	=	0.862032344	;
    startTimes	[	212	]	=	0.870549408	;
    startTimes	[	213	]	=	0.878289593	;
    startTimes	[	214	]	=	0.885416918	;
    startTimes	[	215	]	=	0.892185668	;
    startTimes	[	216	]	=	0.898380971	;
    startTimes	[	217	]	=	0.904206166	;
    startTimes	[	218	]	=	0.909896437	;
    startTimes	[	219	]	=	0.915086733	;
    startTimes	[	220	]	=	0.919711908	;
    startTimes	[	221	]	=	0.923957823	;
    startTimes	[	222	]	=	0.92843847	;
    startTimes	[	223	]	=	0.932561887	;
    startTimes	[	224	]	=	0.93643621	;
    startTimes	[	225	]	=	0.939657975	;
    startTimes	[	226	]	=	0.9426611	;
    startTimes	[	227	]	=	0.945108119	;
    startTimes	[	228	]	=	0.947148325	;
    startTimes	[	229	]	=	0.949238837	;
    startTimes	[	230	]	=	0.951167108	;
    startTimes	[	231	]	=	0.952792507	;
    startTimes	[	232	]	=	0.954022253	;
    startTimes	[	233	]	=	0.955091142	;
    startTimes	[	234	]	=	0.955875276	;
    startTimes	[	235	]	=	0.956875188	;
    startTimes	[	236	]	=	0.957838692	;
    startTimes	[	237	]	=	0.958956258	;
    startTimes	[	238	]	=	0.959757687	;
    startTimes	[	239	]	=	0.960793715	;
    startTimes	[	240	]	=	0.961911685	;
    startTimes	[	241	]	=	0.963100369	;
    startTimes	[	242	]	=	0.964587321	;
    startTimes	[	243	]	=	0.965769005	;
    startTimes	[	244	]	=	0.966798672	;
    startTimes	[	245	]	=	0.968094664	;
    startTimes	[	246	]	=	0.969518949	;
    startTimes	[	247	]	=	0.971035082	;
    startTimes	[	248	]	=	0.972752853	;
    startTimes	[	249	]	=	0.97432911	;
    startTimes	[	250	]	=	0.975795242	;
    startTimes	[	251	]	=	0.977529293	;
    startTimes	[	252	]	=	0.979448313	;
    startTimes	[	253	]	=	0.980844805	;
    startTimes	[	254	]	=	0.982242169	;
    startTimes	[	255	]	=	0.983360661	;
    startTimes	[	256	]	=	0.984444358	;
    startTimes	[	257	]	=	0.985474344	;
    startTimes	[	258	]	=	0.986621714	;
    startTimes	[	259	]	=	0.98768696	;
    startTimes	[	260	]	=	0.988556952	;
    startTimes	[	261	]	=	0.989298872	;
    startTimes	[	262	]	=	0.989902757	;
    startTimes	[	263	]	=	0.990573223	;
    startTimes	[	264	]	=	0.99121407	;
    startTimes	[	265	]	=	0.991858629	;
    startTimes	[	266	]	=	0.992579579	;
    startTimes	[	267	]	=	0.993313715	;
    startTimes	[	268	]	=	0.993821689	;
    startTimes	[	269	]	=	0.994377862	;
    startTimes	[	270	]	=	0.994812846	;
    startTimes	[	271	]	=	0.99526254	;
    startTimes	[	272	]	=	0.995665424	;
    startTimes	[	273	]	=	0.996095064	;
    startTimes	[	274	]	=	0.996406882	;
    startTimes	[	275	]	=	0.996867925	;
    startTimes	[	276	]	=	0.997474644	;
    startTimes	[	277	]	=	0.998047441	;
    startTimes	[	278	]	=	0.998521301	;
    startTimes	[	279	]	=	0.99882626	;
    startTimes	[	280	]	=	0.999050456	;
    startTimes	[	281	]	=	0.999200386	;
    startTimes	[	282	]	=	0.999475475	;
    startTimes	[	283	]	=	0.999641159	;
    startTimes	[	284	]	=	0.99976555	;
    startTimes	[	285	]	=	0.999865626	;
    startTimes	[	286	]	=	0.999952841	;
    startTimes	[	287	]	=	1	;

*/



    return;
}

// distribution of trip distances
void Simulator::setTripDist()
{
    tripDistSize = 60;
    tripDist = new double[tripDistSize];	
    tripDist	[	0	]	=	0	;
    tripDist	[	1	]	=	0	;
    tripDist	[	2	]	=	0	;
    tripDist	[	3	]	=	0.036004487	;
    tripDist	[	4	]	=	0.074292508	;
    tripDist	[	5	]	=	0.114864061	;
    tripDist	[	6	]	=	0.157719149	;
    tripDist	[	7	]	=	0.20285777	;
    tripDist	[	8	]	=	0.245800107	;
    tripDist	[	9	]	=	0.286546159	;
    tripDist	[	10	]	=	0.325095927	;
    tripDist	[	11	]	=	0.361449411	;
    tripDist	[	12	]	=	0.395300041	;
    tripDist	[	13	]	=	0.426647816	;
    tripDist	[	14	]	=	0.455492738	;
    tripDist	[	15	]	=	0.481834806	;
    tripDist	[	16	]	=	0.508150699	;
    tripDist	[	17	]	=	0.534440417	;
    tripDist	[	18	]	=	0.560703961	;
    tripDist	[	19	]	=	0.586941329	;
    tripDist	[	20	]	=	0.611033822	;
    tripDist	[	21	]	=	0.63298144	;
    tripDist	[	22	]	=	0.652784181	;
    tripDist	[	23	]	=	0.670442047	;
    tripDist	[	24	]	=	0.687427239	;
    tripDist	[	25	]	=	0.703739757	;
    tripDist	[	26	]	=	0.7193796	;
    tripDist	[	27	]	=	0.73434677	;
    tripDist	[	28	]	=	0.748966653	;
    tripDist	[	29	]	=	0.763239251	;
    tripDist	[	30	]	=	0.777164562	;
    tripDist	[	31	]	=	0.790742588	;
    tripDist	[	32	]	=	0.80315526	;
    tripDist	[	33	]	=	0.814402577	;
    tripDist	[	34	]	=	0.824484541	;
    tripDist	[	35	]	=	0.833401151	;
    tripDist	[	36	]	=	0.842521513	;
    tripDist	[	37	]	=	0.851845628	;
    tripDist	[	38	]	=	0.861373495	;
    tripDist	[	39	]	=	0.871105115	;
    tripDist	[	40	]	=	0.880233882	;
    tripDist	[	41	]	=	0.888759795	;
    tripDist	[	42	]	=	0.896682855	;
    tripDist	[	43	]	=	0.904003061	;
    tripDist	[	44	]	=	0.911492248	;
    tripDist	[	45	]	=	0.919150417	;
    tripDist	[	46	]	=	0.926977567	;
    tripDist	[	47	]	=	0.934973698	;
    tripDist	[	48	]	=	0.942085618	;
    tripDist	[	49	]	=	0.948313328	;
    tripDist	[	50	]	=	0.953656827	;
    tripDist	[	51	]	=	0.958116116	;
    tripDist	[	52	]	=	0.962421777	;
    tripDist	[	53	]	=	0.966573812	;
    tripDist	[	54	]	=	0.970572219	;
    tripDist	[	55	]	=	0.974417	;
    tripDist	[	56	]	=	0.979282168	;
    tripDist	[	57	]	=	0.985167725	;
    tripDist	[	58	]	=	0.992073668	;
    tripDist	[	59	]	=	1	;


    return;
}

// Distribution of trip generation for larger grid
void Simulator::setTripDistLarge()
{
	tripDistSize = 601;
	tripDist = new double [tripDistSize];	
	tripDist   [     0     ]     =     0     ;
	tripDist   [     1     ]     =     0     ;
	tripDist   [     2     ]     =     0     ;
	tripDist   [     3     ]     =     0.0249940865     ;
	tripDist   [     4     ]     =     0.0499881731     ;
	tripDist   [     5     ]     =     0.079447481     ;
	tripDist   [     6     ]     =     0.1089067889     ;
	tripDist   [     7     ]     =     0.1383660968     ;
	tripDist   [     8     ]     =     0.1678254047     ;
	tripDist   [     9     ]     =     0.20036481     ;
	tripDist   [     10     ]     =     0.2329042154     ;
	tripDist   [     11     ]     =     0.2654436207     ;
	tripDist   [     12     ]     =     0.297983026     ;
	tripDist   [     13     ]     =     0.3242170793     ;
	tripDist   [     14     ]     =     0.3504511327     ;
	tripDist   [     15     ]     =     0.376685186     ;
	tripDist   [     16     ]     =     0.4029192393     ;
	tripDist   [     17     ]     =     0.4217610065     ;
	tripDist   [     18     ]     =     0.4406027736     ;
	tripDist   [     19     ]     =     0.4594445408     ;
	tripDist   [     20     ]     =     0.478286308     ;
	tripDist   [     21     ]     =     0.4898531466     ;
	tripDist   [     22     ]     =     0.5014199853     ;
	tripDist   [     23     ]     =     0.5129868239     ;
	tripDist   [     24     ]     =     0.5245536625     ;
	tripDist   [     25     ]     =     0.5361205012     ;
	tripDist   [     26     ]     =     0.5476873398     ;
	tripDist   [     27     ]     =     0.5592541785     ;
	tripDist   [     28     ]     =     0.5708210171     ;
	tripDist   [     29     ]     =     0.5823878557     ;
	tripDist   [     30     ]     =     0.5939546944     ;
	tripDist   [     31     ]     =     0.605521533     ;
	tripDist   [     32     ]     =     0.6170883717     ;
	tripDist   [     33     ]     =     0.6286552103     ;
	tripDist   [     34     ]     =     0.6402220489     ;
	tripDist   [     35     ]     =     0.6517888876     ;
	tripDist   [     36     ]     =     0.6633557262     ;
	tripDist   [     37     ]     =     0.6749225649     ;
	tripDist   [     38     ]     =     0.6864894035     ;
	tripDist   [     39     ]     =     0.6980562421     ;
	tripDist   [     40     ]     =     0.7096230808     ;
	tripDist   [     41     ]     =     0.7149311591     ;
	tripDist   [     42     ]     =     0.7202392373     ;
	tripDist   [     43     ]     =     0.7255473156     ;
	tripDist   [     44     ]     =     0.7308553939     ;
	tripDist   [     45     ]     =     0.7361634721     ;
	tripDist   [     46     ]     =     0.7414715504     ;
	tripDist   [     47     ]     =     0.7467796287     ;
	tripDist   [     48     ]     =     0.752087707     ;
	tripDist   [     49     ]     =     0.7573957852     ;
	tripDist   [     50     ]     =     0.7627038635     ;
	tripDist   [     51     ]     =     0.7680119418     ;
	tripDist   [     52     ]     =     0.77332002     ;
	tripDist   [     53     ]     =     0.7786280983     ;
	tripDist   [     54     ]     =     0.7839361766     ;
	tripDist   [     55     ]     =     0.7892442549     ;
	tripDist   [     56     ]     =     0.7945523331     ;
	tripDist   [     57     ]     =     0.7998604114     ;
	tripDist   [     58     ]     =     0.8051684897     ;
	tripDist   [     59     ]     =     0.8104765679     ;
	tripDist   [     60     ]     =     0.8157846462     ;
	tripDist   [     61     ]     =     0.8188658528     ;
	tripDist   [     62     ]     =     0.8219470594     ;
	tripDist   [     63     ]     =     0.825028266     ;
	tripDist   [     64     ]     =     0.8281094726     ;
	tripDist   [     65     ]     =     0.8311906793     ;
	tripDist   [     66     ]     =     0.8342718859     ;
	tripDist   [     67     ]     =     0.8373530925     ;
	tripDist   [     68     ]     =     0.8404342991     ;
	tripDist   [     69     ]     =     0.8435155057     ;
	tripDist   [     70     ]     =     0.8465967123     ;
	tripDist   [     71     ]     =     0.8496779189     ;
	tripDist   [     72     ]     =     0.8527591255     ;
	tripDist   [     73     ]     =     0.8558403321     ;
	tripDist   [     74     ]     =     0.8589215387     ;
	tripDist   [     75     ]     =     0.8620027453     ;
	tripDist   [     76     ]     =     0.8650839519     ;
	tripDist   [     77     ]     =     0.8681651585     ;
	tripDist   [     78     ]     =     0.8712463651     ;
	tripDist   [     79     ]     =     0.8743275717     ;
	tripDist   [     80     ]     =     0.8774087783     ;
	tripDist   [     81     ]     =     0.8792618394     ;
	tripDist   [     82     ]     =     0.8811149005     ;
	tripDist   [     83     ]     =     0.8829679615     ;
	tripDist   [     84     ]     =     0.8848210226     ;
	tripDist   [     85     ]     =     0.8866740837     ;
	tripDist   [     86     ]     =     0.8885271447     ;
	tripDist   [     87     ]     =     0.8903802058     ;
	tripDist   [     88     ]     =     0.8922332669     ;
	tripDist   [     89     ]     =     0.8940863279     ;
	tripDist   [     90     ]     =     0.895939389     ;
	tripDist   [     91     ]     =     0.89779245     ;
	tripDist   [     92     ]     =     0.8996455111     ;
	tripDist   [     93     ]     =     0.9014985722     ;
	tripDist   [     94     ]     =     0.9033516332     ;
	tripDist   [     95     ]     =     0.9052046943     ;
	tripDist   [     96     ]     =     0.9070577554     ;
	tripDist   [     97     ]     =     0.9089108164     ;
	tripDist   [     98     ]     =     0.9107638775     ;
	tripDist   [     99     ]     =     0.9126169385     ;
	tripDist   [     100     ]     =     0.9144699996     ;
	tripDist   [     101     ]     =     0.9156317318     ;
	tripDist   [     102     ]     =     0.9167934639     ;
	tripDist   [     103     ]     =     0.917955196     ;
	tripDist   [     104     ]     =     0.9191169282     ;
	tripDist   [     105     ]     =     0.9202786603     ;
	tripDist   [     106     ]     =     0.9214403925     ;
	tripDist   [     107     ]     =     0.9226021246     ;
	tripDist   [     108     ]     =     0.9237638568     ;
	tripDist   [     109     ]     =     0.9249255889     ;
	tripDist   [     110     ]     =     0.9260873211     ;
	tripDist   [     111     ]     =     0.9272490532     ;
	tripDist   [     112     ]     =     0.9284107854     ;
	tripDist   [     113     ]     =     0.9295725175     ;
	tripDist   [     114     ]     =     0.9307342497     ;
	tripDist   [     115     ]     =     0.9318959818     ;
	tripDist   [     116     ]     =     0.9330577139     ;
	tripDist   [     117     ]     =     0.9342194461     ;
	tripDist   [     118     ]     =     0.9353811782     ;
	tripDist   [     119     ]     =     0.9365429104     ;
	tripDist   [     120     ]     =     0.9377046425     ;
	tripDist   [     121     ]     =     0.938535976     ;
	tripDist   [     122     ]     =     0.9393673094     ;
	tripDist   [     123     ]     =     0.9401986429     ;
	tripDist   [     124     ]     =     0.9410299763     ;
	tripDist   [     125     ]     =     0.9418613098     ;
	tripDist   [     126     ]     =     0.9426926432     ;
	tripDist   [     127     ]     =     0.9435239767     ;
	tripDist   [     128     ]     =     0.9443553101     ;
	tripDist   [     129     ]     =     0.9451866436     ;
	tripDist   [     130     ]     =     0.946017977     ;
	tripDist   [     131     ]     =     0.9468493105     ;
	tripDist   [     132     ]     =     0.9476806439     ;
	tripDist   [     133     ]     =     0.9485119774     ;
	tripDist   [     134     ]     =     0.9493433108     ;
	tripDist   [     135     ]     =     0.9501746443     ;
	tripDist   [     136     ]     =     0.9510059777     ;
	tripDist   [     137     ]     =     0.9518373112     ;
	tripDist   [     138     ]     =     0.9526686446     ;
	tripDist   [     139     ]     =     0.9534999781     ;
	tripDist   [     140     ]     =     0.9543313115     ;
	tripDist   [     141     ]     =     0.9548430061     ;
	tripDist   [     142     ]     =     0.9553547006     ;
	tripDist   [     143     ]     =     0.9558663952     ;
	tripDist   [     144     ]     =     0.9563780897     ;
	tripDist   [     145     ]     =     0.9568897842     ;
	tripDist   [     146     ]     =     0.9574014788     ;
	tripDist   [     147     ]     =     0.9579131733     ;
	tripDist   [     148     ]     =     0.9584248679     ;
	tripDist   [     149     ]     =     0.9589365624     ;
	tripDist   [     150     ]     =     0.959448257     ;
	tripDist   [     151     ]     =     0.9599599515     ;
	tripDist   [     152     ]     =     0.960471646     ;
	tripDist   [     153     ]     =     0.9609833406     ;
	tripDist   [     154     ]     =     0.9614950351     ;
	tripDist   [     155     ]     =     0.9620067297     ;
	tripDist   [     156     ]     =     0.9625184242     ;
	tripDist   [     157     ]     =     0.9630301187     ;
	tripDist   [     158     ]     =     0.9635418133     ;
	tripDist   [     159     ]     =     0.9640535078     ;
	tripDist   [     160     ]     =     0.9645652024     ;
	tripDist   [     161     ]     =     0.964933324     ;
	tripDist   [     162     ]     =     0.9653014456     ;
	tripDist   [     163     ]     =     0.9656695672     ;
	tripDist   [     164     ]     =     0.9660376888     ;
	tripDist   [     165     ]     =     0.9664058104     ;
	tripDist   [     166     ]     =     0.966773932     ;
	tripDist   [     167     ]     =     0.9671420536     ;
	tripDist   [     168     ]     =     0.9675101751     ;
	tripDist   [     169     ]     =     0.9678782967     ;
	tripDist   [     170     ]     =     0.9682464183     ;
	tripDist   [     171     ]     =     0.9686145399     ;
	tripDist   [     172     ]     =     0.9689826615     ;
	tripDist   [     173     ]     =     0.9693507831     ;
	tripDist   [     174     ]     =     0.9697189047     ;
	tripDist   [     175     ]     =     0.9700870263     ;
	tripDist   [     176     ]     =     0.9704551479     ;
	tripDist   [     177     ]     =     0.9708232695     ;
	tripDist   [     178     ]     =     0.9711913911     ;
	tripDist   [     179     ]     =     0.9715595127     ;
	tripDist   [     180     ]     =     0.9719276343     ;
	tripDist   [     181     ]     =     0.9721847487     ;
	tripDist   [     182     ]     =     0.9724418631     ;
	tripDist   [     183     ]     =     0.9726989775     ;
	tripDist   [     184     ]     =     0.9729560919     ;
	tripDist   [     185     ]     =     0.9732132063     ;
	tripDist   [     186     ]     =     0.9734703207     ;
	tripDist   [     187     ]     =     0.9737274351     ;
	tripDist   [     188     ]     =     0.9739845495     ;
	tripDist   [     189     ]     =     0.9742416639     ;
	tripDist   [     190     ]     =     0.9744987783     ;
	tripDist   [     191     ]     =     0.9747558927     ;
	tripDist   [     192     ]     =     0.9750130071     ;
	tripDist   [     193     ]     =     0.9752701215     ;
	tripDist   [     194     ]     =     0.9755272359     ;
	tripDist   [     195     ]     =     0.9757843503     ;
	tripDist   [     196     ]     =     0.9760414647     ;
	tripDist   [     197     ]     =     0.9762985791     ;
	tripDist   [     198     ]     =     0.9765556935     ;
	tripDist   [     199     ]     =     0.9768128079     ;
	tripDist   [     200     ]     =     0.9770699223     ;
	tripDist   [     201     ]     =     0.9771863162     ;
	tripDist   [     202     ]     =     0.9773027101     ;
	tripDist   [     203     ]     =     0.977419104     ;
	tripDist   [     204     ]     =     0.9775354979     ;
	tripDist   [     205     ]     =     0.9776518918     ;
	tripDist   [     206     ]     =     0.9777682857     ;
	tripDist   [     207     ]     =     0.9778846797     ;
	tripDist   [     208     ]     =     0.9780010736     ;
	tripDist   [     209     ]     =     0.9781174675     ;
	tripDist   [     210     ]     =     0.9782338614     ;
	tripDist   [     211     ]     =     0.9783502553     ;
	tripDist   [     212     ]     =     0.9784666492     ;
	tripDist   [     213     ]     =     0.9785830431     ;
	tripDist   [     214     ]     =     0.978699437     ;
	tripDist   [     215     ]     =     0.9788158309     ;
	tripDist   [     216     ]     =     0.9789322249     ;
	tripDist   [     217     ]     =     0.9790486188     ;
	tripDist   [     218     ]     =     0.9791650127     ;
	tripDist   [     219     ]     =     0.9792814066     ;
	tripDist   [     220     ]     =     0.9793978005     ;
	tripDist   [     221     ]     =     0.9795141944     ;
	tripDist   [     222     ]     =     0.9796305883     ;
	tripDist   [     223     ]     =     0.9797469822     ;
	tripDist   [     224     ]     =     0.9798633761     ;
	tripDist   [     225     ]     =     0.9799797701     ;
	tripDist   [     226     ]     =     0.980096164     ;
	tripDist   [     227     ]     =     0.9802125579     ;
	tripDist   [     228     ]     =     0.9803289518     ;
	tripDist   [     229     ]     =     0.9804453457     ;
	tripDist   [     230     ]     =     0.9805617396     ;
	tripDist   [     231     ]     =     0.9806781335     ;
	tripDist   [     232     ]     =     0.9807945274     ;
	tripDist   [     233     ]     =     0.9809109213     ;
	tripDist   [     234     ]     =     0.9810273152     ;
	tripDist   [     235     ]     =     0.9811437092     ;
	tripDist   [     236     ]     =     0.9812601031     ;
	tripDist   [     237     ]     =     0.981376497     ;
	tripDist   [     238     ]     =     0.9814928909     ;
	tripDist   [     239     ]     =     0.9816092848     ;
	tripDist   [     240     ]     =     0.9817256787     ;
	tripDist   [     241     ]     =     0.9818420726     ;
	tripDist   [     242     ]     =     0.9819584665     ;
	tripDist   [     243     ]     =     0.9820748604     ;
	tripDist   [     244     ]     =     0.9821912544     ;
	tripDist   [     245     ]     =     0.9823076483     ;
	tripDist   [     246     ]     =     0.9824240422     ;
	tripDist   [     247     ]     =     0.9825404361     ;
	tripDist   [     248     ]     =     0.98265683     ;
	tripDist   [     249     ]     =     0.9827732239     ;
	tripDist   [     250     ]     =     0.9828896178     ;
	tripDist   [     251     ]     =     0.9830060117     ;
	tripDist   [     252     ]     =     0.9831224056     ;
	tripDist   [     253     ]     =     0.9832387996     ;
	tripDist   [     254     ]     =     0.9833551935     ;
	tripDist   [     255     ]     =     0.9834715874     ;
	tripDist   [     256     ]     =     0.9835879813     ;
	tripDist   [     257     ]     =     0.9837043752     ;
	tripDist   [     258     ]     =     0.9838207691     ;
	tripDist   [     259     ]     =     0.983937163     ;
	tripDist   [     260     ]     =     0.9840535569     ;
	tripDist   [     261     ]     =     0.9841699508     ;
	tripDist   [     262     ]     =     0.9842863447     ;
	tripDist   [     263     ]     =     0.9844027387     ;
	tripDist   [     264     ]     =     0.9845191326     ;
	tripDist   [     265     ]     =     0.9846355265     ;
	tripDist   [     266     ]     =     0.9847519204     ;
	tripDist   [     267     ]     =     0.9848683143     ;
	tripDist   [     268     ]     =     0.9849847082     ;
	tripDist   [     269     ]     =     0.9851011021     ;
	tripDist   [     270     ]     =     0.985217496     ;
	tripDist   [     271     ]     =     0.9853338899     ;
	tripDist   [     272     ]     =     0.9854502839     ;
	tripDist   [     273     ]     =     0.9855666778     ;
	tripDist   [     274     ]     =     0.9856830717     ;
	tripDist   [     275     ]     =     0.9857994656     ;
	tripDist   [     276     ]     =     0.9859158595     ;
	tripDist   [     277     ]     =     0.9860322534     ;
	tripDist   [     278     ]     =     0.9861486473     ;
	tripDist   [     279     ]     =     0.9862650412     ;
	tripDist   [     280     ]     =     0.9863814351     ;
	tripDist   [     281     ]     =     0.9864978291     ;
	tripDist   [     282     ]     =     0.986614223     ;
	tripDist   [     283     ]     =     0.9867306169     ;
	tripDist   [     284     ]     =     0.9868470108     ;
	tripDist   [     285     ]     =     0.9869634047     ;
	tripDist   [     286     ]     =     0.9870797986     ;
	tripDist   [     287     ]     =     0.9871961925     ;
	tripDist   [     288     ]     =     0.9873125864     ;
	tripDist   [     289     ]     =     0.9874289803     ;
	tripDist   [     290     ]     =     0.9875453742     ;
	tripDist   [     291     ]     =     0.9876617682     ;
	tripDist   [     292     ]     =     0.9877781621     ;
	tripDist   [     293     ]     =     0.987894556     ;
	tripDist   [     294     ]     =     0.9880109499     ;
	tripDist   [     295     ]     =     0.9881273438     ;
	tripDist   [     296     ]     =     0.9882437377     ;
	tripDist   [     297     ]     =     0.9883601316     ;
	tripDist   [     298     ]     =     0.9884765255     ;
	tripDist   [     299     ]     =     0.9885929194     ;
	tripDist   [     300     ]     =     0.9887093134     ;
	tripDist   [     301     ]     =     0.9887468476     ;
	tripDist   [     302     ]     =     0.9887843819     ;
	tripDist   [     303     ]     =     0.9888219161     ;
	tripDist   [     304     ]     =     0.9888594504     ;
	tripDist   [     305     ]     =     0.9888969846     ;
	tripDist   [     306     ]     =     0.9889345189     ;
	tripDist   [     307     ]     =     0.9889720531     ;
	tripDist   [     308     ]     =     0.9890095874     ;
	tripDist   [     309     ]     =     0.9890471216     ;
	tripDist   [     310     ]     =     0.9890846559     ;
	tripDist   [     311     ]     =     0.9891221901     ;
	tripDist   [     312     ]     =     0.9891597244     ;
	tripDist   [     313     ]     =     0.9891972586     ;
	tripDist   [     314     ]     =     0.9892347929     ;
	tripDist   [     315     ]     =     0.9892723271     ;
	tripDist   [     316     ]     =     0.9893098614     ;
	tripDist   [     317     ]     =     0.9893473956     ;
	tripDist   [     318     ]     =     0.9893849299     ;
	tripDist   [     319     ]     =     0.9894224641     ;
	tripDist   [     320     ]     =     0.9894599984     ;
	tripDist   [     321     ]     =     0.9894975326     ;
	tripDist   [     322     ]     =     0.9895350669     ;
	tripDist   [     323     ]     =     0.9895726012     ;
	tripDist   [     324     ]     =     0.9896101354     ;
	tripDist   [     325     ]     =     0.9896476697     ;
	tripDist   [     326     ]     =     0.9896852039     ;
	tripDist   [     327     ]     =     0.9897227382     ;
	tripDist   [     328     ]     =     0.9897602724     ;
	tripDist   [     329     ]     =     0.9897978067     ;
	tripDist   [     330     ]     =     0.9898353409     ;
	tripDist   [     331     ]     =     0.9898728752     ;
	tripDist   [     332     ]     =     0.9899104094     ;
	tripDist   [     333     ]     =     0.9899479437     ;
	tripDist   [     334     ]     =     0.9899854779     ;
	tripDist   [     335     ]     =     0.9900230122     ;
	tripDist   [     336     ]     =     0.9900605464     ;
	tripDist   [     337     ]     =     0.9900980807     ;
	tripDist   [     338     ]     =     0.9901356149     ;
	tripDist   [     339     ]     =     0.9901731492     ;
	tripDist   [     340     ]     =     0.9902106834     ;
	tripDist   [     341     ]     =     0.9902482177     ;
	tripDist   [     342     ]     =     0.9902857519     ;
	tripDist   [     343     ]     =     0.9903232862     ;
	tripDist   [     344     ]     =     0.9903608204     ;
	tripDist   [     345     ]     =     0.9903983547     ;
	tripDist   [     346     ]     =     0.9904358889     ;
	tripDist   [     347     ]     =     0.9904734232     ;
	tripDist   [     348     ]     =     0.9905109574     ;
	tripDist   [     349     ]     =     0.9905484917     ;
	tripDist   [     350     ]     =     0.990586026     ;
	tripDist   [     351     ]     =     0.9906235602     ;
	tripDist   [     352     ]     =     0.9906610945     ;
	tripDist   [     353     ]     =     0.9906986287     ;
	tripDist   [     354     ]     =     0.990736163     ;
	tripDist   [     355     ]     =     0.9907736972     ;
	tripDist   [     356     ]     =     0.9908112315     ;
	tripDist   [     357     ]     =     0.9908487657     ;
	tripDist   [     358     ]     =     0.9908863     ;
	tripDist   [     359     ]     =     0.9909238342     ;
	tripDist   [     360     ]     =     0.9909613685     ;
	tripDist   [     361     ]     =     0.9909989027     ;
	tripDist   [     362     ]     =     0.991036437     ;
	tripDist   [     363     ]     =     0.9910739712     ;
	tripDist   [     364     ]     =     0.9911115055     ;
	tripDist   [     365     ]     =     0.9911490397     ;
	tripDist   [     366     ]     =     0.991186574     ;
	tripDist   [     367     ]     =     0.9912241082     ;
	tripDist   [     368     ]     =     0.9912616425     ;
	tripDist   [     369     ]     =     0.9912991767     ;
	tripDist   [     370     ]     =     0.991336711     ;
	tripDist   [     371     ]     =     0.9913742452     ;
	tripDist   [     372     ]     =     0.9914117795     ;
	tripDist   [     373     ]     =     0.9914493137     ;
	tripDist   [     374     ]     =     0.991486848     ;
	tripDist   [     375     ]     =     0.9915243823     ;
	tripDist   [     376     ]     =     0.9915619165     ;
	tripDist   [     377     ]     =     0.9915994508     ;
	tripDist   [     378     ]     =     0.991636985     ;
	tripDist   [     379     ]     =     0.9916745193     ;
	tripDist   [     380     ]     =     0.9917120535     ;
	tripDist   [     381     ]     =     0.9917495878     ;
	tripDist   [     382     ]     =     0.991787122     ;
	tripDist   [     383     ]     =     0.9918246563     ;
	tripDist   [     384     ]     =     0.9918621905     ;
	tripDist   [     385     ]     =     0.9918997248     ;
	tripDist   [     386     ]     =     0.991937259     ;
	tripDist   [     387     ]     =     0.9919747933     ;
	tripDist   [     388     ]     =     0.9920123275     ;
	tripDist   [     389     ]     =     0.9920498618     ;
	tripDist   [     390     ]     =     0.992087396     ;
	tripDist   [     391     ]     =     0.9921249303     ;
	tripDist   [     392     ]     =     0.9921624645     ;
	tripDist   [     393     ]     =     0.9921999988     ;
	tripDist   [     394     ]     =     0.992237533     ;
	tripDist   [     395     ]     =     0.9922750673     ;
	tripDist   [     396     ]     =     0.9923126015     ;
	tripDist   [     397     ]     =     0.9923501358     ;
	tripDist   [     398     ]     =     0.99238767     ;
	tripDist   [     399     ]     =     0.9924252043     ;
	tripDist   [     400     ]     =     0.9924627385     ;
	tripDist   [     401     ]     =     0.9925004249     ;
	tripDist   [     402     ]     =     0.9925381112     ;
	tripDist   [     403     ]     =     0.9925757975     ;
	tripDist   [     404     ]     =     0.9926134838     ;
	tripDist   [     405     ]     =     0.9926511701     ;
	tripDist   [     406     ]     =     0.9926888564     ;
	tripDist   [     407     ]     =     0.9927265427     ;
	tripDist   [     408     ]     =     0.992764229     ;
	tripDist   [     409     ]     =     0.9928019153     ;
	tripDist   [     410     ]     =     0.9928396016     ;
	tripDist   [     411     ]     =     0.9928772879     ;
	tripDist   [     412     ]     =     0.9929149742     ;
	tripDist   [     413     ]     =     0.9929526605     ;
	tripDist   [     414     ]     =     0.9929903469     ;
	tripDist   [     415     ]     =     0.9930280332     ;
	tripDist   [     416     ]     =     0.9930657195     ;
	tripDist   [     417     ]     =     0.9931034058     ;
	tripDist   [     418     ]     =     0.9931410921     ;
	tripDist   [     419     ]     =     0.9931787784     ;
	tripDist   [     420     ]     =     0.9932164647     ;
	tripDist   [     421     ]     =     0.993254151     ;
	tripDist   [     422     ]     =     0.9932918373     ;
	tripDist   [     423     ]     =     0.9933295236     ;
	tripDist   [     424     ]     =     0.9933672099     ;
	tripDist   [     425     ]     =     0.9934048962     ;
	tripDist   [     426     ]     =     0.9934425825     ;
	tripDist   [     427     ]     =     0.9934802688     ;
	tripDist   [     428     ]     =     0.9935179552     ;
	tripDist   [     429     ]     =     0.9935556415     ;
	tripDist   [     430     ]     =     0.9935933278     ;
	tripDist   [     431     ]     =     0.9936310141     ;
	tripDist   [     432     ]     =     0.9936687004     ;
	tripDist   [     433     ]     =     0.9937063867     ;
	tripDist   [     434     ]     =     0.993744073     ;
	tripDist   [     435     ]     =     0.9937817593     ;
	tripDist   [     436     ]     =     0.9938194456     ;
	tripDist   [     437     ]     =     0.9938571319     ;
	tripDist   [     438     ]     =     0.9938948182     ;
	tripDist   [     439     ]     =     0.9939325045     ;
	tripDist   [     440     ]     =     0.9939701908     ;
	tripDist   [     441     ]     =     0.9940078771     ;
	tripDist   [     442     ]     =     0.9940455635     ;
	tripDist   [     443     ]     =     0.9940832498     ;
	tripDist   [     444     ]     =     0.9941209361     ;
	tripDist   [     445     ]     =     0.9941586224     ;
	tripDist   [     446     ]     =     0.9941963087     ;
	tripDist   [     447     ]     =     0.994233995     ;
	tripDist   [     448     ]     =     0.9942716813     ;
	tripDist   [     449     ]     =     0.9943093676     ;
	tripDist   [     450     ]     =     0.9943470539     ;
	tripDist   [     451     ]     =     0.9943847402     ;
	tripDist   [     452     ]     =     0.9944224265     ;
	tripDist   [     453     ]     =     0.9944601128     ;
	tripDist   [     454     ]     =     0.9944977991     ;
	tripDist   [     455     ]     =     0.9945354854     ;
	tripDist   [     456     ]     =     0.9945731718     ;
	tripDist   [     457     ]     =     0.9946108581     ;
	tripDist   [     458     ]     =     0.9946485444     ;
	tripDist   [     459     ]     =     0.9946862307     ;
	tripDist   [     460     ]     =     0.994723917     ;
	tripDist   [     461     ]     =     0.9947616033     ;
	tripDist   [     462     ]     =     0.9947992896     ;
	tripDist   [     463     ]     =     0.9948369759     ;
	tripDist   [     464     ]     =     0.9948746622     ;
	tripDist   [     465     ]     =     0.9949123485     ;
	tripDist   [     466     ]     =     0.9949500348     ;
	tripDist   [     467     ]     =     0.9949877211     ;
	tripDist   [     468     ]     =     0.9950254074     ;
	tripDist   [     469     ]     =     0.9950630938     ;
	tripDist   [     470     ]     =     0.9951007801     ;
	tripDist   [     471     ]     =     0.9951384664     ;
	tripDist   [     472     ]     =     0.9951761527     ;
	tripDist   [     473     ]     =     0.995213839     ;
	tripDist   [     474     ]     =     0.9952515253     ;
	tripDist   [     475     ]     =     0.9952892116     ;
	tripDist   [     476     ]     =     0.9953268979     ;
	tripDist   [     477     ]     =     0.9953645842     ;
	tripDist   [     478     ]     =     0.9954022705     ;
	tripDist   [     479     ]     =     0.9954399568     ;
	tripDist   [     480     ]     =     0.9954776431     ;
	tripDist   [     481     ]     =     0.9955153294     ;
	tripDist   [     482     ]     =     0.9955530157     ;
	tripDist   [     483     ]     =     0.9955907021     ;
	tripDist   [     484     ]     =     0.9956283884     ;
	tripDist   [     485     ]     =     0.9956660747     ;
	tripDist   [     486     ]     =     0.995703761     ;
	tripDist   [     487     ]     =     0.9957414473     ;
	tripDist   [     488     ]     =     0.9957791336     ;
	tripDist   [     489     ]     =     0.9958168199     ;
	tripDist   [     490     ]     =     0.9958545062     ;
	tripDist   [     491     ]     =     0.9958921925     ;
	tripDist   [     492     ]     =     0.9959298788     ;
	tripDist   [     493     ]     =     0.9959675651     ;
	tripDist   [     494     ]     =     0.9960052514     ;
	tripDist   [     495     ]     =     0.9960429377     ;
	tripDist   [     496     ]     =     0.996080624     ;
	tripDist   [     497     ]     =     0.9961183104     ;
	tripDist   [     498     ]     =     0.9961559967     ;
	tripDist   [     499     ]     =     0.996193683     ;
	tripDist   [     500     ]     =     0.9962313693     ;
	tripDist   [     501     ]     =     0.9962690556     ;
	tripDist   [     502     ]     =     0.9963067419     ;
	tripDist   [     503     ]     =     0.9963444282     ;
	tripDist   [     504     ]     =     0.9963821145     ;
	tripDist   [     505     ]     =     0.9964198008     ;
	tripDist   [     506     ]     =     0.9964574871     ;
	tripDist   [     507     ]     =     0.9964951734     ;
	tripDist   [     508     ]     =     0.9965328597     ;
	tripDist   [     509     ]     =     0.996570546     ;
	tripDist   [     510     ]     =     0.9966082323     ;
	tripDist   [     511     ]     =     0.9966459187     ;
	tripDist   [     512     ]     =     0.996683605     ;
	tripDist   [     513     ]     =     0.9967212913     ;
	tripDist   [     514     ]     =     0.9967589776     ;
	tripDist   [     515     ]     =     0.9967966639     ;
	tripDist   [     516     ]     =     0.9968343502     ;
	tripDist   [     517     ]     =     0.9968720365     ;
	tripDist   [     518     ]     =     0.9969097228     ;
	tripDist   [     519     ]     =     0.9969474091     ;
	tripDist   [     520     ]     =     0.9969850954     ;
	tripDist   [     521     ]     =     0.9970227817     ;
	tripDist   [     522     ]     =     0.997060468     ;
	tripDist   [     523     ]     =     0.9970981543     ;
	tripDist   [     524     ]     =     0.9971358406     ;
	tripDist   [     525     ]     =     0.997173527     ;
	tripDist   [     526     ]     =     0.9972112133     ;
	tripDist   [     527     ]     =     0.9972488996     ;
	tripDist   [     528     ]     =     0.9972865859     ;
	tripDist   [     529     ]     =     0.9973242722     ;
	tripDist   [     530     ]     =     0.9973619585     ;
	tripDist   [     531     ]     =     0.9973996448     ;
	tripDist   [     532     ]     =     0.9974373311     ;
	tripDist   [     533     ]     =     0.9974750174     ;
	tripDist   [     534     ]     =     0.9975127037     ;
	tripDist   [     535     ]     =     0.99755039     ;
	tripDist   [     536     ]     =     0.9975880763     ;
	tripDist   [     537     ]     =     0.9976257626     ;
	tripDist   [     538     ]     =     0.997663449     ;
	tripDist   [     539     ]     =     0.9977011353     ;
	tripDist   [     540     ]     =     0.9977388216     ;
	tripDist   [     541     ]     =     0.9977765079     ;
	tripDist   [     542     ]     =     0.9978141942     ;
	tripDist   [     543     ]     =     0.9978518805     ;
	tripDist   [     544     ]     =     0.9978895668     ;
	tripDist   [     545     ]     =     0.9979272531     ;
	tripDist   [     546     ]     =     0.9979649394     ;
	tripDist   [     547     ]     =     0.9980026257     ;
	tripDist   [     548     ]     =     0.998040312     ;
	tripDist   [     549     ]     =     0.9980779983     ;
	tripDist   [     550     ]     =     0.9981156846     ;
	tripDist   [     551     ]     =     0.9981533709     ;
	tripDist   [     552     ]     =     0.9981910573     ;
	tripDist   [     553     ]     =     0.9982287436     ;
	tripDist   [     554     ]     =     0.9982664299     ;
	tripDist   [     555     ]     =     0.9983041162     ;
	tripDist   [     556     ]     =     0.9983418025     ;
	tripDist   [     557     ]     =     0.9983794888     ;
	tripDist   [     558     ]     =     0.9984171751     ;
	tripDist   [     559     ]     =     0.9984548614     ;
	tripDist   [     560     ]     =     0.9984925477     ;
	tripDist   [     561     ]     =     0.998530234     ;
	tripDist   [     562     ]     =     0.9985679203     ;
	tripDist   [     563     ]     =     0.9986056066     ;
	tripDist   [     564     ]     =     0.9986432929     ;
	tripDist   [     565     ]     =     0.9986809792     ;
	tripDist   [     566     ]     =     0.9987186656     ;
	tripDist   [     567     ]     =     0.9987563519     ;
	tripDist   [     568     ]     =     0.9987940382     ;
	tripDist   [     569     ]     =     0.9988317245     ;
	tripDist   [     570     ]     =     0.9988694108     ;
	tripDist   [     571     ]     =     0.9989070971     ;
	tripDist   [     572     ]     =     0.9989447834     ;
	tripDist   [     573     ]     =     0.9989824697     ;
	tripDist   [     574     ]     =     0.999020156     ;
	tripDist   [     575     ]     =     0.9990578423     ;
	tripDist   [     576     ]     =     0.9990955286     ;
	tripDist   [     577     ]     =     0.9991332149     ;
	tripDist   [     578     ]     =     0.9991709012     ;
	tripDist   [     579     ]     =     0.9992085875     ;
	tripDist   [     580     ]     =     0.9992462739     ;
	tripDist   [     581     ]     =     0.9992839602     ;
	tripDist   [     582     ]     =     0.9993216465     ;
	tripDist   [     583     ]     =     0.9993593328     ;
	tripDist   [     584     ]     =     0.9993970191     ;
	tripDist   [     585     ]     =     0.9994347054     ;
	tripDist   [     586     ]     =     0.9994723917     ;
	tripDist   [     587     ]     =     0.999510078     ;
	tripDist   [     588     ]     =     0.9995477643     ;
	tripDist   [     589     ]     =     0.9995854506     ;
	tripDist   [     590     ]     =     0.9996231369     ;
	tripDist   [     591     ]     =     0.9996608232     ;
	tripDist   [     592     ]     =     0.9996985095     ;
	tripDist   [     593     ]     =     0.9997361958     ;
	tripDist   [     594     ]     =     0.9997738822     ;
	tripDist   [     595     ]     =     0.9998115685     ;
	tripDist   [     596     ]     =     0.9998492548     ;
	tripDist   [     597     ]     =     0.9998869411     ;
	tripDist   [     598     ]     =     0.9999246274     ;
	tripDist   [     599     ]     =     0.9999623137     ;
	tripDist   [     600     ]     =     1     ;

	return;
}

void Simulator::setBusinessTripVOTT()
{
	businessTripVOTT = new double [926];
	businessTripVOTT   [     0     ]     =     0     ;
	businessTripVOTT   [     1     ]     =     0.1     ;
	businessTripVOTT   [     2     ]     =     0.2     ;
	businessTripVOTT   [     3     ]     =     0.3     ;
	businessTripVOTT   [     4     ]     =     0.4     ;
	businessTripVOTT   [     5     ]     =     0.5     ;
	businessTripVOTT   [     6     ]     =     0.6     ;
	businessTripVOTT   [     7     ]     =     0.7     ;
	businessTripVOTT   [     8     ]     =     0.8     ;
	businessTripVOTT   [     9     ]     =     0.9     ;
	businessTripVOTT   [     10     ]     =     1     ;
	businessTripVOTT   [     11     ]     =     1.1     ;
	businessTripVOTT   [     12     ]     =     1.2     ;
	businessTripVOTT   [     13     ]     =     1.3     ;
	businessTripVOTT   [     14     ]     =     1.4     ;
	businessTripVOTT   [     15     ]     =     1.5     ;
	businessTripVOTT   [     16     ]     =     1.6     ;
	businessTripVOTT   [     17     ]     =     1.7     ;
	businessTripVOTT   [     18     ]     =     1.8     ;
	businessTripVOTT   [     19     ]     =     1.9     ;
	businessTripVOTT   [     20     ]     =     2     ;
	businessTripVOTT   [     21     ]     =     2.1     ;
	businessTripVOTT   [     22     ]     =     2.2     ;
	businessTripVOTT   [     23     ]     =     2.3     ;
	businessTripVOTT   [     24     ]     =     2.4     ;
	businessTripVOTT   [     25     ]     =     2.5     ;
	businessTripVOTT   [     26     ]     =     2.6     ;
	businessTripVOTT   [     27     ]     =     2.7     ;
	businessTripVOTT   [     28     ]     =     2.8     ;
	businessTripVOTT   [     29     ]     =     2.9     ;
	businessTripVOTT   [     30     ]     =     3     ;
	businessTripVOTT   [     31     ]     =     3.1     ;
	businessTripVOTT   [     32     ]     =     3.2     ;
	businessTripVOTT   [     33     ]     =     3.3     ;
	businessTripVOTT   [     34     ]     =     3.4     ;
	businessTripVOTT   [     35     ]     =     3.5     ;
	businessTripVOTT   [     36     ]     =     3.6     ;
	businessTripVOTT   [     37     ]     =     3.7     ;
	businessTripVOTT   [     38     ]     =     3.8     ;
	businessTripVOTT   [     39     ]     =     3.9     ;
	businessTripVOTT   [     40     ]     =     4     ;
	businessTripVOTT   [     41     ]     =     4.1     ;
	businessTripVOTT   [     42     ]     =     4.2     ;
	businessTripVOTT   [     43     ]     =     4.3     ;
	businessTripVOTT   [     44     ]     =     4.4     ;
	businessTripVOTT   [     45     ]     =     4.5     ;
	businessTripVOTT   [     46     ]     =     4.6     ;
	businessTripVOTT   [     47     ]     =     4.7     ;
	businessTripVOTT   [     48     ]     =     4.8     ;
	businessTripVOTT   [     49     ]     =     4.9     ;
	businessTripVOTT   [     50     ]     =     5     ;
	businessTripVOTT   [     51     ]     =     5.1     ;
	businessTripVOTT   [     52     ]     =     5.2     ;
	businessTripVOTT   [     53     ]     =     5.3     ;
	businessTripVOTT   [     54     ]     =     5.4     ;
	businessTripVOTT   [     55     ]     =     5.5     ;
	businessTripVOTT   [     56     ]     =     5.6     ;
	businessTripVOTT   [     57     ]     =     5.7     ;
	businessTripVOTT   [     58     ]     =     5.8     ;
	businessTripVOTT   [     59     ]     =     5.9     ;
	businessTripVOTT   [     60     ]     =     6     ;
	businessTripVOTT   [     61     ]     =     6.1     ;
	businessTripVOTT   [     62     ]     =     6.2     ;
	businessTripVOTT   [     63     ]     =     6.3     ;
	businessTripVOTT   [     64     ]     =     6.4     ;
	businessTripVOTT   [     65     ]     =     6.5     ;
	businessTripVOTT   [     66     ]     =     6.6     ;
	businessTripVOTT   [     67     ]     =     6.7     ;
	businessTripVOTT   [     68     ]     =     6.8     ;
	businessTripVOTT   [     69     ]     =     6.9     ;
	businessTripVOTT   [     70     ]     =     7     ;
	businessTripVOTT   [     71     ]     =     7.1     ;
	businessTripVOTT   [     72     ]     =     7.2     ;
	businessTripVOTT   [     73     ]     =     7.3     ;
	businessTripVOTT   [     74     ]     =     7.4     ;
	businessTripVOTT   [     75     ]     =     7.5     ;
	businessTripVOTT   [     76     ]     =     7.6     ;
	businessTripVOTT   [     77     ]     =     7.7     ;
	businessTripVOTT   [     78     ]     =     7.8     ;
	businessTripVOTT   [     79     ]     =     7.9     ;
	businessTripVOTT   [     80     ]     =     8     ;
	businessTripVOTT   [     81     ]     =     8.1     ;
	businessTripVOTT   [     82     ]     =     8.2     ;
	businessTripVOTT   [     83     ]     =     8.3     ;
	businessTripVOTT   [     84     ]     =     8.4     ;
	businessTripVOTT   [     85     ]     =     8.5     ;
	businessTripVOTT   [     86     ]     =     8.6     ;
	businessTripVOTT   [     87     ]     =     8.7     ;
	businessTripVOTT   [     88     ]     =     8.8     ;
	businessTripVOTT   [     89     ]     =     8.9     ;
	businessTripVOTT   [     90     ]     =     9     ;
	businessTripVOTT   [     91     ]     =     9.1     ;
	businessTripVOTT   [     92     ]     =     9.2     ;
	businessTripVOTT   [     93     ]     =     9.3     ;
	businessTripVOTT   [     94     ]     =     9.4     ;
	businessTripVOTT   [     95     ]     =     9.5     ;
	businessTripVOTT   [     96     ]     =     9.6     ;
	businessTripVOTT   [     97     ]     =     9.7     ;
	businessTripVOTT   [     98     ]     =     9.8     ;
	businessTripVOTT   [     99     ]     =     9.9     ;
	businessTripVOTT   [     100     ]     =     10     ;
	businessTripVOTT   [     101     ]     =     10.1     ;
	businessTripVOTT   [     102     ]     =     10.2     ;
	businessTripVOTT   [     103     ]     =     10.3     ;
	businessTripVOTT   [     104     ]     =     10.4     ;
	businessTripVOTT   [     105     ]     =     10.5     ;
	businessTripVOTT   [     106     ]     =     10.6     ;
	businessTripVOTT   [     107     ]     =     10.7     ;
	businessTripVOTT   [     108     ]     =     10.8     ;
	businessTripVOTT   [     109     ]     =     10.9     ;
	businessTripVOTT   [     110     ]     =     11     ;
	businessTripVOTT   [     111     ]     =     11.1     ;
	businessTripVOTT   [     112     ]     =     11.2     ;
	businessTripVOTT   [     113     ]     =     11.3     ;
	businessTripVOTT   [     114     ]     =     11.4     ;
	businessTripVOTT   [     115     ]     =     11.5     ;
	businessTripVOTT   [     116     ]     =     11.6     ;
	businessTripVOTT   [     117     ]     =     11.7     ;
	businessTripVOTT   [     118     ]     =     11.8     ;
	businessTripVOTT   [     119     ]     =     11.9     ;
	businessTripVOTT   [     120     ]     =     12     ;
	businessTripVOTT   [     121     ]     =     12.1     ;
	businessTripVOTT   [     122     ]     =     12.2     ;
	businessTripVOTT   [     123     ]     =     12.3     ;
	businessTripVOTT   [     124     ]     =     12.4     ;
	businessTripVOTT   [     125     ]     =     12.5     ;
	businessTripVOTT   [     126     ]     =     12.6     ;
	businessTripVOTT   [     127     ]     =     12.7     ;
	businessTripVOTT   [     128     ]     =     12.8     ;
	businessTripVOTT   [     129     ]     =     12.9     ;
	businessTripVOTT   [     130     ]     =     13     ;
	businessTripVOTT   [     131     ]     =     13.1     ;
	businessTripVOTT   [     132     ]     =     13.2     ;
	businessTripVOTT   [     133     ]     =     13.3     ;
	businessTripVOTT   [     134     ]     =     13.4     ;
	businessTripVOTT   [     135     ]     =     13.5     ;
	businessTripVOTT   [     136     ]     =     13.6     ;
	businessTripVOTT   [     137     ]     =     13.7     ;
	businessTripVOTT   [     138     ]     =     13.8     ;
	businessTripVOTT   [     139     ]     =     13.9     ;
	businessTripVOTT   [     140     ]     =     14     ;
	businessTripVOTT   [     141     ]     =     14.1     ;
	businessTripVOTT   [     142     ]     =     14.2     ;
	businessTripVOTT   [     143     ]     =     14.3     ;
	businessTripVOTT   [     144     ]     =     14.4     ;
	businessTripVOTT   [     145     ]     =     14.5     ;
	businessTripVOTT   [     146     ]     =     14.6     ;
	businessTripVOTT   [     147     ]     =     14.7     ;
	businessTripVOTT   [     148     ]     =     14.8     ;
	businessTripVOTT   [     149     ]     =     14.9     ;
	businessTripVOTT   [     150     ]     =     15     ;
	businessTripVOTT   [     151     ]     =     15.1     ;
	businessTripVOTT   [     152     ]     =     15.2     ;
	businessTripVOTT   [     153     ]     =     15.3     ;
	businessTripVOTT   [     154     ]     =     15.4     ;
	businessTripVOTT   [     155     ]     =     15.5     ;
	businessTripVOTT   [     156     ]     =     15.6     ;
	businessTripVOTT   [     157     ]     =     15.7     ;
	businessTripVOTT   [     158     ]     =     15.8     ;
	businessTripVOTT   [     159     ]     =     15.9     ;
	businessTripVOTT   [     160     ]     =     16     ;
	businessTripVOTT   [     161     ]     =     16.1     ;
	businessTripVOTT   [     162     ]     =     16.2     ;
	businessTripVOTT   [     163     ]     =     16.3     ;
	businessTripVOTT   [     164     ]     =     16.4     ;
	businessTripVOTT   [     165     ]     =     16.5     ;
	businessTripVOTT   [     166     ]     =     16.6     ;
	businessTripVOTT   [     167     ]     =     16.7     ;
	businessTripVOTT   [     168     ]     =     16.8     ;
	businessTripVOTT   [     169     ]     =     16.9     ;
	businessTripVOTT   [     170     ]     =     17     ;
	businessTripVOTT   [     171     ]     =     17.1     ;
	businessTripVOTT   [     172     ]     =     17.2     ;
	businessTripVOTT   [     173     ]     =     17.3     ;
	businessTripVOTT   [     174     ]     =     17.4     ;
	businessTripVOTT   [     175     ]     =     17.5     ;
	businessTripVOTT   [     176     ]     =     17.6     ;
	businessTripVOTT   [     177     ]     =     17.7     ;
	businessTripVOTT   [     178     ]     =     17.8     ;
	businessTripVOTT   [     179     ]     =     17.9     ;
	businessTripVOTT   [     180     ]     =     18     ;
	businessTripVOTT   [     181     ]     =     18.1     ;
	businessTripVOTT   [     182     ]     =     18.2     ;
	businessTripVOTT   [     183     ]     =     18.3     ;
	businessTripVOTT   [     184     ]     =     18.4     ;
	businessTripVOTT   [     185     ]     =     18.5     ;
	businessTripVOTT   [     186     ]     =     18.6     ;
	businessTripVOTT   [     187     ]     =     18.7     ;
	businessTripVOTT   [     188     ]     =     18.8     ;
	businessTripVOTT   [     189     ]     =     18.9     ;
	businessTripVOTT   [     190     ]     =     19     ;
	businessTripVOTT   [     191     ]     =     19.1     ;
	businessTripVOTT   [     192     ]     =     19.2     ;
	businessTripVOTT   [     193     ]     =     19.3     ;
	businessTripVOTT   [     194     ]     =     19.4     ;
	businessTripVOTT   [     195     ]     =     19.5     ;
	businessTripVOTT   [     196     ]     =     19.6     ;
	businessTripVOTT   [     197     ]     =     19.7     ;
	businessTripVOTT   [     198     ]     =     19.8     ;
	businessTripVOTT   [     199     ]     =     19.9     ;
	businessTripVOTT   [     200     ]     =     20     ;
	businessTripVOTT   [     201     ]     =     20.1     ;
	businessTripVOTT   [     202     ]     =     20.2     ;
	businessTripVOTT   [     203     ]     =     20.3     ;
	businessTripVOTT   [     204     ]     =     20.4     ;
	businessTripVOTT   [     205     ]     =     20.5     ;
	businessTripVOTT   [     206     ]     =     20.6     ;
	businessTripVOTT   [     207     ]     =     20.7     ;
	businessTripVOTT   [     208     ]     =     20.8     ;
	businessTripVOTT   [     209     ]     =     20.9     ;
	businessTripVOTT   [     210     ]     =     21     ;
	businessTripVOTT   [     211     ]     =     21.1     ;
	businessTripVOTT   [     212     ]     =     21.2     ;
	businessTripVOTT   [     213     ]     =     21.3     ;
	businessTripVOTT   [     214     ]     =     21.4     ;
	businessTripVOTT   [     215     ]     =     21.5     ;
	businessTripVOTT   [     216     ]     =     21.6     ;
	businessTripVOTT   [     217     ]     =     21.7     ;
	businessTripVOTT   [     218     ]     =     21.8     ;
	businessTripVOTT   [     219     ]     =     21.9     ;
	businessTripVOTT   [     220     ]     =     22     ;
	businessTripVOTT   [     221     ]     =     22.1     ;
	businessTripVOTT   [     222     ]     =     22.2     ;
	businessTripVOTT   [     223     ]     =     22.3     ;
	businessTripVOTT   [     224     ]     =     22.4     ;
	businessTripVOTT   [     225     ]     =     22.5     ;
	businessTripVOTT   [     226     ]     =     22.6     ;
	businessTripVOTT   [     227     ]     =     22.7     ;
	businessTripVOTT   [     228     ]     =     22.8     ;
	businessTripVOTT   [     229     ]     =     22.9     ;
	businessTripVOTT   [     230     ]     =     23     ;
	businessTripVOTT   [     231     ]     =     23.1     ;
	businessTripVOTT   [     232     ]     =     23.2     ;
	businessTripVOTT   [     233     ]     =     23.3     ;
	businessTripVOTT   [     234     ]     =     23.4     ;
	businessTripVOTT   [     235     ]     =     23.5     ;
	businessTripVOTT   [     236     ]     =     23.6     ;
	businessTripVOTT   [     237     ]     =     23.7     ;
	businessTripVOTT   [     238     ]     =     23.8     ;
	businessTripVOTT   [     239     ]     =     23.9     ;
	businessTripVOTT   [     240     ]     =     24     ;
	businessTripVOTT   [     241     ]     =     24.1     ;
	businessTripVOTT   [     242     ]     =     24.2     ;
	businessTripVOTT   [     243     ]     =     24.3     ;
	businessTripVOTT   [     244     ]     =     24.4     ;
	businessTripVOTT   [     245     ]     =     24.5     ;
	businessTripVOTT   [     246     ]     =     24.6     ;
	businessTripVOTT   [     247     ]     =     24.7     ;
	businessTripVOTT   [     248     ]     =     24.8     ;
	businessTripVOTT   [     249     ]     =     24.9     ;
	businessTripVOTT   [     250     ]     =     25     ;
	businessTripVOTT   [     251     ]     =     25.1     ;
	businessTripVOTT   [     252     ]     =     25.2     ;
	businessTripVOTT   [     253     ]     =     25.3     ;
	businessTripVOTT   [     254     ]     =     25.4     ;
	businessTripVOTT   [     255     ]     =     25.5     ;
	businessTripVOTT   [     256     ]     =     25.6     ;
	businessTripVOTT   [     257     ]     =     25.7     ;
	businessTripVOTT   [     258     ]     =     25.8     ;
	businessTripVOTT   [     259     ]     =     25.9     ;
	businessTripVOTT   [     260     ]     =     26     ;
	businessTripVOTT   [     261     ]     =     26.1     ;
	businessTripVOTT   [     262     ]     =     26.2     ;
	businessTripVOTT   [     263     ]     =     26.3     ;
	businessTripVOTT   [     264     ]     =     26.4     ;
	businessTripVOTT   [     265     ]     =     26.5     ;
	businessTripVOTT   [     266     ]     =     26.6     ;
	businessTripVOTT   [     267     ]     =     26.7     ;
	businessTripVOTT   [     268     ]     =     26.8     ;
	businessTripVOTT   [     269     ]     =     26.9     ;
	businessTripVOTT   [     270     ]     =     27     ;
	businessTripVOTT   [     271     ]     =     27.1     ;
	businessTripVOTT   [     272     ]     =     27.2     ;
	businessTripVOTT   [     273     ]     =     27.3     ;
	businessTripVOTT   [     274     ]     =     27.4     ;
	businessTripVOTT   [     275     ]     =     27.5     ;
	businessTripVOTT   [     276     ]     =     27.6     ;
	businessTripVOTT   [     277     ]     =     27.7     ;
	businessTripVOTT   [     278     ]     =     27.8     ;
	businessTripVOTT   [     279     ]     =     27.9     ;
	businessTripVOTT   [     280     ]     =     28     ;
	businessTripVOTT   [     281     ]     =     28.1     ;
	businessTripVOTT   [     282     ]     =     28.2     ;
	businessTripVOTT   [     283     ]     =     28.3     ;
	businessTripVOTT   [     284     ]     =     28.4     ;
	businessTripVOTT   [     285     ]     =     28.5     ;
	businessTripVOTT   [     286     ]     =     28.6     ;
	businessTripVOTT   [     287     ]     =     28.7     ;
	businessTripVOTT   [     288     ]     =     28.8     ;
	businessTripVOTT   [     289     ]     =     28.9     ;
	businessTripVOTT   [     290     ]     =     29     ;
	businessTripVOTT   [     291     ]     =     29.1     ;
	businessTripVOTT   [     292     ]     =     29.2     ;
	businessTripVOTT   [     293     ]     =     29.3     ;
	businessTripVOTT   [     294     ]     =     29.4     ;
	businessTripVOTT   [     295     ]     =     29.5     ;
	businessTripVOTT   [     296     ]     =     29.6     ;
	businessTripVOTT   [     297     ]     =     29.7     ;
	businessTripVOTT   [     298     ]     =     29.8     ;
	businessTripVOTT   [     299     ]     =     29.9     ;
	businessTripVOTT   [     300     ]     =     30     ;
	businessTripVOTT   [     301     ]     =     30.1     ;
	businessTripVOTT   [     302     ]     =     30.2     ;
	businessTripVOTT   [     303     ]     =     30.3     ;
	businessTripVOTT   [     304     ]     =     30.4     ;
	businessTripVOTT   [     305     ]     =     30.5     ;
	businessTripVOTT   [     306     ]     =     30.6     ;
	businessTripVOTT   [     307     ]     =     30.7     ;
	businessTripVOTT   [     308     ]     =     30.8     ;
	businessTripVOTT   [     309     ]     =     30.9     ;
	businessTripVOTT   [     310     ]     =     31     ;
	businessTripVOTT   [     311     ]     =     31.1     ;
	businessTripVOTT   [     312     ]     =     31.2     ;
	businessTripVOTT   [     313     ]     =     31.3     ;
	businessTripVOTT   [     314     ]     =     31.4     ;
	businessTripVOTT   [     315     ]     =     31.5     ;
	businessTripVOTT   [     316     ]     =     31.6     ;
	businessTripVOTT   [     317     ]     =     31.7     ;
	businessTripVOTT   [     318     ]     =     31.8     ;
	businessTripVOTT   [     319     ]     =     31.9     ;
	businessTripVOTT   [     320     ]     =     32     ;
	businessTripVOTT   [     321     ]     =     32.1     ;
	businessTripVOTT   [     322     ]     =     32.2     ;
	businessTripVOTT   [     323     ]     =     32.3     ;
	businessTripVOTT   [     324     ]     =     32.4     ;
	businessTripVOTT   [     325     ]     =     32.5     ;
	businessTripVOTT   [     326     ]     =     32.6     ;
	businessTripVOTT   [     327     ]     =     32.7     ;
	businessTripVOTT   [     328     ]     =     32.8     ;
	businessTripVOTT   [     329     ]     =     32.9     ;
	businessTripVOTT   [     330     ]     =     33     ;
	businessTripVOTT   [     331     ]     =     33.1     ;
	businessTripVOTT   [     332     ]     =     33.2     ;
	businessTripVOTT   [     333     ]     =     33.3     ;
	businessTripVOTT   [     334     ]     =     33.4     ;
	businessTripVOTT   [     335     ]     =     33.5     ;
	businessTripVOTT   [     336     ]     =     33.6     ;
	businessTripVOTT   [     337     ]     =     33.7     ;
	businessTripVOTT   [     338     ]     =     33.8     ;
	businessTripVOTT   [     339     ]     =     33.9     ;
	businessTripVOTT   [     340     ]     =     34     ;
	businessTripVOTT   [     341     ]     =     34.1     ;
	businessTripVOTT   [     342     ]     =     34.2     ;
	businessTripVOTT   [     343     ]     =     34.3     ;
	businessTripVOTT   [     344     ]     =     34.4     ;
	businessTripVOTT   [     345     ]     =     34.5     ;
	businessTripVOTT   [     346     ]     =     34.6     ;
	businessTripVOTT   [     347     ]     =     34.7     ;
	businessTripVOTT   [     348     ]     =     34.8     ;
	businessTripVOTT   [     349     ]     =     34.9     ;
	businessTripVOTT   [     350     ]     =     35     ;
	businessTripVOTT   [     351     ]     =     35.1     ;
	businessTripVOTT   [     352     ]     =     35.2     ;
	businessTripVOTT   [     353     ]     =     35.3     ;
	businessTripVOTT   [     354     ]     =     35.4     ;
	businessTripVOTT   [     355     ]     =     35.5     ;
	businessTripVOTT   [     356     ]     =     35.6     ;
	businessTripVOTT   [     357     ]     =     35.7     ;
	businessTripVOTT   [     358     ]     =     35.8     ;
	businessTripVOTT   [     359     ]     =     35.9     ;
	businessTripVOTT   [     360     ]     =     36     ;
	businessTripVOTT   [     361     ]     =     36.1     ;
	businessTripVOTT   [     362     ]     =     36.2     ;
	businessTripVOTT   [     363     ]     =     36.3     ;
	businessTripVOTT   [     364     ]     =     36.4     ;
	businessTripVOTT   [     365     ]     =     36.5     ;
	businessTripVOTT   [     366     ]     =     36.6     ;
	businessTripVOTT   [     367     ]     =     36.7     ;
	businessTripVOTT   [     368     ]     =     36.8     ;
	businessTripVOTT   [     369     ]     =     36.9     ;
	businessTripVOTT   [     370     ]     =     37     ;
	businessTripVOTT   [     371     ]     =     37.1     ;
	businessTripVOTT   [     372     ]     =     37.2     ;
	businessTripVOTT   [     373     ]     =     37.3     ;
	businessTripVOTT   [     374     ]     =     37.4     ;
	businessTripVOTT   [     375     ]     =     37.5     ;
	businessTripVOTT   [     376     ]     =     37.6     ;
	businessTripVOTT   [     377     ]     =     37.7     ;
	businessTripVOTT   [     378     ]     =     37.8     ;
	businessTripVOTT   [     379     ]     =     37.9     ;
	businessTripVOTT   [     380     ]     =     38     ;
	businessTripVOTT   [     381     ]     =     38.1     ;
	businessTripVOTT   [     382     ]     =     38.2     ;
	businessTripVOTT   [     383     ]     =     38.3     ;
	businessTripVOTT   [     384     ]     =     38.4     ;
	businessTripVOTT   [     385     ]     =     38.5     ;
	businessTripVOTT   [     386     ]     =     38.6     ;
	businessTripVOTT   [     387     ]     =     38.7     ;
	businessTripVOTT   [     388     ]     =     38.8     ;
	businessTripVOTT   [     389     ]     =     38.9     ;
	businessTripVOTT   [     390     ]     =     39     ;
	businessTripVOTT   [     391     ]     =     39.1     ;
	businessTripVOTT   [     392     ]     =     39.2     ;
	businessTripVOTT   [     393     ]     =     39.3     ;
	businessTripVOTT   [     394     ]     =     39.4     ;
	businessTripVOTT   [     395     ]     =     39.5     ;
	businessTripVOTT   [     396     ]     =     39.6     ;
	businessTripVOTT   [     397     ]     =     39.7     ;
	businessTripVOTT   [     398     ]     =     39.8     ;
	businessTripVOTT   [     399     ]     =     39.9     ;
	businessTripVOTT   [     400     ]     =     40     ;
	businessTripVOTT   [     401     ]     =     40.1     ;
	businessTripVOTT   [     402     ]     =     40.2     ;
	businessTripVOTT   [     403     ]     =     40.3     ;
	businessTripVOTT   [     404     ]     =     40.4     ;
	businessTripVOTT   [     405     ]     =     40.5     ;
	businessTripVOTT   [     406     ]     =     40.6     ;
	businessTripVOTT   [     407     ]     =     40.7     ;
	businessTripVOTT   [     408     ]     =     40.8     ;
	businessTripVOTT   [     409     ]     =     40.9     ;
	businessTripVOTT   [     410     ]     =     41     ;
	businessTripVOTT   [     411     ]     =     41.1     ;
	businessTripVOTT   [     412     ]     =     41.2     ;
	businessTripVOTT   [     413     ]     =     41.3     ;
	businessTripVOTT   [     414     ]     =     41.4     ;
	businessTripVOTT   [     415     ]     =     41.5     ;
	businessTripVOTT   [     416     ]     =     41.6     ;
	businessTripVOTT   [     417     ]     =     41.7     ;
	businessTripVOTT   [     418     ]     =     41.8     ;
	businessTripVOTT   [     419     ]     =     41.9     ;
	businessTripVOTT   [     420     ]     =     42     ;
	businessTripVOTT   [     421     ]     =     42.1     ;
	businessTripVOTT   [     422     ]     =     42.2     ;
	businessTripVOTT   [     423     ]     =     42.3     ;
	businessTripVOTT   [     424     ]     =     42.4     ;
	businessTripVOTT   [     425     ]     =     42.5     ;
	businessTripVOTT   [     426     ]     =     42.6     ;
	businessTripVOTT   [     427     ]     =     42.7     ;
	businessTripVOTT   [     428     ]     =     42.8     ;
	businessTripVOTT   [     429     ]     =     42.9     ;
	businessTripVOTT   [     430     ]     =     43     ;
	businessTripVOTT   [     431     ]     =     43.1     ;
	businessTripVOTT   [     432     ]     =     43.2     ;
	businessTripVOTT   [     433     ]     =     43.3     ;
	businessTripVOTT   [     434     ]     =     43.4     ;
	businessTripVOTT   [     435     ]     =     43.5     ;
	businessTripVOTT   [     436     ]     =     43.6     ;
	businessTripVOTT   [     437     ]     =     43.7     ;
	businessTripVOTT   [     438     ]     =     43.8     ;
	businessTripVOTT   [     439     ]     =     43.9     ;
	businessTripVOTT   [     440     ]     =     44     ;
	businessTripVOTT   [     441     ]     =     44.1     ;
	businessTripVOTT   [     442     ]     =     44.2     ;
	businessTripVOTT   [     443     ]     =     44.3     ;
	businessTripVOTT   [     444     ]     =     44.4     ;
	businessTripVOTT   [     445     ]     =     44.5     ;
	businessTripVOTT   [     446     ]     =     44.6     ;
	businessTripVOTT   [     447     ]     =     44.7     ;
	businessTripVOTT   [     448     ]     =     44.8     ;
	businessTripVOTT   [     449     ]     =     44.9     ;
	businessTripVOTT   [     450     ]     =     45     ;
	businessTripVOTT   [     451     ]     =     45.1     ;
	businessTripVOTT   [     452     ]     =     45.2     ;
	businessTripVOTT   [     453     ]     =     45.3     ;
	businessTripVOTT   [     454     ]     =     45.4     ;
	businessTripVOTT   [     455     ]     =     45.5     ;
	businessTripVOTT   [     456     ]     =     45.6     ;
	businessTripVOTT   [     457     ]     =     45.7     ;
	businessTripVOTT   [     458     ]     =     45.8     ;
	businessTripVOTT   [     459     ]     =     45.9     ;
	businessTripVOTT   [     460     ]     =     46     ;
	businessTripVOTT   [     461     ]     =     46.1     ;
	businessTripVOTT   [     462     ]     =     46.2     ;
	businessTripVOTT   [     463     ]     =     46.3     ;
	businessTripVOTT   [     464     ]     =     46.4     ;
	businessTripVOTT   [     465     ]     =     46.5     ;
	businessTripVOTT   [     466     ]     =     46.6     ;
	businessTripVOTT   [     467     ]     =     46.7     ;
	businessTripVOTT   [     468     ]     =     46.8     ;
	businessTripVOTT   [     469     ]     =     46.9     ;
	businessTripVOTT   [     470     ]     =     47     ;
	businessTripVOTT   [     471     ]     =     47.1     ;
	businessTripVOTT   [     472     ]     =     47.2     ;
	businessTripVOTT   [     473     ]     =     47.3     ;
	businessTripVOTT   [     474     ]     =     47.4     ;
	businessTripVOTT   [     475     ]     =     47.5     ;
	businessTripVOTT   [     476     ]     =     47.6     ;
	businessTripVOTT   [     477     ]     =     47.7     ;
	businessTripVOTT   [     478     ]     =     47.8     ;
	businessTripVOTT   [     479     ]     =     47.9     ;
	businessTripVOTT   [     480     ]     =     48     ;
	businessTripVOTT   [     481     ]     =     48.1     ;
	businessTripVOTT   [     482     ]     =     48.2     ;
	businessTripVOTT   [     483     ]     =     48.3     ;
	businessTripVOTT   [     484     ]     =     48.4     ;
	businessTripVOTT   [     485     ]     =     48.5     ;
	businessTripVOTT   [     486     ]     =     48.6     ;
	businessTripVOTT   [     487     ]     =     48.7     ;
	businessTripVOTT   [     488     ]     =     48.8     ;
	businessTripVOTT   [     489     ]     =     48.9     ;
	businessTripVOTT   [     490     ]     =     49     ;
	businessTripVOTT   [     491     ]     =     49.1     ;
	businessTripVOTT   [     492     ]     =     49.2     ;
	businessTripVOTT   [     493     ]     =     49.3     ;
	businessTripVOTT   [     494     ]     =     49.4     ;
	businessTripVOTT   [     495     ]     =     49.5     ;
	businessTripVOTT   [     496     ]     =     49.6     ;
	businessTripVOTT   [     497     ]     =     49.7     ;
	businessTripVOTT   [     498     ]     =     49.8     ;
	businessTripVOTT   [     499     ]     =     49.9     ;
	businessTripVOTT   [     500     ]     =     50     ;
	businessTripVOTT   [     501     ]     =     50.1     ;
	businessTripVOTT   [     502     ]     =     50.2     ;
	businessTripVOTT   [     503     ]     =     50.3     ;
	businessTripVOTT   [     504     ]     =     50.4     ;
	businessTripVOTT   [     505     ]     =     50.5     ;
	businessTripVOTT   [     506     ]     =     50.6     ;
	businessTripVOTT   [     507     ]     =     50.7     ;
	businessTripVOTT   [     508     ]     =     50.8     ;
	businessTripVOTT   [     509     ]     =     50.9     ;
	businessTripVOTT   [     510     ]     =     51     ;
	businessTripVOTT   [     511     ]     =     51.1     ;
	businessTripVOTT   [     512     ]     =     51.2     ;
	businessTripVOTT   [     513     ]     =     51.3     ;
	businessTripVOTT   [     514     ]     =     51.4     ;
	businessTripVOTT   [     515     ]     =     51.5     ;
	businessTripVOTT   [     516     ]     =     51.6     ;
	businessTripVOTT   [     517     ]     =     51.7     ;
	businessTripVOTT   [     518     ]     =     51.8     ;
	businessTripVOTT   [     519     ]     =     51.9     ;
	businessTripVOTT   [     520     ]     =     52     ;
	businessTripVOTT   [     521     ]     =     52.1     ;
	businessTripVOTT   [     522     ]     =     52.2     ;
	businessTripVOTT   [     523     ]     =     52.3     ;
	businessTripVOTT   [     524     ]     =     52.4     ;
	businessTripVOTT   [     525     ]     =     52.5     ;
	businessTripVOTT   [     526     ]     =     52.6     ;
	businessTripVOTT   [     527     ]     =     52.7     ;
	businessTripVOTT   [     528     ]     =     52.8     ;
	businessTripVOTT   [     529     ]     =     52.9     ;
	businessTripVOTT   [     530     ]     =     53     ;
	businessTripVOTT   [     531     ]     =     53.1     ;
	businessTripVOTT   [     532     ]     =     53.2     ;
	businessTripVOTT   [     533     ]     =     53.3     ;
	businessTripVOTT   [     534     ]     =     53.4     ;
	businessTripVOTT   [     535     ]     =     53.5     ;
	businessTripVOTT   [     536     ]     =     53.6     ;
	businessTripVOTT   [     537     ]     =     53.7     ;
	businessTripVOTT   [     538     ]     =     53.8     ;
	businessTripVOTT   [     539     ]     =     53.9     ;
	businessTripVOTT   [     540     ]     =     54     ;
	businessTripVOTT   [     541     ]     =     54.1     ;
	businessTripVOTT   [     542     ]     =     54.2     ;
	businessTripVOTT   [     543     ]     =     54.3     ;
	businessTripVOTT   [     544     ]     =     54.4     ;
	businessTripVOTT   [     545     ]     =     54.5     ;
	businessTripVOTT   [     546     ]     =     54.6     ;
	businessTripVOTT   [     547     ]     =     54.7     ;
	businessTripVOTT   [     548     ]     =     54.8     ;
	businessTripVOTT   [     549     ]     =     54.9     ;
	businessTripVOTT   [     550     ]     =     55     ;
	businessTripVOTT   [     551     ]     =     55.1     ;
	businessTripVOTT   [     552     ]     =     55.2     ;
	businessTripVOTT   [     553     ]     =     55.3     ;
	businessTripVOTT   [     554     ]     =     55.4     ;
	businessTripVOTT   [     555     ]     =     55.5     ;
	businessTripVOTT   [     556     ]     =     55.6     ;
	businessTripVOTT   [     557     ]     =     55.7     ;
	businessTripVOTT   [     558     ]     =     55.8     ;
	businessTripVOTT   [     559     ]     =     55.9     ;
	businessTripVOTT   [     560     ]     =     56     ;
	businessTripVOTT   [     561     ]     =     56.1     ;
	businessTripVOTT   [     562     ]     =     56.2     ;
	businessTripVOTT   [     563     ]     =     56.3     ;
	businessTripVOTT   [     564     ]     =     56.4     ;
	businessTripVOTT   [     565     ]     =     56.5     ;
	businessTripVOTT   [     566     ]     =     56.6     ;
	businessTripVOTT   [     567     ]     =     56.7     ;
	businessTripVOTT   [     568     ]     =     56.8     ;
	businessTripVOTT   [     569     ]     =     56.9     ;
	businessTripVOTT   [     570     ]     =     57     ;
	businessTripVOTT   [     571     ]     =     57.1     ;
	businessTripVOTT   [     572     ]     =     57.2     ;
	businessTripVOTT   [     573     ]     =     57.3     ;
	businessTripVOTT   [     574     ]     =     57.4     ;
	businessTripVOTT   [     575     ]     =     57.5     ;
	businessTripVOTT   [     576     ]     =     57.6     ;
	businessTripVOTT   [     577     ]     =     57.7     ;
	businessTripVOTT   [     578     ]     =     57.8     ;
	businessTripVOTT   [     579     ]     =     57.9     ;
	businessTripVOTT   [     580     ]     =     58     ;
	businessTripVOTT   [     581     ]     =     58.1     ;
	businessTripVOTT   [     582     ]     =     58.2     ;
	businessTripVOTT   [     583     ]     =     58.3     ;
	businessTripVOTT   [     584     ]     =     58.4     ;
	businessTripVOTT   [     585     ]     =     58.5     ;
	businessTripVOTT   [     586     ]     =     58.6     ;
	businessTripVOTT   [     587     ]     =     58.7     ;
	businessTripVOTT   [     588     ]     =     58.8     ;
	businessTripVOTT   [     589     ]     =     58.9     ;
	businessTripVOTT   [     590     ]     =     59     ;
	businessTripVOTT   [     591     ]     =     59.1     ;
	businessTripVOTT   [     592     ]     =     59.2     ;
	businessTripVOTT   [     593     ]     =     59.3     ;
	businessTripVOTT   [     594     ]     =     59.4     ;
	businessTripVOTT   [     595     ]     =     59.5     ;
	businessTripVOTT   [     596     ]     =     59.6     ;
	businessTripVOTT   [     597     ]     =     59.7     ;
	businessTripVOTT   [     598     ]     =     59.8     ;
	businessTripVOTT   [     599     ]     =     59.9     ;
	businessTripVOTT   [     600     ]     =     60     ;
	businessTripVOTT   [     601     ]     =     60.1     ;
	businessTripVOTT   [     602     ]     =     60.2     ;
	businessTripVOTT   [     603     ]     =     60.3     ;
	businessTripVOTT   [     604     ]     =     60.4     ;
	businessTripVOTT   [     605     ]     =     60.5     ;
	businessTripVOTT   [     606     ]     =     60.6     ;
	businessTripVOTT   [     607     ]     =     60.7     ;
	businessTripVOTT   [     608     ]     =     60.8     ;
	businessTripVOTT   [     609     ]     =     60.9     ;
	businessTripVOTT   [     610     ]     =     61     ;
	businessTripVOTT   [     611     ]     =     61.1     ;
	businessTripVOTT   [     612     ]     =     61.2     ;
	businessTripVOTT   [     613     ]     =     61.3     ;
	businessTripVOTT   [     614     ]     =     61.4     ;
	businessTripVOTT   [     615     ]     =     61.5     ;
	businessTripVOTT   [     616     ]     =     61.6     ;
	businessTripVOTT   [     617     ]     =     61.7     ;
	businessTripVOTT   [     618     ]     =     61.8     ;
	businessTripVOTT   [     619     ]     =     61.9     ;
	businessTripVOTT   [     620     ]     =     62     ;
	businessTripVOTT   [     621     ]     =     62.1     ;
	businessTripVOTT   [     622     ]     =     62.2     ;
	businessTripVOTT   [     623     ]     =     62.3     ;
	businessTripVOTT   [     624     ]     =     62.4     ;
	businessTripVOTT   [     625     ]     =     62.5     ;
	businessTripVOTT   [     626     ]     =     62.6     ;
	businessTripVOTT   [     627     ]     =     62.7     ;
	businessTripVOTT   [     628     ]     =     62.8     ;
	businessTripVOTT   [     629     ]     =     62.9     ;
	businessTripVOTT   [     630     ]     =     63     ;
	businessTripVOTT   [     631     ]     =     63.1     ;
	businessTripVOTT   [     632     ]     =     63.2     ;
	businessTripVOTT   [     633     ]     =     63.3     ;
	businessTripVOTT   [     634     ]     =     63.4     ;
	businessTripVOTT   [     635     ]     =     63.5     ;
	businessTripVOTT   [     636     ]     =     63.6     ;
	businessTripVOTT   [     637     ]     =     63.7     ;
	businessTripVOTT   [     638     ]     =     63.8     ;
	businessTripVOTT   [     639     ]     =     63.9     ;
	businessTripVOTT   [     640     ]     =     64     ;
	businessTripVOTT   [     641     ]     =     64.1     ;
	businessTripVOTT   [     642     ]     =     64.2     ;
	businessTripVOTT   [     643     ]     =     64.3     ;
	businessTripVOTT   [     644     ]     =     64.4     ;
	businessTripVOTT   [     645     ]     =     64.5     ;
	businessTripVOTT   [     646     ]     =     64.6     ;
	businessTripVOTT   [     647     ]     =     64.7     ;
	businessTripVOTT   [     648     ]     =     64.8     ;
	businessTripVOTT   [     649     ]     =     64.9     ;
	businessTripVOTT   [     650     ]     =     65     ;
	businessTripVOTT   [     651     ]     =     65.1     ;
	businessTripVOTT   [     652     ]     =     65.2     ;
	businessTripVOTT   [     653     ]     =     65.3     ;
	businessTripVOTT   [     654     ]     =     65.4     ;
	businessTripVOTT   [     655     ]     =     65.5     ;
	businessTripVOTT   [     656     ]     =     65.6     ;
	businessTripVOTT   [     657     ]     =     65.7     ;
	businessTripVOTT   [     658     ]     =     65.8     ;
	businessTripVOTT   [     659     ]     =     65.9     ;
	businessTripVOTT   [     660     ]     =     66     ;
	businessTripVOTT   [     661     ]     =     66.1     ;
	businessTripVOTT   [     662     ]     =     66.2     ;
	businessTripVOTT   [     663     ]     =     66.3     ;
	businessTripVOTT   [     664     ]     =     66.4     ;
	businessTripVOTT   [     665     ]     =     66.5     ;
	businessTripVOTT   [     666     ]     =     66.6     ;
	businessTripVOTT   [     667     ]     =     66.7     ;
	businessTripVOTT   [     668     ]     =     66.8     ;
	businessTripVOTT   [     669     ]     =     66.9     ;
	businessTripVOTT   [     670     ]     =     67     ;
	businessTripVOTT   [     671     ]     =     67.1     ;
	businessTripVOTT   [     672     ]     =     67.2     ;
	businessTripVOTT   [     673     ]     =     67.3     ;
	businessTripVOTT   [     674     ]     =     67.4     ;
	businessTripVOTT   [     675     ]     =     67.5     ;
	businessTripVOTT   [     676     ]     =     67.6     ;
	businessTripVOTT   [     677     ]     =     67.7     ;
	businessTripVOTT   [     678     ]     =     67.8     ;
	businessTripVOTT   [     679     ]     =     67.9     ;
	businessTripVOTT   [     680     ]     =     68     ;
	businessTripVOTT   [     681     ]     =     68.1     ;
	businessTripVOTT   [     682     ]     =     68.2     ;
	businessTripVOTT   [     683     ]     =     68.3     ;
	businessTripVOTT   [     684     ]     =     68.4     ;
	businessTripVOTT   [     685     ]     =     68.5     ;
	businessTripVOTT   [     686     ]     =     68.6     ;
	businessTripVOTT   [     687     ]     =     68.7     ;
	businessTripVOTT   [     688     ]     =     68.8     ;
	businessTripVOTT   [     689     ]     =     68.9     ;
	businessTripVOTT   [     690     ]     =     69     ;
	businessTripVOTT   [     691     ]     =     69.1     ;
	businessTripVOTT   [     692     ]     =     69.2     ;
	businessTripVOTT   [     693     ]     =     69.3     ;
	businessTripVOTT   [     694     ]     =     69.4     ;
	businessTripVOTT   [     695     ]     =     69.5     ;
	businessTripVOTT   [     696     ]     =     69.6     ;
	businessTripVOTT   [     697     ]     =     69.7     ;
	businessTripVOTT   [     698     ]     =     69.8     ;
	businessTripVOTT   [     699     ]     =     69.9     ;
	businessTripVOTT   [     700     ]     =     70     ;
	businessTripVOTT   [     701     ]     =     70.1     ;
	businessTripVOTT   [     702     ]     =     70.2     ;
	businessTripVOTT   [     703     ]     =     70.3     ;
	businessTripVOTT   [     704     ]     =     70.4     ;
	businessTripVOTT   [     705     ]     =     70.5     ;
	businessTripVOTT   [     706     ]     =     70.6     ;
	businessTripVOTT   [     707     ]     =     70.7     ;
	businessTripVOTT   [     708     ]     =     70.8     ;
	businessTripVOTT   [     709     ]     =     70.9     ;
	businessTripVOTT   [     710     ]     =     71     ;
	businessTripVOTT   [     711     ]     =     71.1     ;
	businessTripVOTT   [     712     ]     =     71.2     ;
	businessTripVOTT   [     713     ]     =     71.3     ;
	businessTripVOTT   [     714     ]     =     71.4     ;
	businessTripVOTT   [     715     ]     =     71.5     ;
	businessTripVOTT   [     716     ]     =     71.6     ;
	businessTripVOTT   [     717     ]     =     71.7     ;
	businessTripVOTT   [     718     ]     =     71.8     ;
	businessTripVOTT   [     719     ]     =     71.9     ;
	businessTripVOTT   [     720     ]     =     72     ;
	businessTripVOTT   [     721     ]     =     72.1     ;
	businessTripVOTT   [     722     ]     =     72.2     ;
	businessTripVOTT   [     723     ]     =     72.3     ;
	businessTripVOTT   [     724     ]     =     72.4     ;
	businessTripVOTT   [     725     ]     =     72.5     ;
	businessTripVOTT   [     726     ]     =     72.6     ;
	businessTripVOTT   [     727     ]     =     72.7     ;
	businessTripVOTT   [     728     ]     =     72.8     ;
	businessTripVOTT   [     729     ]     =     72.9     ;
	businessTripVOTT   [     730     ]     =     73     ;
	businessTripVOTT   [     731     ]     =     73.1     ;
	businessTripVOTT   [     732     ]     =     73.2     ;
	businessTripVOTT   [     733     ]     =     73.3     ;
	businessTripVOTT   [     734     ]     =     73.4     ;
	businessTripVOTT   [     735     ]     =     73.5     ;
	businessTripVOTT   [     736     ]     =     73.6     ;
	businessTripVOTT   [     737     ]     =     73.7     ;
	businessTripVOTT   [     738     ]     =     73.8     ;
	businessTripVOTT   [     739     ]     =     73.9     ;
	businessTripVOTT   [     740     ]     =     74     ;
	businessTripVOTT   [     741     ]     =     74.1     ;
	businessTripVOTT   [     742     ]     =     74.2     ;
	businessTripVOTT   [     743     ]     =     74.3     ;
	businessTripVOTT   [     744     ]     =     74.4     ;
	businessTripVOTT   [     745     ]     =     74.5     ;
	businessTripVOTT   [     746     ]     =     74.6     ;
	businessTripVOTT   [     747     ]     =     74.7     ;
	businessTripVOTT   [     748     ]     =     74.8     ;
	businessTripVOTT   [     749     ]     =     74.9     ;
	businessTripVOTT   [     750     ]     =     75     ;
	businessTripVOTT   [     751     ]     =     75.1     ;
	businessTripVOTT   [     752     ]     =     75.2     ;
	businessTripVOTT   [     753     ]     =     75.3     ;
	businessTripVOTT   [     754     ]     =     75.4     ;
	businessTripVOTT   [     755     ]     =     75.5     ;
	businessTripVOTT   [     756     ]     =     75.6     ;
	businessTripVOTT   [     757     ]     =     75.7     ;
	businessTripVOTT   [     758     ]     =     75.8     ;
	businessTripVOTT   [     759     ]     =     75.9     ;
	businessTripVOTT   [     760     ]     =     76     ;
	businessTripVOTT   [     761     ]     =     76.1     ;
	businessTripVOTT   [     762     ]     =     76.2     ;
	businessTripVOTT   [     763     ]     =     76.3     ;
	businessTripVOTT   [     764     ]     =     76.4     ;
	businessTripVOTT   [     765     ]     =     76.5     ;
	businessTripVOTT   [     766     ]     =     76.6     ;
	businessTripVOTT   [     767     ]     =     76.7     ;
	businessTripVOTT   [     768     ]     =     76.8     ;
	businessTripVOTT   [     769     ]     =     76.9     ;
	businessTripVOTT   [     770     ]     =     77     ;
	businessTripVOTT   [     771     ]     =     77.1     ;
	businessTripVOTT   [     772     ]     =     77.2     ;
	businessTripVOTT   [     773     ]     =     77.3     ;
	businessTripVOTT   [     774     ]     =     77.4     ;
	businessTripVOTT   [     775     ]     =     77.5     ;
	businessTripVOTT   [     776     ]     =     77.6     ;
	businessTripVOTT   [     777     ]     =     77.7     ;
	businessTripVOTT   [     778     ]     =     77.8     ;
	businessTripVOTT   [     779     ]     =     77.9     ;
	businessTripVOTT   [     780     ]     =     78     ;
	businessTripVOTT   [     781     ]     =     78.1     ;
	businessTripVOTT   [     782     ]     =     78.2     ;
	businessTripVOTT   [     783     ]     =     78.3     ;
	businessTripVOTT   [     784     ]     =     78.4     ;
	businessTripVOTT   [     785     ]     =     78.5     ;
	businessTripVOTT   [     786     ]     =     78.6     ;
	businessTripVOTT   [     787     ]     =     78.7     ;
	businessTripVOTT   [     788     ]     =     78.8     ;
	businessTripVOTT   [     789     ]     =     78.9     ;
	businessTripVOTT   [     790     ]     =     79     ;
	businessTripVOTT   [     791     ]     =     79.1     ;
	businessTripVOTT   [     792     ]     =     79.2     ;
	businessTripVOTT   [     793     ]     =     79.3     ;
	businessTripVOTT   [     794     ]     =     79.4     ;
	businessTripVOTT   [     795     ]     =     79.5     ;
	businessTripVOTT   [     796     ]     =     79.6     ;
	businessTripVOTT   [     797     ]     =     79.7     ;
	businessTripVOTT   [     798     ]     =     79.8     ;
	businessTripVOTT   [     799     ]     =     79.9     ;
	businessTripVOTT   [     800     ]     =     80     ;
	businessTripVOTT   [     801     ]     =     80.1     ;
	businessTripVOTT   [     802     ]     =     80.2     ;
	businessTripVOTT   [     803     ]     =     80.3     ;
	businessTripVOTT   [     804     ]     =     80.4     ;
	businessTripVOTT   [     805     ]     =     80.5     ;
	businessTripVOTT   [     806     ]     =     80.6     ;
	businessTripVOTT   [     807     ]     =     80.7     ;
	businessTripVOTT   [     808     ]     =     80.8     ;
	businessTripVOTT   [     809     ]     =     80.9     ;
	businessTripVOTT   [     810     ]     =     81     ;
	businessTripVOTT   [     811     ]     =     81.1     ;
	businessTripVOTT   [     812     ]     =     81.2     ;
	businessTripVOTT   [     813     ]     =     81.3     ;
	businessTripVOTT   [     814     ]     =     81.4     ;
	businessTripVOTT   [     815     ]     =     81.5     ;
	businessTripVOTT   [     816     ]     =     81.6     ;
	businessTripVOTT   [     817     ]     =     81.7     ;
	businessTripVOTT   [     818     ]     =     81.8     ;
	businessTripVOTT   [     819     ]     =     81.9     ;
	businessTripVOTT   [     820     ]     =     82     ;
	businessTripVOTT   [     821     ]     =     82.1     ;
	businessTripVOTT   [     822     ]     =     82.2     ;
	businessTripVOTT   [     823     ]     =     82.3     ;
	businessTripVOTT   [     824     ]     =     82.4     ;
	businessTripVOTT   [     825     ]     =     82.5     ;
	businessTripVOTT   [     826     ]     =     82.6     ;
	businessTripVOTT   [     827     ]     =     82.7     ;
	businessTripVOTT   [     828     ]     =     82.8     ;
	businessTripVOTT   [     829     ]     =     82.9     ;
	businessTripVOTT   [     830     ]     =     83     ;
	businessTripVOTT   [     831     ]     =     83.1     ;
	businessTripVOTT   [     832     ]     =     83.2     ;
	businessTripVOTT   [     833     ]     =     83.3     ;
	businessTripVOTT   [     834     ]     =     83.4     ;
	businessTripVOTT   [     835     ]     =     83.5     ;
	businessTripVOTT   [     836     ]     =     83.6     ;
	businessTripVOTT   [     837     ]     =     83.7     ;
	businessTripVOTT   [     838     ]     =     83.8     ;
	businessTripVOTT   [     839     ]     =     83.9     ;
	businessTripVOTT   [     840     ]     =     84     ;
	businessTripVOTT   [     841     ]     =     84.1     ;
	businessTripVOTT   [     842     ]     =     84.2     ;
	businessTripVOTT   [     843     ]     =     84.3     ;
	businessTripVOTT   [     844     ]     =     84.4     ;
	businessTripVOTT   [     845     ]     =     84.5     ;
	businessTripVOTT   [     846     ]     =     84.6     ;
	businessTripVOTT   [     847     ]     =     84.7     ;
	businessTripVOTT   [     848     ]     =     84.8     ;
	businessTripVOTT   [     849     ]     =     84.9     ;
	businessTripVOTT   [     850     ]     =     85     ;
	businessTripVOTT   [     851     ]     =     85.1     ;
	businessTripVOTT   [     852     ]     =     85.2     ;
	businessTripVOTT   [     853     ]     =     85.3     ;
	businessTripVOTT   [     854     ]     =     85.4     ;
	businessTripVOTT   [     855     ]     =     85.5     ;
	businessTripVOTT   [     856     ]     =     85.6     ;
	businessTripVOTT   [     857     ]     =     85.7     ;
	businessTripVOTT   [     858     ]     =     85.8     ;
	businessTripVOTT   [     859     ]     =     85.9     ;
	businessTripVOTT   [     860     ]     =     86     ;
	businessTripVOTT   [     861     ]     =     86.1     ;
	businessTripVOTT   [     862     ]     =     86.2     ;
	businessTripVOTT   [     863     ]     =     86.3     ;
	businessTripVOTT   [     864     ]     =     86.4     ;
	businessTripVOTT   [     865     ]     =     86.5     ;
	businessTripVOTT   [     866     ]     =     86.6     ;
	businessTripVOTT   [     867     ]     =     86.7     ;
	businessTripVOTT   [     868     ]     =     86.8     ;
	businessTripVOTT   [     869     ]     =     86.9     ;
	businessTripVOTT   [     870     ]     =     87     ;
	businessTripVOTT   [     871     ]     =     87.1     ;
	businessTripVOTT   [     872     ]     =     87.2     ;
	businessTripVOTT   [     873     ]     =     87.3     ;
	businessTripVOTT   [     874     ]     =     87.4     ;
	businessTripVOTT   [     875     ]     =     87.5     ;
	businessTripVOTT   [     876     ]     =     87.6     ;
	businessTripVOTT   [     877     ]     =     87.7     ;
	businessTripVOTT   [     878     ]     =     87.8     ;
	businessTripVOTT   [     879     ]     =     87.9     ;
	businessTripVOTT   [     880     ]     =     88     ;
	businessTripVOTT   [     881     ]     =     88.1     ;
	businessTripVOTT   [     882     ]     =     88.2     ;
	businessTripVOTT   [     883     ]     =     88.3     ;
	businessTripVOTT   [     884     ]     =     88.4     ;
	businessTripVOTT   [     885     ]     =     88.5     ;
	businessTripVOTT   [     886     ]     =     88.6     ;
	businessTripVOTT   [     887     ]     =     88.7     ;
	businessTripVOTT   [     888     ]     =     88.8     ;
	businessTripVOTT   [     889     ]     =     88.9     ;
	businessTripVOTT   [     890     ]     =     89     ;
	businessTripVOTT   [     891     ]     =     89.1     ;
	businessTripVOTT   [     892     ]     =     89.2     ;
	businessTripVOTT   [     893     ]     =     89.3     ;
	businessTripVOTT   [     894     ]     =     89.4     ;
	businessTripVOTT   [     895     ]     =     89.5     ;
	businessTripVOTT   [     896     ]     =     89.6     ;
	businessTripVOTT   [     897     ]     =     89.7     ;
	businessTripVOTT   [     898     ]     =     89.8     ;
	businessTripVOTT   [     899     ]     =     89.9     ;
	businessTripVOTT   [     900     ]     =     90     ;
	businessTripVOTT   [     901     ]     =     90.1     ;
	businessTripVOTT   [     902     ]     =     90.2     ;
	businessTripVOTT   [     903     ]     =     90.3     ;
	businessTripVOTT   [     904     ]     =     90.4     ;
	businessTripVOTT   [     905     ]     =     90.5     ;
	businessTripVOTT   [     906     ]     =     90.6     ;
	businessTripVOTT   [     907     ]     =     90.7     ;
	businessTripVOTT   [     908     ]     =     90.8     ;
	businessTripVOTT   [     909     ]     =     90.9     ;
	businessTripVOTT   [     910     ]     =     91     ;
	businessTripVOTT   [     911     ]     =     91.1     ;
	businessTripVOTT   [     912     ]     =     91.2     ;
	businessTripVOTT   [     913     ]     =     91.3     ;
	businessTripVOTT   [     914     ]     =     91.4     ;
	businessTripVOTT   [     915     ]     =     91.5     ;
	businessTripVOTT   [     916     ]     =     91.6     ;
	businessTripVOTT   [     917     ]     =     91.7     ;
	businessTripVOTT   [     918     ]     =     91.8     ;
	businessTripVOTT   [     919     ]     =     91.9     ;
	businessTripVOTT   [     920     ]     =     92     ;
	businessTripVOTT   [     921     ]     =     92.1     ;
	businessTripVOTT   [     922     ]     =     92.2     ;
	businessTripVOTT   [     923     ]     =     92.3     ;
	businessTripVOTT   [     924     ]     =     92.4     ;
	businessTripVOTT   [     925     ]     =     92.5     ;
}

void Simulator::setBusinessTripProbability()
{

	businessTripProbability = new double[926];
	businessTripProbability  [     0     ] =   0.0000   ;
	businessTripProbability  [     1     ] =   0.0029   ;
	businessTripProbability  [     2     ] =   0.0059   ;
	businessTripProbability  [     3     ] =   0.0088   ;
	businessTripProbability  [     4     ] =   0.0117   ;
	businessTripProbability  [     5     ] =   0.0146   ;
	businessTripProbability  [     6     ] =   0.0176   ;
	businessTripProbability  [     7     ] =   0.0205   ;
	businessTripProbability  [     8     ] =   0.0234   ;
	businessTripProbability  [     9     ] =   0.0263   ;
	businessTripProbability  [     10     ] =   0.0293   ;
	businessTripProbability  [     11     ] =   0.0322   ;
	businessTripProbability  [     12     ] =   0.0351   ;
	businessTripProbability  [     13     ] =   0.0380   ;
	businessTripProbability  [     14     ] =   0.0410   ;
	businessTripProbability  [     15     ] =   0.0439   ;
	businessTripProbability  [     16     ] =   0.0468   ;
	businessTripProbability  [     17     ] =   0.0498   ;
	businessTripProbability  [     18     ] =   0.0527   ;
	businessTripProbability  [     19     ] =   0.0556   ;
	businessTripProbability  [     20     ] =   0.0585   ;
	businessTripProbability  [     21     ] =   0.0615   ;
	businessTripProbability  [     22     ] =   0.0644   ;
	businessTripProbability  [     23     ] =   0.0673   ;
	businessTripProbability  [     24     ] =   0.0702   ;
	businessTripProbability  [     25     ] =   0.0731   ;
	businessTripProbability  [     26     ] =   0.0759   ;
	businessTripProbability  [     27     ] =   0.0788   ;
	businessTripProbability  [     28     ] =   0.0817   ;
	businessTripProbability  [     29     ] =   0.0845   ;
	businessTripProbability  [     30     ] =   0.0874   ;
	businessTripProbability  [     31     ] =   0.0902   ;
	businessTripProbability  [     32     ] =   0.0931   ;
	businessTripProbability  [     33     ] =   0.0959   ;
	businessTripProbability  [     34     ] =   0.0988   ;
	businessTripProbability  [     35     ] =   0.1016   ;
	businessTripProbability  [     36     ] =   0.1045   ;
	businessTripProbability  [     37     ] =   0.1073   ;
	businessTripProbability  [     38     ] =   0.1102   ;
	businessTripProbability  [     39     ] =   0.1130   ;
	businessTripProbability  [     40     ] =   0.1159   ;
	businessTripProbability  [     41     ] =   0.1187   ;
	businessTripProbability  [     42     ] =   0.1216   ;
	businessTripProbability  [     43     ] =   0.1245   ;
	businessTripProbability  [     44     ] =   0.1273   ;
	businessTripProbability  [     45     ] =   0.1302   ;
	businessTripProbability  [     46     ] =   0.1330   ;
	businessTripProbability  [     47     ] =   0.1359   ;
	businessTripProbability  [     48     ] =   0.1387   ;
	businessTripProbability  [     49     ] =   0.1422   ;
	businessTripProbability  [     50     ] =   0.1457   ;
	businessTripProbability  [     51     ] =   0.1491   ;
	businessTripProbability  [     52     ] =   0.1526   ;
	businessTripProbability  [     53     ] =   0.1561   ;
	businessTripProbability  [     54     ] =   0.1596   ;
	businessTripProbability  [     55     ] =   0.1630   ;
	businessTripProbability  [     56     ] =   0.1665   ;
	businessTripProbability  [     57     ] =   0.1700   ;
	businessTripProbability  [     58     ] =   0.1735   ;
	businessTripProbability  [     59     ] =   0.1769   ;
	businessTripProbability  [     60     ] =   0.1804   ;
	businessTripProbability  [     61     ] =   0.1839   ;
	businessTripProbability  [     62     ] =   0.1874   ;
	businessTripProbability  [     63     ] =   0.1908   ;
	businessTripProbability  [     64     ] =   0.1943   ;
	businessTripProbability  [     65     ] =   0.1978   ;
	businessTripProbability  [     66     ] =   0.2013   ;
	businessTripProbability  [     67     ] =   0.2047   ;
	businessTripProbability  [     68     ] =   0.2082   ;
	businessTripProbability  [     69     ] =   0.2117   ;
	businessTripProbability  [     70     ] =   0.2152   ;
	businessTripProbability  [     71     ] =   0.2186   ;
	businessTripProbability  [     72     ] =   0.2221   ;
	businessTripProbability  [     73     ] =   0.2254   ;
	businessTripProbability  [     74     ] =   0.2287   ;
	businessTripProbability  [     75     ] =   0.2321   ;
	businessTripProbability  [     76     ] =   0.2354   ;
	businessTripProbability  [     77     ] =   0.2387   ;
	businessTripProbability  [     78     ] =   0.2420   ;
	businessTripProbability  [     79     ] =   0.2453   ;
	businessTripProbability  [     80     ] =   0.2487   ;
	businessTripProbability  [     81     ] =   0.2520   ;
	businessTripProbability  [     82     ] =   0.2553   ;
	businessTripProbability  [     83     ] =   0.2586   ;
	businessTripProbability  [     84     ] =   0.2619   ;
	businessTripProbability  [     85     ] =   0.2653   ;
	businessTripProbability  [     86     ] =   0.2686   ;
	businessTripProbability  [     87     ] =   0.2719   ;
	businessTripProbability  [     88     ] =   0.2752   ;
	businessTripProbability  [     89     ] =   0.2785   ;
	businessTripProbability  [     90     ] =   0.2819   ;
	businessTripProbability  [     91     ] =   0.2852   ;
	businessTripProbability  [     92     ] =   0.2885   ;
	businessTripProbability  [     93     ] =   0.2918   ;
	businessTripProbability  [     94     ] =   0.2951   ;
	businessTripProbability  [     95     ] =   0.2984   ;
	businessTripProbability  [     96     ] =   0.3018   ;
	businessTripProbability  [     97     ] =   0.3051   ;
	businessTripProbability  [     98     ] =   0.3084   ;
	businessTripProbability  [     99     ] =   0.3117   ;
	businessTripProbability  [     100     ] =   0.3150   ;
	businessTripProbability  [     101     ] =   0.3184   ;
	businessTripProbability  [     102     ] =   0.3217   ;
	businessTripProbability  [     103     ] =   0.3250   ;
	businessTripProbability  [     104     ] =   0.3283   ;
	businessTripProbability  [     105     ] =   0.3316   ;
	businessTripProbability  [     106     ] =   0.3350   ;
	businessTripProbability  [     107     ] =   0.3383   ;
	businessTripProbability  [     108     ] =   0.3416   ;
	businessTripProbability  [     109     ] =   0.3449   ;
	businessTripProbability  [     110     ] =   0.3482   ;
	businessTripProbability  [     111     ] =   0.3516   ;
	businessTripProbability  [     112     ] =   0.3549   ;
	businessTripProbability  [     113     ] =   0.3582   ;
	businessTripProbability  [     114     ] =   0.3615   ;
	businessTripProbability  [     115     ] =   0.3648   ;
	businessTripProbability  [     116     ] =   0.3681   ;
	businessTripProbability  [     117     ] =   0.3715   ;
	businessTripProbability  [     118     ] =   0.3748   ;
	businessTripProbability  [     119     ] =   0.3781   ;
	businessTripProbability  [     120     ] =   0.3814   ;
	businessTripProbability  [     121     ] =   0.3841   ;
	businessTripProbability  [     122     ] =   0.3868   ;
	businessTripProbability  [     123     ] =   0.3895   ;
	businessTripProbability  [     124     ] =   0.3922   ;
	businessTripProbability  [     125     ] =   0.3949   ;
	businessTripProbability  [     126     ] =   0.3976   ;
	businessTripProbability  [     127     ] =   0.4003   ;
	businessTripProbability  [     128     ] =   0.4030   ;
	businessTripProbability  [     129     ] =   0.4057   ;
	businessTripProbability  [     130     ] =   0.4084   ;
	businessTripProbability  [     131     ] =   0.4111   ;
	businessTripProbability  [     132     ] =   0.4138   ;
	businessTripProbability  [     133     ] =   0.4165   ;
	businessTripProbability  [     134     ] =   0.4192   ;
	businessTripProbability  [     135     ] =   0.4219   ;
	businessTripProbability  [     136     ] =   0.4246   ;
	businessTripProbability  [     137     ] =   0.4273   ;
	businessTripProbability  [     138     ] =   0.4301   ;
	businessTripProbability  [     139     ] =   0.4328   ;
	businessTripProbability  [     140     ] =   0.4355   ;
	businessTripProbability  [     141     ] =   0.4382   ;
	businessTripProbability  [     142     ] =   0.4409   ;
	businessTripProbability  [     143     ] =   0.4436   ;
	businessTripProbability  [     144     ] =   0.4463   ;
	businessTripProbability  [     145     ] =   0.4490   ;
	businessTripProbability  [     146     ] =   0.4517   ;
	businessTripProbability  [     147     ] =   0.4544   ;
	businessTripProbability  [     148     ] =   0.4571   ;
	businessTripProbability  [     149     ] =   0.4598   ;
	businessTripProbability  [     150     ] =   0.4625   ;
	businessTripProbability  [     151     ] =   0.4652   ;
	businessTripProbability  [     152     ] =   0.4679   ;
	businessTripProbability  [     153     ] =   0.4706   ;
	businessTripProbability  [     154     ] =   0.4733   ;
	businessTripProbability  [     155     ] =   0.4760   ;
	businessTripProbability  [     156     ] =   0.4787   ;
	businessTripProbability  [     157     ] =   0.4814   ;
	businessTripProbability  [     158     ] =   0.4841   ;
	businessTripProbability  [     159     ] =   0.4868   ;
	businessTripProbability  [     160     ] =   0.4895   ;
	businessTripProbability  [     161     ] =   0.4922   ;
	businessTripProbability  [     162     ] =   0.4949   ;
	businessTripProbability  [     163     ] =   0.4976   ;
	businessTripProbability  [     164     ] =   0.5003   ;
	businessTripProbability  [     165     ] =   0.5030   ;
	businessTripProbability  [     166     ] =   0.5057   ;
	businessTripProbability  [     167     ] =   0.5084   ;
	businessTripProbability  [     168     ] =   0.5111   ;
	businessTripProbability  [     169     ] =   0.5132   ;
	businessTripProbability  [     170     ] =   0.5153   ;
	businessTripProbability  [     171     ] =   0.5174   ;
	businessTripProbability  [     172     ] =   0.5196   ;
	businessTripProbability  [     173     ] =   0.5217   ;
	businessTripProbability  [     174     ] =   0.5238   ;
	businessTripProbability  [     175     ] =   0.5259   ;
	businessTripProbability  [     176     ] =   0.5280   ;
	businessTripProbability  [     177     ] =   0.5302   ;
	businessTripProbability  [     178     ] =   0.5323   ;
	businessTripProbability  [     179     ] =   0.5344   ;
	businessTripProbability  [     180     ] =   0.5365   ;
	businessTripProbability  [     181     ] =   0.5386   ;
	businessTripProbability  [     182     ] =   0.5407   ;
	businessTripProbability  [     183     ] =   0.5429   ;
	businessTripProbability  [     184     ] =   0.5450   ;
	businessTripProbability  [     185     ] =   0.5471   ;
	businessTripProbability  [     186     ] =   0.5492   ;
	businessTripProbability  [     187     ] =   0.5513   ;
	businessTripProbability  [     188     ] =   0.5535   ;
	businessTripProbability  [     189     ] =   0.5556   ;
	businessTripProbability  [     190     ] =   0.5577   ;
	businessTripProbability  [     191     ] =   0.5598   ;
	businessTripProbability  [     192     ] =   0.5619   ;
	businessTripProbability  [     193     ] =   0.5640   ;
	businessTripProbability  [     194     ] =   0.5662   ;
	businessTripProbability  [     195     ] =   0.5683   ;
	businessTripProbability  [     196     ] =   0.5704   ;
	businessTripProbability  [     197     ] =   0.5725   ;
	businessTripProbability  [     198     ] =   0.5746   ;
	businessTripProbability  [     199     ] =   0.5768   ;
	businessTripProbability  [     200     ] =   0.5789   ;
	businessTripProbability  [     201     ] =   0.5810   ;
	businessTripProbability  [     202     ] =   0.5831   ;
	businessTripProbability  [     203     ] =   0.5852   ;
	businessTripProbability  [     204     ] =   0.5873   ;
	businessTripProbability  [     205     ] =   0.5895   ;
	businessTripProbability  [     206     ] =   0.5916   ;
	businessTripProbability  [     207     ] =   0.5937   ;
	businessTripProbability  [     208     ] =   0.5958   ;
	businessTripProbability  [     209     ] =   0.5979   ;
	businessTripProbability  [     210     ] =   0.6000   ;
	businessTripProbability  [     211     ] =   0.6022   ;
	businessTripProbability  [     212     ] =   0.6043   ;
	businessTripProbability  [     213     ] =   0.6064   ;
	businessTripProbability  [     214     ] =   0.6085   ;
	businessTripProbability  [     215     ] =   0.6106   ;
	businessTripProbability  [     216     ] =   0.6128   ;
	businessTripProbability  [     217     ] =   0.6149   ;
	businessTripProbability  [     218     ] =   0.6170   ;
	businessTripProbability  [     219     ] =   0.6191   ;
	businessTripProbability  [     220     ] =   0.6212   ;
	businessTripProbability  [     221     ] =   0.6233   ;
	businessTripProbability  [     222     ] =   0.6255   ;
	businessTripProbability  [     223     ] =   0.6276   ;
	businessTripProbability  [     224     ] =   0.6297   ;
	businessTripProbability  [     225     ] =   0.6318   ;
	businessTripProbability  [     226     ] =   0.6339   ;
	businessTripProbability  [     227     ] =   0.6361   ;
	businessTripProbability  [     228     ] =   0.6382   ;
	businessTripProbability  [     229     ] =   0.6403   ;
	businessTripProbability  [     230     ] =   0.6424   ;
	businessTripProbability  [     231     ] =   0.6445   ;
	businessTripProbability  [     232     ] =   0.6466   ;
	businessTripProbability  [     233     ] =   0.6488   ;
	businessTripProbability  [     234     ] =   0.6509   ;
	businessTripProbability  [     235     ] =   0.6530   ;
	businessTripProbability  [     236     ] =   0.6551   ;
	businessTripProbability  [     237     ] =   0.6572   ;
	businessTripProbability  [     238     ] =   0.6594   ;
	businessTripProbability  [     239     ] =   0.6615   ;
	businessTripProbability  [     240     ] =   0.6636   ;
	businessTripProbability  [     241     ] =   0.6649   ;
	businessTripProbability  [     242     ] =   0.6662   ;
	businessTripProbability  [     243     ] =   0.6676   ;
	businessTripProbability  [     244     ] =   0.6689   ;
	businessTripProbability  [     245     ] =   0.6702   ;
	businessTripProbability  [     246     ] =   0.6715   ;
	businessTripProbability  [     247     ] =   0.6728   ;
	businessTripProbability  [     248     ] =   0.6742   ;
	businessTripProbability  [     249     ] =   0.6755   ;
	businessTripProbability  [     250     ] =   0.6768   ;
	businessTripProbability  [     251     ] =   0.6781   ;
	businessTripProbability  [     252     ] =   0.6794   ;
	businessTripProbability  [     253     ] =   0.6808   ;
	businessTripProbability  [     254     ] =   0.6821   ;
	businessTripProbability  [     255     ] =   0.6834   ;
	businessTripProbability  [     256     ] =   0.6847   ;
	businessTripProbability  [     257     ] =   0.6861   ;
	businessTripProbability  [     258     ] =   0.6874   ;
	businessTripProbability  [     259     ] =   0.6887   ;
	businessTripProbability  [     260     ] =   0.6900   ;
	businessTripProbability  [     261     ] =   0.6913   ;
	businessTripProbability  [     262     ] =   0.6927   ;
	businessTripProbability  [     263     ] =   0.6940   ;
	businessTripProbability  [     264     ] =   0.6953   ;
	businessTripProbability  [     265     ] =   0.6966   ;
	businessTripProbability  [     266     ] =   0.6980   ;
	businessTripProbability  [     267     ] =   0.6993   ;
	businessTripProbability  [     268     ] =   0.7006   ;
	businessTripProbability  [     269     ] =   0.7019   ;
	businessTripProbability  [     270     ] =   0.7032   ;
	businessTripProbability  [     271     ] =   0.7046   ;
	businessTripProbability  [     272     ] =   0.7059   ;
	businessTripProbability  [     273     ] =   0.7072   ;
	businessTripProbability  [     274     ] =   0.7085   ;
	businessTripProbability  [     275     ] =   0.7098   ;
	businessTripProbability  [     276     ] =   0.7112   ;
	businessTripProbability  [     277     ] =   0.7125   ;
	businessTripProbability  [     278     ] =   0.7138   ;
	businessTripProbability  [     279     ] =   0.7151   ;
	businessTripProbability  [     280     ] =   0.7165   ;
	businessTripProbability  [     281     ] =   0.7178   ;
	businessTripProbability  [     282     ] =   0.7191   ;
	businessTripProbability  [     283     ] =   0.7204   ;
	businessTripProbability  [     284     ] =   0.7217   ;
	businessTripProbability  [     285     ] =   0.7231   ;
	businessTripProbability  [     286     ] =   0.7244   ;
	businessTripProbability  [     287     ] =   0.7257   ;
	businessTripProbability  [     288     ] =   0.7270   ;
	businessTripProbability  [     289     ] =   0.7283   ;
	businessTripProbability  [     290     ] =   0.7297   ;
	businessTripProbability  [     291     ] =   0.7310   ;
	businessTripProbability  [     292     ] =   0.7323   ;
	businessTripProbability  [     293     ] =   0.7336   ;
	businessTripProbability  [     294     ] =   0.7350   ;
	businessTripProbability  [     295     ] =   0.7363   ;
	businessTripProbability  [     296     ] =   0.7376   ;
	businessTripProbability  [     297     ] =   0.7389   ;
	businessTripProbability  [     298     ] =   0.7402   ;
	businessTripProbability  [     299     ] =   0.7416   ;
	businessTripProbability  [     300     ] =   0.7429   ;
	businessTripProbability  [     301     ] =   0.7442   ;
	businessTripProbability  [     302     ] =   0.7455   ;
	businessTripProbability  [     303     ] =   0.7468   ;
	businessTripProbability  [     304     ] =   0.7482   ;
	businessTripProbability  [     305     ] =   0.7495   ;
	businessTripProbability  [     306     ] =   0.7508   ;
	businessTripProbability  [     307     ] =   0.7521   ;
	businessTripProbability  [     308     ] =   0.7535   ;
	businessTripProbability  [     309     ] =   0.7548   ;
	businessTripProbability  [     310     ] =   0.7561   ;
	businessTripProbability  [     311     ] =   0.7574   ;
	businessTripProbability  [     312     ] =   0.7587   ;
	businessTripProbability  [     313     ] =   0.7601   ;
	businessTripProbability  [     314     ] =   0.7614   ;
	businessTripProbability  [     315     ] =   0.7627   ;
	businessTripProbability  [     316     ] =   0.7640   ;
	businessTripProbability  [     317     ] =   0.7653   ;
	businessTripProbability  [     318     ] =   0.7667   ;
	businessTripProbability  [     319     ] =   0.7680   ;
	businessTripProbability  [     320     ] =   0.7693   ;
	businessTripProbability  [     321     ] =   0.7706   ;
	businessTripProbability  [     322     ] =   0.7720   ;
	businessTripProbability  [     323     ] =   0.7733   ;
	businessTripProbability  [     324     ] =   0.7746   ;
	businessTripProbability  [     325     ] =   0.7759   ;
	businessTripProbability  [     326     ] =   0.7772   ;
	businessTripProbability  [     327     ] =   0.7786   ;
	businessTripProbability  [     328     ] =   0.7799   ;
	businessTripProbability  [     329     ] =   0.7812   ;
	businessTripProbability  [     330     ] =   0.7825   ;
	businessTripProbability  [     331     ] =   0.7838   ;
	businessTripProbability  [     332     ] =   0.7852   ;
	businessTripProbability  [     333     ] =   0.7865   ;
	businessTripProbability  [     334     ] =   0.7878   ;
	businessTripProbability  [     335     ] =   0.7891   ;
	businessTripProbability  [     336     ] =   0.7905   ;
	businessTripProbability  [     337     ] =   0.7918   ;
	businessTripProbability  [     338     ] =   0.7931   ;
	businessTripProbability  [     339     ] =   0.7944   ;
	businessTripProbability  [     340     ] =   0.7957   ;
	businessTripProbability  [     341     ] =   0.7971   ;
	businessTripProbability  [     342     ] =   0.7984   ;
	businessTripProbability  [     343     ] =   0.7997   ;
	businessTripProbability  [     344     ] =   0.8010   ;
	businessTripProbability  [     345     ] =   0.8024   ;
	businessTripProbability  [     346     ] =   0.8037   ;
	businessTripProbability  [     347     ] =   0.8050   ;
	businessTripProbability  [     348     ] =   0.8063   ;
	businessTripProbability  [     349     ] =   0.8076   ;
	businessTripProbability  [     350     ] =   0.8090   ;
	businessTripProbability  [     351     ] =   0.8103   ;
	businessTripProbability  [     352     ] =   0.8116   ;
	businessTripProbability  [     353     ] =   0.8129   ;
	businessTripProbability  [     354     ] =   0.8142   ;
	businessTripProbability  [     355     ] =   0.8156   ;
	businessTripProbability  [     356     ] =   0.8169   ;
	businessTripProbability  [     357     ] =   0.8182   ;
	businessTripProbability  [     358     ] =   0.8195   ;
	businessTripProbability  [     359     ] =   0.8209   ;
	businessTripProbability  [     360     ] =   0.8222   ;
	businessTripProbability  [     361     ] =   0.8235   ;
	businessTripProbability  [     362     ] =   0.8238   ;
	businessTripProbability  [     363     ] =   0.8241   ;
	businessTripProbability  [     364     ] =   0.8244   ;
	businessTripProbability  [     365     ] =   0.8247   ;
	businessTripProbability  [     366     ] =   0.8251   ;
	businessTripProbability  [     367     ] =   0.8254   ;
	businessTripProbability  [     368     ] =   0.8257   ;
	businessTripProbability  [     369     ] =   0.8260   ;
	businessTripProbability  [     370     ] =   0.8263   ;
	businessTripProbability  [     371     ] =   0.8266   ;
	businessTripProbability  [     372     ] =   0.8269   ;
	businessTripProbability  [     373     ] =   0.8272   ;
	businessTripProbability  [     374     ] =   0.8276   ;
	businessTripProbability  [     375     ] =   0.8279   ;
	businessTripProbability  [     376     ] =   0.8282   ;
	businessTripProbability  [     377     ] =   0.8285   ;
	businessTripProbability  [     378     ] =   0.8288   ;
	businessTripProbability  [     379     ] =   0.8291   ;
	businessTripProbability  [     380     ] =   0.8294   ;
	businessTripProbability  [     381     ] =   0.8297   ;
	businessTripProbability  [     382     ] =   0.8301   ;
	businessTripProbability  [     383     ] =   0.8304   ;
	businessTripProbability  [     384     ] =   0.8307   ;
	businessTripProbability  [     385     ] =   0.8310   ;
	businessTripProbability  [     386     ] =   0.8313   ;
	businessTripProbability  [     387     ] =   0.8316   ;
	businessTripProbability  [     388     ] =   0.8319   ;
	businessTripProbability  [     389     ] =   0.8323   ;
	businessTripProbability  [     390     ] =   0.8326   ;
	businessTripProbability  [     391     ] =   0.8329   ;
	businessTripProbability  [     392     ] =   0.8332   ;
	businessTripProbability  [     393     ] =   0.8335   ;
	businessTripProbability  [     394     ] =   0.8338   ;
	businessTripProbability  [     395     ] =   0.8341   ;
	businessTripProbability  [     396     ] =   0.8344   ;
	businessTripProbability  [     397     ] =   0.8348   ;
	businessTripProbability  [     398     ] =   0.8351   ;
	businessTripProbability  [     399     ] =   0.8354   ;
	businessTripProbability  [     400     ] =   0.8357   ;
	businessTripProbability  [     401     ] =   0.8360   ;
	businessTripProbability  [     402     ] =   0.8363   ;
	businessTripProbability  [     403     ] =   0.8366   ;
	businessTripProbability  [     404     ] =   0.8369   ;
	businessTripProbability  [     405     ] =   0.8373   ;
	businessTripProbability  [     406     ] =   0.8376   ;
	businessTripProbability  [     407     ] =   0.8379   ;
	businessTripProbability  [     408     ] =   0.8382   ;
	businessTripProbability  [     409     ] =   0.8385   ;
	businessTripProbability  [     410     ] =   0.8388   ;
	businessTripProbability  [     411     ] =   0.8391   ;
	businessTripProbability  [     412     ] =   0.8394   ;
	businessTripProbability  [     413     ] =   0.8398   ;
	businessTripProbability  [     414     ] =   0.8401   ;
	businessTripProbability  [     415     ] =   0.8404   ;
	businessTripProbability  [     416     ] =   0.8407   ;
	businessTripProbability  [     417     ] =   0.8410   ;
	businessTripProbability  [     418     ] =   0.8413   ;
	businessTripProbability  [     419     ] =   0.8416   ;
	businessTripProbability  [     420     ] =   0.8419   ;
	businessTripProbability  [     421     ] =   0.8423   ;
	businessTripProbability  [     422     ] =   0.8426   ;
	businessTripProbability  [     423     ] =   0.8429   ;
	businessTripProbability  [     424     ] =   0.8432   ;
	businessTripProbability  [     425     ] =   0.8435   ;
	businessTripProbability  [     426     ] =   0.8438   ;
	businessTripProbability  [     427     ] =   0.8441   ;
	businessTripProbability  [     428     ] =   0.8444   ;
	businessTripProbability  [     429     ] =   0.8448   ;
	businessTripProbability  [     430     ] =   0.8451   ;
	businessTripProbability  [     431     ] =   0.8454   ;
	businessTripProbability  [     432     ] =   0.8457   ;
	businessTripProbability  [     433     ] =   0.8460   ;
	businessTripProbability  [     434     ] =   0.8463   ;
	businessTripProbability  [     435     ] =   0.8466   ;
	businessTripProbability  [     436     ] =   0.8469   ;
	businessTripProbability  [     437     ] =   0.8473   ;
	businessTripProbability  [     438     ] =   0.8476   ;
	businessTripProbability  [     439     ] =   0.8479   ;
	businessTripProbability  [     440     ] =   0.8482   ;
	businessTripProbability  [     441     ] =   0.8485   ;
	businessTripProbability  [     442     ] =   0.8488   ;
	businessTripProbability  [     443     ] =   0.8491   ;
	businessTripProbability  [     444     ] =   0.8494   ;
	businessTripProbability  [     445     ] =   0.8498   ;
	businessTripProbability  [     446     ] =   0.8501   ;
	businessTripProbability  [     447     ] =   0.8504   ;
	businessTripProbability  [     448     ] =   0.8507   ;
	businessTripProbability  [     449     ] =   0.8510   ;
	businessTripProbability  [     450     ] =   0.8513   ;
	businessTripProbability  [     451     ] =   0.8516   ;
	businessTripProbability  [     452     ] =   0.8519   ;
	businessTripProbability  [     453     ] =   0.8523   ;
	businessTripProbability  [     454     ] =   0.8526   ;
	businessTripProbability  [     455     ] =   0.8529   ;
	businessTripProbability  [     456     ] =   0.8532   ;
	businessTripProbability  [     457     ] =   0.8535   ;
	businessTripProbability  [     458     ] =   0.8538   ;
	businessTripProbability  [     459     ] =   0.8541   ;
	businessTripProbability  [     460     ] =   0.8545   ;
	businessTripProbability  [     461     ] =   0.8548   ;
	businessTripProbability  [     462     ] =   0.8551   ;
	businessTripProbability  [     463     ] =   0.8554   ;
	businessTripProbability  [     464     ] =   0.8557   ;
	businessTripProbability  [     465     ] =   0.8560   ;
	businessTripProbability  [     466     ] =   0.8563   ;
	businessTripProbability  [     467     ] =   0.8566   ;
	businessTripProbability  [     468     ] =   0.8570   ;
	businessTripProbability  [     469     ] =   0.8573   ;
	businessTripProbability  [     470     ] =   0.8576   ;
	businessTripProbability  [     471     ] =   0.8579   ;
	businessTripProbability  [     472     ] =   0.8582   ;
	businessTripProbability  [     473     ] =   0.8585   ;
	businessTripProbability  [     474     ] =   0.8588   ;
	businessTripProbability  [     475     ] =   0.8591   ;
	businessTripProbability  [     476     ] =   0.8595   ;
	businessTripProbability  [     477     ] =   0.8598   ;
	businessTripProbability  [     478     ] =   0.8601   ;
	businessTripProbability  [     479     ] =   0.8604   ;
	businessTripProbability  [     480     ] =   0.8607   ;
	businessTripProbability  [     481     ] =   0.8610   ;
	businessTripProbability  [     482     ] =   0.8613   ;
	businessTripProbability  [     483     ] =   0.8616   ;
	businessTripProbability  [     484     ] =   0.8620   ;
	businessTripProbability  [     485     ] =   0.8623   ;
	businessTripProbability  [     486     ] =   0.8626   ;
	businessTripProbability  [     487     ] =   0.8629   ;
	businessTripProbability  [     488     ] =   0.8632   ;
	businessTripProbability  [     489     ] =   0.8635   ;
	businessTripProbability  [     490     ] =   0.8638   ;
	businessTripProbability  [     491     ] =   0.8641   ;
	businessTripProbability  [     492     ] =   0.8645   ;
	businessTripProbability  [     493     ] =   0.8648   ;
	businessTripProbability  [     494     ] =   0.8651   ;
	businessTripProbability  [     495     ] =   0.8654   ;
	businessTripProbability  [     496     ] =   0.8657   ;
	businessTripProbability  [     497     ] =   0.8660   ;
	businessTripProbability  [     498     ] =   0.8663   ;
	businessTripProbability  [     499     ] =   0.8666   ;
	businessTripProbability  [     500     ] =   0.8670   ;
	businessTripProbability  [     501     ] =   0.8673   ;
	businessTripProbability  [     502     ] =   0.8676   ;
	businessTripProbability  [     503     ] =   0.8679   ;
	businessTripProbability  [     504     ] =   0.8682   ;
	businessTripProbability  [     505     ] =   0.8685   ;
	businessTripProbability  [     506     ] =   0.8688   ;
	businessTripProbability  [     507     ] =   0.8691   ;
	businessTripProbability  [     508     ] =   0.8695   ;
	businessTripProbability  [     509     ] =   0.8698   ;
	businessTripProbability  [     510     ] =   0.8701   ;
	businessTripProbability  [     511     ] =   0.8704   ;
	businessTripProbability  [     512     ] =   0.8707   ;
	businessTripProbability  [     513     ] =   0.8710   ;
	businessTripProbability  [     514     ] =   0.8713   ;
	businessTripProbability  [     515     ] =   0.8716   ;
	businessTripProbability  [     516     ] =   0.8720   ;
	businessTripProbability  [     517     ] =   0.8723   ;
	businessTripProbability  [     518     ] =   0.8726   ;
	businessTripProbability  [     519     ] =   0.8729   ;
	businessTripProbability  [     520     ] =   0.8732   ;
	businessTripProbability  [     521     ] =   0.8735   ;
	businessTripProbability  [     522     ] =   0.8738   ;
	businessTripProbability  [     523     ] =   0.8742   ;
	businessTripProbability  [     524     ] =   0.8745   ;
	businessTripProbability  [     525     ] =   0.8748   ;
	businessTripProbability  [     526     ] =   0.8751   ;
	businessTripProbability  [     527     ] =   0.8754   ;
	businessTripProbability  [     528     ] =   0.8757   ;
	businessTripProbability  [     529     ] =   0.8760   ;
	businessTripProbability  [     530     ] =   0.8763   ;
	businessTripProbability  [     531     ] =   0.8767   ;
	businessTripProbability  [     532     ] =   0.8770   ;
	businessTripProbability  [     533     ] =   0.8773   ;
	businessTripProbability  [     534     ] =   0.8776   ;
	businessTripProbability  [     535     ] =   0.8779   ;
	businessTripProbability  [     536     ] =   0.8782   ;
	businessTripProbability  [     537     ] =   0.8785   ;
	businessTripProbability  [     538     ] =   0.8788   ;
	businessTripProbability  [     539     ] =   0.8792   ;
	businessTripProbability  [     540     ] =   0.8795   ;
	businessTripProbability  [     541     ] =   0.8798   ;
	businessTripProbability  [     542     ] =   0.8801   ;
	businessTripProbability  [     543     ] =   0.8804   ;
	businessTripProbability  [     544     ] =   0.8807   ;
	businessTripProbability  [     545     ] =   0.8810   ;
	businessTripProbability  [     546     ] =   0.8813   ;
	businessTripProbability  [     547     ] =   0.8817   ;
	businessTripProbability  [     548     ] =   0.8820   ;
	businessTripProbability  [     549     ] =   0.8823   ;
	businessTripProbability  [     550     ] =   0.8826   ;
	businessTripProbability  [     551     ] =   0.8829   ;
	businessTripProbability  [     552     ] =   0.8832   ;
	businessTripProbability  [     553     ] =   0.8835   ;
	businessTripProbability  [     554     ] =   0.8838   ;
	businessTripProbability  [     555     ] =   0.8842   ;
	businessTripProbability  [     556     ] =   0.8845   ;
	businessTripProbability  [     557     ] =   0.8848   ;
	businessTripProbability  [     558     ] =   0.8851   ;
	businessTripProbability  [     559     ] =   0.8854   ;
	businessTripProbability  [     560     ] =   0.8857   ;
	businessTripProbability  [     561     ] =   0.8860   ;
	businessTripProbability  [     562     ] =   0.8863   ;
	businessTripProbability  [     563     ] =   0.8867   ;
	businessTripProbability  [     564     ] =   0.8870   ;
	businessTripProbability  [     565     ] =   0.8873   ;
	businessTripProbability  [     566     ] =   0.8876   ;
	businessTripProbability  [     567     ] =   0.8879   ;
	businessTripProbability  [     568     ] =   0.8882   ;
	businessTripProbability  [     569     ] =   0.8885   ;
	businessTripProbability  [     570     ] =   0.8888   ;
	businessTripProbability  [     571     ] =   0.8892   ;
	businessTripProbability  [     572     ] =   0.8895   ;
	businessTripProbability  [     573     ] =   0.8898   ;
	businessTripProbability  [     574     ] =   0.8901   ;
	businessTripProbability  [     575     ] =   0.8904   ;
	businessTripProbability  [     576     ] =   0.8907   ;
	businessTripProbability  [     577     ] =   0.8910   ;
	businessTripProbability  [     578     ] =   0.8913   ;
	businessTripProbability  [     579     ] =   0.8917   ;
	businessTripProbability  [     580     ] =   0.8920   ;
	businessTripProbability  [     581     ] =   0.8923   ;
	businessTripProbability  [     582     ] =   0.8926   ;
	businessTripProbability  [     583     ] =   0.8929   ;
	businessTripProbability  [     584     ] =   0.8932   ;
	businessTripProbability  [     585     ] =   0.8935   ;
	businessTripProbability  [     586     ] =   0.8939   ;
	businessTripProbability  [     587     ] =   0.8942   ;
	businessTripProbability  [     588     ] =   0.8945   ;
	businessTripProbability  [     589     ] =   0.8948   ;
	businessTripProbability  [     590     ] =   0.8951   ;
	businessTripProbability  [     591     ] =   0.8954   ;
	businessTripProbability  [     592     ] =   0.8957   ;
	businessTripProbability  [     593     ] =   0.8960   ;
	businessTripProbability  [     594     ] =   0.8964   ;
	businessTripProbability  [     595     ] =   0.8967   ;
	businessTripProbability  [     596     ] =   0.8970   ;
	businessTripProbability  [     597     ] =   0.8973   ;
	businessTripProbability  [     598     ] =   0.8976   ;
	businessTripProbability  [     599     ] =   0.8979   ;
	businessTripProbability  [     600     ] =   0.8982   ;
	businessTripProbability  [     601     ] =   0.8985   ;
	businessTripProbability  [     602     ] =   0.8989   ;
	businessTripProbability  [     603     ] =   0.8992   ;
	businessTripProbability  [     604     ] =   0.8995   ;
	businessTripProbability  [     605     ] =   0.8998   ;
	businessTripProbability  [     606     ] =   0.9001   ;
	businessTripProbability  [     607     ] =   0.9004   ;
	businessTripProbability  [     608     ] =   0.9007   ;
	businessTripProbability  [     609     ] =   0.9010   ;
	businessTripProbability  [     610     ] =   0.9014   ;
	businessTripProbability  [     611     ] =   0.9017   ;
	businessTripProbability  [     612     ] =   0.9020   ;
	businessTripProbability  [     613     ] =   0.9023   ;
	businessTripProbability  [     614     ] =   0.9026   ;
	businessTripProbability  [     615     ] =   0.9029   ;
	businessTripProbability  [     616     ] =   0.9032   ;
	businessTripProbability  [     617     ] =   0.9035   ;
	businessTripProbability  [     618     ] =   0.9039   ;
	businessTripProbability  [     619     ] =   0.9042   ;
	businessTripProbability  [     620     ] =   0.9045   ;
	businessTripProbability  [     621     ] =   0.9048   ;
	businessTripProbability  [     622     ] =   0.9051   ;
	businessTripProbability  [     623     ] =   0.9054   ;
	businessTripProbability  [     624     ] =   0.9057   ;
	businessTripProbability  [     625     ] =   0.9060   ;
	businessTripProbability  [     626     ] =   0.9064   ;
	businessTripProbability  [     627     ] =   0.9067   ;
	businessTripProbability  [     628     ] =   0.9070   ;
	businessTripProbability  [     629     ] =   0.9073   ;
	businessTripProbability  [     630     ] =   0.9076   ;
	businessTripProbability  [     631     ] =   0.9079   ;
	businessTripProbability  [     632     ] =   0.9082   ;
	businessTripProbability  [     633     ] =   0.9085   ;
	businessTripProbability  [     634     ] =   0.9089   ;
	businessTripProbability  [     635     ] =   0.9092   ;
	businessTripProbability  [     636     ] =   0.9095   ;
	businessTripProbability  [     637     ] =   0.9098   ;
	businessTripProbability  [     638     ] =   0.9101   ;
	businessTripProbability  [     639     ] =   0.9104   ;
	businessTripProbability  [     640     ] =   0.9107   ;
	businessTripProbability  [     641     ] =   0.9110   ;
	businessTripProbability  [     642     ] =   0.9114   ;
	businessTripProbability  [     643     ] =   0.9117   ;
	businessTripProbability  [     644     ] =   0.9120   ;
	businessTripProbability  [     645     ] =   0.9123   ;
	businessTripProbability  [     646     ] =   0.9126   ;
	businessTripProbability  [     647     ] =   0.9129   ;
	businessTripProbability  [     648     ] =   0.9132   ;
	businessTripProbability  [     649     ] =   0.9136   ;
	businessTripProbability  [     650     ] =   0.9139   ;
	businessTripProbability  [     651     ] =   0.9142   ;
	businessTripProbability  [     652     ] =   0.9145   ;
	businessTripProbability  [     653     ] =   0.9148   ;
	businessTripProbability  [     654     ] =   0.9151   ;
	businessTripProbability  [     655     ] =   0.9154   ;
	businessTripProbability  [     656     ] =   0.9157   ;
	businessTripProbability  [     657     ] =   0.9161   ;
	businessTripProbability  [     658     ] =   0.9164   ;
	businessTripProbability  [     659     ] =   0.9167   ;
	businessTripProbability  [     660     ] =   0.9170   ;
	businessTripProbability  [     661     ] =   0.9173   ;
	businessTripProbability  [     662     ] =   0.9176   ;
	businessTripProbability  [     663     ] =   0.9179   ;
	businessTripProbability  [     664     ] =   0.9182   ;
	businessTripProbability  [     665     ] =   0.9186   ;
	businessTripProbability  [     666     ] =   0.9189   ;
	businessTripProbability  [     667     ] =   0.9192   ;
	businessTripProbability  [     668     ] =   0.9195   ;
	businessTripProbability  [     669     ] =   0.9198   ;
	businessTripProbability  [     670     ] =   0.9201   ;
	businessTripProbability  [     671     ] =   0.9204   ;
	businessTripProbability  [     672     ] =   0.9207   ;
	businessTripProbability  [     673     ] =   0.9211   ;
	businessTripProbability  [     674     ] =   0.9214   ;
	businessTripProbability  [     675     ] =   0.9217   ;
	businessTripProbability  [     676     ] =   0.9220   ;
	businessTripProbability  [     677     ] =   0.9223   ;
	businessTripProbability  [     678     ] =   0.9226   ;
	businessTripProbability  [     679     ] =   0.9229   ;
	businessTripProbability  [     680     ] =   0.9232   ;
	businessTripProbability  [     681     ] =   0.9236   ;
	businessTripProbability  [     682     ] =   0.9239   ;
	businessTripProbability  [     683     ] =   0.9242   ;
	businessTripProbability  [     684     ] =   0.9245   ;
	businessTripProbability  [     685     ] =   0.9248   ;
	businessTripProbability  [     686     ] =   0.9251   ;
	businessTripProbability  [     687     ] =   0.9254   ;
	businessTripProbability  [     688     ] =   0.9257   ;
	businessTripProbability  [     689     ] =   0.9261   ;
	businessTripProbability  [     690     ] =   0.9264   ;
	businessTripProbability  [     691     ] =   0.9267   ;
	businessTripProbability  [     692     ] =   0.9270   ;
	businessTripProbability  [     693     ] =   0.9273   ;
	businessTripProbability  [     694     ] =   0.9276   ;
	businessTripProbability  [     695     ] =   0.9279   ;
	businessTripProbability  [     696     ] =   0.9282   ;
	businessTripProbability  [     697     ] =   0.9286   ;
	businessTripProbability  [     698     ] =   0.9289   ;
	businessTripProbability  [     699     ] =   0.9292   ;
	businessTripProbability  [     700     ] =   0.9295   ;
	businessTripProbability  [     701     ] =   0.9298   ;
	businessTripProbability  [     702     ] =   0.9301   ;
	businessTripProbability  [     703     ] =   0.9304   ;
	businessTripProbability  [     704     ] =   0.9307   ;
	businessTripProbability  [     705     ] =   0.9311   ;
	businessTripProbability  [     706     ] =   0.9314   ;
	businessTripProbability  [     707     ] =   0.9317   ;
	businessTripProbability  [     708     ] =   0.9320   ;
	businessTripProbability  [     709     ] =   0.9323   ;
	businessTripProbability  [     710     ] =   0.9326   ;
	businessTripProbability  [     711     ] =   0.9329   ;
	businessTripProbability  [     712     ] =   0.9332   ;
	businessTripProbability  [     713     ] =   0.9336   ;
	businessTripProbability  [     714     ] =   0.9339   ;
	businessTripProbability  [     715     ] =   0.9342   ;
	businessTripProbability  [     716     ] =   0.9345   ;
	businessTripProbability  [     717     ] =   0.9348   ;
	businessTripProbability  [     718     ] =   0.9351   ;
	businessTripProbability  [     719     ] =   0.9354   ;
	businessTripProbability  [     720     ] =   0.9358   ;
	businessTripProbability  [     721     ] =   0.9361   ;
	businessTripProbability  [     722     ] =   0.9364   ;
	businessTripProbability  [     723     ] =   0.9367   ;
	businessTripProbability  [     724     ] =   0.9370   ;
	businessTripProbability  [     725     ] =   0.9373   ;
	businessTripProbability  [     726     ] =   0.9376   ;
	businessTripProbability  [     727     ] =   0.9379   ;
	businessTripProbability  [     728     ] =   0.9383   ;
	businessTripProbability  [     729     ] =   0.9386   ;
	businessTripProbability  [     730     ] =   0.9389   ;
	businessTripProbability  [     731     ] =   0.9392   ;
	businessTripProbability  [     732     ] =   0.9395   ;
	businessTripProbability  [     733     ] =   0.9398   ;
	businessTripProbability  [     734     ] =   0.9401   ;
	businessTripProbability  [     735     ] =   0.9404   ;
	businessTripProbability  [     736     ] =   0.9408   ;
	businessTripProbability  [     737     ] =   0.9411   ;
	businessTripProbability  [     738     ] =   0.9414   ;
	businessTripProbability  [     739     ] =   0.9417   ;
	businessTripProbability  [     740     ] =   0.9420   ;
	businessTripProbability  [     741     ] =   0.9423   ;
	businessTripProbability  [     742     ] =   0.9426   ;
	businessTripProbability  [     743     ] =   0.9429   ;
	businessTripProbability  [     744     ] =   0.9433   ;
	businessTripProbability  [     745     ] =   0.9436   ;
	businessTripProbability  [     746     ] =   0.9439   ;
	businessTripProbability  [     747     ] =   0.9442   ;
	businessTripProbability  [     748     ] =   0.9445   ;
	businessTripProbability  [     749     ] =   0.9448   ;
	businessTripProbability  [     750     ] =   0.9451   ;
	businessTripProbability  [     751     ] =   0.9454   ;
	businessTripProbability  [     752     ] =   0.9458   ;
	businessTripProbability  [     753     ] =   0.9461   ;
	businessTripProbability  [     754     ] =   0.9464   ;
	businessTripProbability  [     755     ] =   0.9467   ;
	businessTripProbability  [     756     ] =   0.9470   ;
	businessTripProbability  [     757     ] =   0.9473   ;
	businessTripProbability  [     758     ] =   0.9476   ;
	businessTripProbability  [     759     ] =   0.9479   ;
	businessTripProbability  [     760     ] =   0.9483   ;
	businessTripProbability  [     761     ] =   0.9486   ;
	businessTripProbability  [     762     ] =   0.9489   ;
	businessTripProbability  [     763     ] =   0.9492   ;
	businessTripProbability  [     764     ] =   0.9495   ;
	businessTripProbability  [     765     ] =   0.9498   ;
	businessTripProbability  [     766     ] =   0.9501   ;
	businessTripProbability  [     767     ] =   0.9504   ;
	businessTripProbability  [     768     ] =   0.9508   ;
	businessTripProbability  [     769     ] =   0.9511   ;
	businessTripProbability  [     770     ] =   0.9514   ;
	businessTripProbability  [     771     ] =   0.9517   ;
	businessTripProbability  [     772     ] =   0.9520   ;
	businessTripProbability  [     773     ] =   0.9523   ;
	businessTripProbability  [     774     ] =   0.9526   ;
	businessTripProbability  [     775     ] =   0.9529   ;
	businessTripProbability  [     776     ] =   0.9533   ;
	businessTripProbability  [     777     ] =   0.9536   ;
	businessTripProbability  [     778     ] =   0.9539   ;
	businessTripProbability  [     779     ] =   0.9542   ;
	businessTripProbability  [     780     ] =   0.9545   ;
	businessTripProbability  [     781     ] =   0.9548   ;
	businessTripProbability  [     782     ] =   0.9551   ;
	businessTripProbability  [     783     ] =   0.9555   ;
	businessTripProbability  [     784     ] =   0.9558   ;
	businessTripProbability  [     785     ] =   0.9561   ;
	businessTripProbability  [     786     ] =   0.9564   ;
	businessTripProbability  [     787     ] =   0.9567   ;
	businessTripProbability  [     788     ] =   0.9570   ;
	businessTripProbability  [     789     ] =   0.9573   ;
	businessTripProbability  [     790     ] =   0.9576   ;
	businessTripProbability  [     791     ] =   0.9580   ;
	businessTripProbability  [     792     ] =   0.9583   ;
	businessTripProbability  [     793     ] =   0.9586   ;
	businessTripProbability  [     794     ] =   0.9589   ;
	businessTripProbability  [     795     ] =   0.9592   ;
	businessTripProbability  [     796     ] =   0.9595   ;
	businessTripProbability  [     797     ] =   0.9598   ;
	businessTripProbability  [     798     ] =   0.9601   ;
	businessTripProbability  [     799     ] =   0.9605   ;
	businessTripProbability  [     800     ] =   0.9608   ;
	businessTripProbability  [     801     ] =   0.9611   ;
	businessTripProbability  [     802     ] =   0.9614   ;
	businessTripProbability  [     803     ] =   0.9617   ;
	businessTripProbability  [     804     ] =   0.9620   ;
	businessTripProbability  [     805     ] =   0.9623   ;
	businessTripProbability  [     806     ] =   0.9626   ;
	businessTripProbability  [     807     ] =   0.9630   ;
	businessTripProbability  [     808     ] =   0.9633   ;
	businessTripProbability  [     809     ] =   0.9636   ;
	businessTripProbability  [     810     ] =   0.9639   ;
	businessTripProbability  [     811     ] =   0.9642   ;
	businessTripProbability  [     812     ] =   0.9645   ;
	businessTripProbability  [     813     ] =   0.9648   ;
	businessTripProbability  [     814     ] =   0.9651   ;
	businessTripProbability  [     815     ] =   0.9655   ;
	businessTripProbability  [     816     ] =   0.9658   ;
	businessTripProbability  [     817     ] =   0.9661   ;
	businessTripProbability  [     818     ] =   0.9664   ;
	businessTripProbability  [     819     ] =   0.9667   ;
	businessTripProbability  [     820     ] =   0.9670   ;
	businessTripProbability  [     821     ] =   0.9673   ;
	businessTripProbability  [     822     ] =   0.9676   ;
	businessTripProbability  [     823     ] =   0.9680   ;
	businessTripProbability  [     824     ] =   0.9683   ;
	businessTripProbability  [     825     ] =   0.9686   ;
	businessTripProbability  [     826     ] =   0.9689   ;
	businessTripProbability  [     827     ] =   0.9692   ;
	businessTripProbability  [     828     ] =   0.9695   ;
	businessTripProbability  [     829     ] =   0.9698   ;
	businessTripProbability  [     830     ] =   0.9701   ;
	businessTripProbability  [     831     ] =   0.9705   ;
	businessTripProbability  [     832     ] =   0.9708   ;
	businessTripProbability  [     833     ] =   0.9711   ;
	businessTripProbability  [     834     ] =   0.9714   ;
	businessTripProbability  [     835     ] =   0.9717   ;
	businessTripProbability  [     836     ] =   0.9720   ;
	businessTripProbability  [     837     ] =   0.9723   ;
	businessTripProbability  [     838     ] =   0.9726   ;
	businessTripProbability  [     839     ] =   0.9730   ;
	businessTripProbability  [     840     ] =   0.9733   ;
	businessTripProbability  [     841     ] =   0.9736   ;
	businessTripProbability  [     842     ] =   0.9739   ;
	businessTripProbability  [     843     ] =   0.9742   ;
	businessTripProbability  [     844     ] =   0.9745   ;
	businessTripProbability  [     845     ] =   0.9748   ;
	businessTripProbability  [     846     ] =   0.9752   ;
	businessTripProbability  [     847     ] =   0.9755   ;
	businessTripProbability  [     848     ] =   0.9758   ;
	businessTripProbability  [     849     ] =   0.9761   ;
	businessTripProbability  [     850     ] =   0.9764   ;
	businessTripProbability  [     851     ] =   0.9767   ;
	businessTripProbability  [     852     ] =   0.9770   ;
	businessTripProbability  [     853     ] =   0.9773   ;
	businessTripProbability  [     854     ] =   0.9777   ;
	businessTripProbability  [     855     ] =   0.9780   ;
	businessTripProbability  [     856     ] =   0.9783   ;
	businessTripProbability  [     857     ] =   0.9786   ;
	businessTripProbability  [     858     ] =   0.9789   ;
	businessTripProbability  [     859     ] =   0.9792   ;
	businessTripProbability  [     860     ] =   0.9795   ;
	businessTripProbability  [     861     ] =   0.9798   ;
	businessTripProbability  [     862     ] =   0.9802   ;
	businessTripProbability  [     863     ] =   0.9805   ;
	businessTripProbability  [     864     ] =   0.9808   ;
	businessTripProbability  [     865     ] =   0.9811   ;
	businessTripProbability  [     866     ] =   0.9814   ;
	businessTripProbability  [     867     ] =   0.9817   ;
	businessTripProbability  [     868     ] =   0.9820   ;
	businessTripProbability  [     869     ] =   0.9823   ;
	businessTripProbability  [     870     ] =   0.9827   ;
	businessTripProbability  [     871     ] =   0.9830   ;
	businessTripProbability  [     872     ] =   0.9833   ;
	businessTripProbability  [     873     ] =   0.9836   ;
	businessTripProbability  [     874     ] =   0.9839   ;
	businessTripProbability  [     875     ] =   0.9842   ;
	businessTripProbability  [     876     ] =   0.9845   ;
	businessTripProbability  [     877     ] =   0.9848   ;
	businessTripProbability  [     878     ] =   0.9852   ;
	businessTripProbability  [     879     ] =   0.9855   ;
	businessTripProbability  [     880     ] =   0.9858   ;
	businessTripProbability  [     881     ] =   0.9861   ;
	businessTripProbability  [     882     ] =   0.9864   ;
	businessTripProbability  [     883     ] =   0.9867   ;
	businessTripProbability  [     884     ] =   0.9870   ;
	businessTripProbability  [     885     ] =   0.9873   ;
	businessTripProbability  [     886     ] =   0.9877   ;
	businessTripProbability  [     887     ] =   0.9880   ;
	businessTripProbability  [     888     ] =   0.9883   ;
	businessTripProbability  [     889     ] =   0.9886   ;
	businessTripProbability  [     890     ] =   0.9889   ;
	businessTripProbability  [     891     ] =   0.9892   ;
	businessTripProbability  [     892     ] =   0.9895   ;
	businessTripProbability  [     893     ] =   0.9898   ;
	businessTripProbability  [     894     ] =   0.9902   ;
	businessTripProbability  [     895     ] =   0.9905   ;
	businessTripProbability  [     896     ] =   0.9908   ;
	businessTripProbability  [     897     ] =   0.9911   ;
	businessTripProbability  [     898     ] =   0.9914   ;
	businessTripProbability  [     899     ] =   0.9917   ;
	businessTripProbability  [     900     ] =   0.9920   ;
	businessTripProbability  [     901     ] =   0.9923   ;
	businessTripProbability  [     902     ] =   0.9927   ;
	businessTripProbability  [     903     ] =   0.9930   ;
	businessTripProbability  [     904     ] =   0.9933   ;
	businessTripProbability  [     905     ] =   0.9936   ;
	businessTripProbability  [     906     ] =   0.9939   ;
	businessTripProbability  [     907     ] =   0.9942   ;
	businessTripProbability  [     908     ] =   0.9945   ;
	businessTripProbability  [     909     ] =   0.9949   ;
	businessTripProbability  [     910     ] =   0.9952   ;
	businessTripProbability  [     911     ] =   0.9955   ;
	businessTripProbability  [     912     ] =   0.9958   ;
	businessTripProbability  [     913     ] =   0.9961   ;
	businessTripProbability  [     914     ] =   0.9964   ;
	businessTripProbability  [     915     ] =   0.9967   ;
	businessTripProbability  [     916     ] =   0.9970   ;
	businessTripProbability  [     917     ] =   0.9974   ;
	businessTripProbability  [     918     ] =   0.9977   ;
	businessTripProbability  [     919     ] =   0.9980   ;
	businessTripProbability  [     920     ] =   0.9983   ;
	businessTripProbability  [     921     ] =   0.9986   ;
	businessTripProbability  [     922     ] =   0.9989   ;
	businessTripProbability  [     923     ] =   0.9992   ;
	businessTripProbability  [     924     ] =   0.9995   ;
	businessTripProbability  [     925     ] =   0.9999   ;
}

void Simulator::setPersonalTripVOTT()
{
	
	personalTripVOTT = new double[484];
	personalTripVOTT   [     0     ]     =     0     ;
	personalTripVOTT   [     1     ]     =     0.1     ;
	personalTripVOTT   [     2     ]     =     0.2     ;
	personalTripVOTT   [     3     ]     =     0.3     ;
	personalTripVOTT   [     4     ]     =     0.4     ;
	personalTripVOTT   [     5     ]     =     0.5     ;
	personalTripVOTT   [     6     ]     =     0.6     ;
	personalTripVOTT   [     7     ]     =     0.7     ;
	personalTripVOTT   [     8     ]     =     0.8     ;
	personalTripVOTT   [     9     ]     =     0.9     ;
	personalTripVOTT   [     10     ]     =     1     ;
	personalTripVOTT   [     11     ]     =     1.1     ;
	personalTripVOTT   [     12     ]     =     1.2     ;
	personalTripVOTT   [     13     ]     =     1.3     ;
	personalTripVOTT   [     14     ]     =     1.4     ;
	personalTripVOTT   [     15     ]     =     1.5     ;
	personalTripVOTT   [     16     ]     =     1.6     ;
	personalTripVOTT   [     17     ]     =     1.7     ;
	personalTripVOTT   [     18     ]     =     1.8     ;
	personalTripVOTT   [     19     ]     =     1.9     ;
	personalTripVOTT   [     20     ]     =     2     ;
	personalTripVOTT   [     21     ]     =     2.1     ;
	personalTripVOTT   [     22     ]     =     2.2     ;
	personalTripVOTT   [     23     ]     =     2.3     ;
	personalTripVOTT   [     24     ]     =     2.4     ;
	personalTripVOTT   [     25     ]     =     2.5     ;
	personalTripVOTT   [     26     ]     =     2.6     ;
	personalTripVOTT   [     27     ]     =     2.7     ;
	personalTripVOTT   [     28     ]     =     2.8     ;
	personalTripVOTT   [     29     ]     =     2.9     ;
	personalTripVOTT   [     30     ]     =     3     ;
	personalTripVOTT   [     31     ]     =     3.1     ;
	personalTripVOTT   [     32     ]     =     3.2     ;
	personalTripVOTT   [     33     ]     =     3.3     ;
	personalTripVOTT   [     34     ]     =     3.4     ;
	personalTripVOTT   [     35     ]     =     3.5     ;
	personalTripVOTT   [     36     ]     =     3.6     ;
	personalTripVOTT   [     37     ]     =     3.7     ;
	personalTripVOTT   [     38     ]     =     3.8     ;
	personalTripVOTT   [     39     ]     =     3.9     ;
	personalTripVOTT   [     40     ]     =     4     ;
	personalTripVOTT   [     41     ]     =     4.1     ;
	personalTripVOTT   [     42     ]     =     4.2     ;
	personalTripVOTT   [     43     ]     =     4.3     ;
	personalTripVOTT   [     44     ]     =     4.4     ;
	personalTripVOTT   [     45     ]     =     4.5     ;
	personalTripVOTT   [     46     ]     =     4.6     ;
	personalTripVOTT   [     47     ]     =     4.7     ;
	personalTripVOTT   [     48     ]     =     4.8     ;
	personalTripVOTT   [     49     ]     =     4.9     ;
	personalTripVOTT   [     50     ]     =     5     ;
	personalTripVOTT   [     51     ]     =     5.1     ;
	personalTripVOTT   [     52     ]     =     5.2     ;
	personalTripVOTT   [     53     ]     =     5.3     ;
	personalTripVOTT   [     54     ]     =     5.4     ;
	personalTripVOTT   [     55     ]     =     5.5     ;
	personalTripVOTT   [     56     ]     =     5.6     ;
	personalTripVOTT   [     57     ]     =     5.7     ;
	personalTripVOTT   [     58     ]     =     5.8     ;
	personalTripVOTT   [     59     ]     =     5.9     ;
	personalTripVOTT   [     60     ]     =     6     ;
	personalTripVOTT   [     61     ]     =     6.1     ;
	personalTripVOTT   [     62     ]     =     6.2     ;
	personalTripVOTT   [     63     ]     =     6.3     ;
	personalTripVOTT   [     64     ]     =     6.4     ;
	personalTripVOTT   [     65     ]     =     6.5     ;
	personalTripVOTT   [     66     ]     =     6.6     ;
	personalTripVOTT   [     67     ]     =     6.7     ;
	personalTripVOTT   [     68     ]     =     6.8     ;
	personalTripVOTT   [     69     ]     =     6.9     ;
	personalTripVOTT   [     70     ]     =     7     ;
	personalTripVOTT   [     71     ]     =     7.1     ;
	personalTripVOTT   [     72     ]     =     7.2     ;
	personalTripVOTT   [     73     ]     =     7.3     ;
	personalTripVOTT   [     74     ]     =     7.4     ;
	personalTripVOTT   [     75     ]     =     7.5     ;
	personalTripVOTT   [     76     ]     =     7.6     ;
	personalTripVOTT   [     77     ]     =     7.7     ;
	personalTripVOTT   [     78     ]     =     7.8     ;
	personalTripVOTT   [     79     ]     =     7.9     ;
	personalTripVOTT   [     80     ]     =     8     ;
	personalTripVOTT   [     81     ]     =     8.1     ;
	personalTripVOTT   [     82     ]     =     8.2     ;
	personalTripVOTT   [     83     ]     =     8.3     ;
	personalTripVOTT   [     84     ]     =     8.4     ;
	personalTripVOTT   [     85     ]     =     8.5     ;
	personalTripVOTT   [     86     ]     =     8.6     ;
	personalTripVOTT   [     87     ]     =     8.7     ;
	personalTripVOTT   [     88     ]     =     8.8     ;
	personalTripVOTT   [     89     ]     =     8.9     ;
	personalTripVOTT   [     90     ]     =     9     ;
	personalTripVOTT   [     91     ]     =     9.1     ;
	personalTripVOTT   [     92     ]     =     9.2     ;
	personalTripVOTT   [     93     ]     =     9.3     ;
	personalTripVOTT   [     94     ]     =     9.4     ;
	personalTripVOTT   [     95     ]     =     9.5     ;
	personalTripVOTT   [     96     ]     =     9.6     ;
	personalTripVOTT   [     97     ]     =     9.7     ;
	personalTripVOTT   [     98     ]     =     9.8     ;
	personalTripVOTT   [     99     ]     =     9.9     ;
	personalTripVOTT   [     100     ]     =     10     ;
	personalTripVOTT   [     101     ]     =     10.1     ;
	personalTripVOTT   [     102     ]     =     10.2     ;
	personalTripVOTT   [     103     ]     =     10.3     ;
	personalTripVOTT   [     104     ]     =     10.4     ;
	personalTripVOTT   [     105     ]     =     10.5     ;
	personalTripVOTT   [     106     ]     =     10.6     ;
	personalTripVOTT   [     107     ]     =     10.7     ;
	personalTripVOTT   [     108     ]     =     10.8     ;
	personalTripVOTT   [     109     ]     =     10.9     ;
	personalTripVOTT   [     110     ]     =     11     ;
	personalTripVOTT   [     111     ]     =     11.1     ;
	personalTripVOTT   [     112     ]     =     11.2     ;
	personalTripVOTT   [     113     ]     =     11.3     ;
	personalTripVOTT   [     114     ]     =     11.4     ;
	personalTripVOTT   [     115     ]     =     11.5     ;
	personalTripVOTT   [     116     ]     =     11.6     ;
	personalTripVOTT   [     117     ]     =     11.7     ;
	personalTripVOTT   [     118     ]     =     11.8     ;
	personalTripVOTT   [     119     ]     =     11.9     ;
	personalTripVOTT   [     120     ]     =     12     ;
	personalTripVOTT   [     121     ]     =     12.1     ;
	personalTripVOTT   [     122     ]     =     12.2     ;
	personalTripVOTT   [     123     ]     =     12.3     ;
	personalTripVOTT   [     124     ]     =     12.4     ;
	personalTripVOTT   [     125     ]     =     12.5     ;
	personalTripVOTT   [     126     ]     =     12.6     ;
	personalTripVOTT   [     127     ]     =     12.7     ;
	personalTripVOTT   [     128     ]     =     12.8     ;
	personalTripVOTT   [     129     ]     =     12.9     ;
	personalTripVOTT   [     130     ]     =     13     ;
	personalTripVOTT   [     131     ]     =     13.1     ;
	personalTripVOTT   [     132     ]     =     13.2     ;
	personalTripVOTT   [     133     ]     =     13.3     ;
	personalTripVOTT   [     134     ]     =     13.4     ;
	personalTripVOTT   [     135     ]     =     13.5     ;
	personalTripVOTT   [     136     ]     =     13.6     ;
	personalTripVOTT   [     137     ]     =     13.7     ;
	personalTripVOTT   [     138     ]     =     13.8     ;
	personalTripVOTT   [     139     ]     =     13.9     ;
	personalTripVOTT   [     140     ]     =     14     ;
	personalTripVOTT   [     141     ]     =     14.1     ;
	personalTripVOTT   [     142     ]     =     14.2     ;
	personalTripVOTT   [     143     ]     =     14.3     ;
	personalTripVOTT   [     144     ]     =     14.4     ;
	personalTripVOTT   [     145     ]     =     14.5     ;
	personalTripVOTT   [     146     ]     =     14.6     ;
	personalTripVOTT   [     147     ]     =     14.7     ;
	personalTripVOTT   [     148     ]     =     14.8     ;
	personalTripVOTT   [     149     ]     =     14.9     ;
	personalTripVOTT   [     150     ]     =     15     ;
	personalTripVOTT   [     151     ]     =     15.1     ;
	personalTripVOTT   [     152     ]     =     15.2     ;
	personalTripVOTT   [     153     ]     =     15.3     ;
	personalTripVOTT   [     154     ]     =     15.4     ;
	personalTripVOTT   [     155     ]     =     15.5     ;
	personalTripVOTT   [     156     ]     =     15.6     ;
	personalTripVOTT   [     157     ]     =     15.7     ;
	personalTripVOTT   [     158     ]     =     15.8     ;
	personalTripVOTT   [     159     ]     =     15.9     ;
	personalTripVOTT   [     160     ]     =     16     ;
	personalTripVOTT   [     161     ]     =     16.1     ;
	personalTripVOTT   [     162     ]     =     16.2     ;
	personalTripVOTT   [     163     ]     =     16.3     ;
	personalTripVOTT   [     164     ]     =     16.4     ;
	personalTripVOTT   [     165     ]     =     16.5     ;
	personalTripVOTT   [     166     ]     =     16.6     ;
	personalTripVOTT   [     167     ]     =     16.7     ;
	personalTripVOTT   [     168     ]     =     16.8     ;
	personalTripVOTT   [     169     ]     =     16.9     ;
	personalTripVOTT   [     170     ]     =     17     ;
	personalTripVOTT   [     171     ]     =     17.1     ;
	personalTripVOTT   [     172     ]     =     17.2     ;
	personalTripVOTT   [     173     ]     =     17.3     ;
	personalTripVOTT   [     174     ]     =     17.4     ;
	personalTripVOTT   [     175     ]     =     17.5     ;
	personalTripVOTT   [     176     ]     =     17.6     ;
	personalTripVOTT   [     177     ]     =     17.7     ;
	personalTripVOTT   [     178     ]     =     17.8     ;
	personalTripVOTT   [     179     ]     =     17.9     ;
	personalTripVOTT   [     180     ]     =     18     ;
	personalTripVOTT   [     181     ]     =     18.1     ;
	personalTripVOTT   [     182     ]     =     18.2     ;
	personalTripVOTT   [     183     ]     =     18.3     ;
	personalTripVOTT   [     184     ]     =     18.4     ;
	personalTripVOTT   [     185     ]     =     18.5     ;
	personalTripVOTT   [     186     ]     =     18.6     ;
	personalTripVOTT   [     187     ]     =     18.7     ;
	personalTripVOTT   [     188     ]     =     18.8     ;
	personalTripVOTT   [     189     ]     =     18.9     ;
	personalTripVOTT   [     190     ]     =     19     ;
	personalTripVOTT   [     191     ]     =     19.1     ;
	personalTripVOTT   [     192     ]     =     19.2     ;
	personalTripVOTT   [     193     ]     =     19.3     ;
	personalTripVOTT   [     194     ]     =     19.4     ;
	personalTripVOTT   [     195     ]     =     19.5     ;
	personalTripVOTT   [     196     ]     =     19.6     ;
	personalTripVOTT   [     197     ]     =     19.7     ;
	personalTripVOTT   [     198     ]     =     19.8     ;
	personalTripVOTT   [     199     ]     =     19.9     ;
	personalTripVOTT   [     200     ]     =     20     ;
	personalTripVOTT   [     201     ]     =     20.1     ;
	personalTripVOTT   [     202     ]     =     20.2     ;
	personalTripVOTT   [     203     ]     =     20.3     ;
	personalTripVOTT   [     204     ]     =     20.4     ;
	personalTripVOTT   [     205     ]     =     20.5     ;
	personalTripVOTT   [     206     ]     =     20.6     ;
	personalTripVOTT   [     207     ]     =     20.7     ;
	personalTripVOTT   [     208     ]     =     20.8     ;
	personalTripVOTT   [     209     ]     =     20.9     ;
	personalTripVOTT   [     210     ]     =     21     ;
	personalTripVOTT   [     211     ]     =     21.1     ;
	personalTripVOTT   [     212     ]     =     21.2     ;
	personalTripVOTT   [     213     ]     =     21.3     ;
	personalTripVOTT   [     214     ]     =     21.4     ;
	personalTripVOTT   [     215     ]     =     21.5     ;
	personalTripVOTT   [     216     ]     =     21.6     ;
	personalTripVOTT   [     217     ]     =     21.7     ;
	personalTripVOTT   [     218     ]     =     21.8     ;
	personalTripVOTT   [     219     ]     =     21.9     ;
	personalTripVOTT   [     220     ]     =     22     ;
	personalTripVOTT   [     221     ]     =     22.1     ;
	personalTripVOTT   [     222     ]     =     22.2     ;
	personalTripVOTT   [     223     ]     =     22.3     ;
	personalTripVOTT   [     224     ]     =     22.4     ;
	personalTripVOTT   [     225     ]     =     22.5     ;
	personalTripVOTT   [     226     ]     =     22.6     ;
	personalTripVOTT   [     227     ]     =     22.7     ;
	personalTripVOTT   [     228     ]     =     22.8     ;
	personalTripVOTT   [     229     ]     =     22.9     ;
	personalTripVOTT   [     230     ]     =     23     ;
	personalTripVOTT   [     231     ]     =     23.1     ;
	personalTripVOTT   [     232     ]     =     23.2     ;
	personalTripVOTT   [     233     ]     =     23.3     ;
	personalTripVOTT   [     234     ]     =     23.4     ;
	personalTripVOTT   [     235     ]     =     23.5     ;
	personalTripVOTT   [     236     ]     =     23.6     ;
	personalTripVOTT   [     237     ]     =     23.7     ;
	personalTripVOTT   [     238     ]     =     23.8     ;
	personalTripVOTT   [     239     ]     =     23.9     ;
	personalTripVOTT   [     240     ]     =     24     ;
	personalTripVOTT   [     241     ]     =     24.1     ;
	personalTripVOTT   [     242     ]     =     24.2     ;
	personalTripVOTT   [     243     ]     =     24.3     ;
	personalTripVOTT   [     244     ]     =     24.4     ;
	personalTripVOTT   [     245     ]     =     24.5     ;
	personalTripVOTT   [     246     ]     =     24.6     ;
	personalTripVOTT   [     247     ]     =     24.7     ;
	personalTripVOTT   [     248     ]     =     24.8     ;
	personalTripVOTT   [     249     ]     =     24.9     ;
	personalTripVOTT   [     250     ]     =     25     ;
	personalTripVOTT   [     251     ]     =     25.1     ;
	personalTripVOTT   [     252     ]     =     25.2     ;
	personalTripVOTT   [     253     ]     =     25.3     ;
	personalTripVOTT   [     254     ]     =     25.4     ;
	personalTripVOTT   [     255     ]     =     25.5     ;
	personalTripVOTT   [     256     ]     =     25.6     ;
	personalTripVOTT   [     257     ]     =     25.7     ;
	personalTripVOTT   [     258     ]     =     25.8     ;
	personalTripVOTT   [     259     ]     =     25.9     ;
	personalTripVOTT   [     260     ]     =     26     ;
	personalTripVOTT   [     261     ]     =     26.1     ;
	personalTripVOTT   [     262     ]     =     26.2     ;
	personalTripVOTT   [     263     ]     =     26.3     ;
	personalTripVOTT   [     264     ]     =     26.4     ;
	personalTripVOTT   [     265     ]     =     26.5     ;
	personalTripVOTT   [     266     ]     =     26.6     ;
	personalTripVOTT   [     267     ]     =     26.7     ;
	personalTripVOTT   [     268     ]     =     26.8     ;
	personalTripVOTT   [     269     ]     =     26.9     ;
	personalTripVOTT   [     270     ]     =     27     ;
	personalTripVOTT   [     271     ]     =     27.1     ;
	personalTripVOTT   [     272     ]     =     27.2     ;
	personalTripVOTT   [     273     ]     =     27.3     ;
	personalTripVOTT   [     274     ]     =     27.4     ;
	personalTripVOTT   [     275     ]     =     27.5     ;
	personalTripVOTT   [     276     ]     =     27.6     ;
	personalTripVOTT   [     277     ]     =     27.7     ;
	personalTripVOTT   [     278     ]     =     27.8     ;
	personalTripVOTT   [     279     ]     =     27.9     ;
	personalTripVOTT   [     280     ]     =     28     ;
	personalTripVOTT   [     281     ]     =     28.1     ;
	personalTripVOTT   [     282     ]     =     28.2     ;
	personalTripVOTT   [     283     ]     =     28.3     ;
	personalTripVOTT   [     284     ]     =     28.4     ;
	personalTripVOTT   [     285     ]     =     28.5     ;
	personalTripVOTT   [     286     ]     =     28.6     ;
	personalTripVOTT   [     287     ]     =     28.7     ;
	personalTripVOTT   [     288     ]     =     28.8     ;
	personalTripVOTT   [     289     ]     =     28.9     ;
	personalTripVOTT   [     290     ]     =     29     ;
	personalTripVOTT   [     291     ]     =     29.1     ;
	personalTripVOTT   [     292     ]     =     29.2     ;
	personalTripVOTT   [     293     ]     =     29.3     ;
	personalTripVOTT   [     294     ]     =     29.4     ;
	personalTripVOTT   [     295     ]     =     29.5     ;
	personalTripVOTT   [     296     ]     =     29.6     ;
	personalTripVOTT   [     297     ]     =     29.7     ;
	personalTripVOTT   [     298     ]     =     29.8     ;
	personalTripVOTT   [     299     ]     =     29.9     ;
	personalTripVOTT   [     300     ]     =     30     ;
	personalTripVOTT   [     301     ]     =     30.1     ;
	personalTripVOTT   [     302     ]     =     30.2     ;
	personalTripVOTT   [     303     ]     =     30.3     ;
	personalTripVOTT   [     304     ]     =     30.4     ;
	personalTripVOTT   [     305     ]     =     30.5     ;
	personalTripVOTT   [     306     ]     =     30.6     ;
	personalTripVOTT   [     307     ]     =     30.7     ;
	personalTripVOTT   [     308     ]     =     30.8     ;
	personalTripVOTT   [     309     ]     =     30.9     ;
	personalTripVOTT   [     310     ]     =     31     ;
	personalTripVOTT   [     311     ]     =     31.1     ;
	personalTripVOTT   [     312     ]     =     31.2     ;
	personalTripVOTT   [     313     ]     =     31.3     ;
	personalTripVOTT   [     314     ]     =     31.4     ;
	personalTripVOTT   [     315     ]     =     31.5     ;
	personalTripVOTT   [     316     ]     =     31.6     ;
	personalTripVOTT   [     317     ]     =     31.7     ;
	personalTripVOTT   [     318     ]     =     31.8     ;
	personalTripVOTT   [     319     ]     =     31.9     ;
	personalTripVOTT   [     320     ]     =     32     ;
	personalTripVOTT   [     321     ]     =     32.1     ;
	personalTripVOTT   [     322     ]     =     32.2     ;
	personalTripVOTT   [     323     ]     =     32.3     ;
	personalTripVOTT   [     324     ]     =     32.4     ;
	personalTripVOTT   [     325     ]     =     32.5     ;
	personalTripVOTT   [     326     ]     =     32.6     ;
	personalTripVOTT   [     327     ]     =     32.7     ;
	personalTripVOTT   [     328     ]     =     32.8     ;
	personalTripVOTT   [     329     ]     =     32.9     ;
	personalTripVOTT   [     330     ]     =     33     ;
	personalTripVOTT   [     331     ]     =     33.1     ;
	personalTripVOTT   [     332     ]     =     33.2     ;
	personalTripVOTT   [     333     ]     =     33.3     ;
	personalTripVOTT   [     334     ]     =     33.4     ;
	personalTripVOTT   [     335     ]     =     33.5     ;
	personalTripVOTT   [     336     ]     =     33.6     ;
	personalTripVOTT   [     337     ]     =     33.7     ;
	personalTripVOTT   [     338     ]     =     33.8     ;
	personalTripVOTT   [     339     ]     =     33.9     ;
	personalTripVOTT   [     340     ]     =     34     ;
	personalTripVOTT   [     341     ]     =     34.1     ;
	personalTripVOTT   [     342     ]     =     34.2     ;
	personalTripVOTT   [     343     ]     =     34.3     ;
	personalTripVOTT   [     344     ]     =     34.4     ;
	personalTripVOTT   [     345     ]     =     34.5     ;
	personalTripVOTT   [     346     ]     =     34.6     ;
	personalTripVOTT   [     347     ]     =     34.7     ;
	personalTripVOTT   [     348     ]     =     34.8     ;
	personalTripVOTT   [     349     ]     =     34.9     ;
	personalTripVOTT   [     350     ]     =     35     ;
	personalTripVOTT   [     351     ]     =     35.1     ;
	personalTripVOTT   [     352     ]     =     35.2     ;
	personalTripVOTT   [     353     ]     =     35.3     ;
	personalTripVOTT   [     354     ]     =     35.4     ;
	personalTripVOTT   [     355     ]     =     35.5     ;
	personalTripVOTT   [     356     ]     =     35.6     ;
	personalTripVOTT   [     357     ]     =     35.7     ;
	personalTripVOTT   [     358     ]     =     35.8     ;
	personalTripVOTT   [     359     ]     =     35.9     ;
	personalTripVOTT   [     360     ]     =     36     ;
	personalTripVOTT   [     361     ]     =     36.1     ;
	personalTripVOTT   [     362     ]     =     36.2     ;
	personalTripVOTT   [     363     ]     =     36.3     ;
	personalTripVOTT   [     364     ]     =     36.4     ;
	personalTripVOTT   [     365     ]     =     36.5     ;
	personalTripVOTT   [     366     ]     =     36.6     ;
	personalTripVOTT   [     367     ]     =     36.7     ;
	personalTripVOTT   [     368     ]     =     36.8     ;
	personalTripVOTT   [     369     ]     =     36.9     ;
	personalTripVOTT   [     370     ]     =     37     ;
	personalTripVOTT   [     371     ]     =     37.1     ;
	personalTripVOTT   [     372     ]     =     37.2     ;
	personalTripVOTT   [     373     ]     =     37.3     ;
	personalTripVOTT   [     374     ]     =     37.4     ;
	personalTripVOTT   [     375     ]     =     37.5     ;
	personalTripVOTT   [     376     ]     =     37.6     ;
	personalTripVOTT   [     377     ]     =     37.7     ;
	personalTripVOTT   [     378     ]     =     37.8     ;
	personalTripVOTT   [     379     ]     =     37.9     ;
	personalTripVOTT   [     380     ]     =     38     ;
	personalTripVOTT   [     381     ]     =     38.1     ;
	personalTripVOTT   [     382     ]     =     38.2     ;
	personalTripVOTT   [     383     ]     =     38.3     ;
	personalTripVOTT   [     384     ]     =     38.4     ;
	personalTripVOTT   [     385     ]     =     38.5     ;
	personalTripVOTT   [     386     ]     =     38.6     ;
	personalTripVOTT   [     387     ]     =     38.7     ;
	personalTripVOTT   [     388     ]     =     38.8     ;
	personalTripVOTT   [     389     ]     =     38.9     ;
	personalTripVOTT   [     390     ]     =     39     ;
	personalTripVOTT   [     391     ]     =     39.1     ;
	personalTripVOTT   [     392     ]     =     39.2     ;
	personalTripVOTT   [     393     ]     =     39.3     ;
	personalTripVOTT   [     394     ]     =     39.4     ;
	personalTripVOTT   [     395     ]     =     39.5     ;
	personalTripVOTT   [     396     ]     =     39.6     ;
	personalTripVOTT   [     397     ]     =     39.7     ;
	personalTripVOTT   [     398     ]     =     39.8     ;
	personalTripVOTT   [     399     ]     =     39.9     ;
	personalTripVOTT   [     400     ]     =     40     ;
	personalTripVOTT   [     401     ]     =     40.1     ;
	personalTripVOTT   [     402     ]     =     40.2     ;
	personalTripVOTT   [     403     ]     =     40.3     ;
	personalTripVOTT   [     404     ]     =     40.4     ;
	personalTripVOTT   [     405     ]     =     40.5     ;
	personalTripVOTT   [     406     ]     =     40.6     ;
	personalTripVOTT   [     407     ]     =     40.7     ;
	personalTripVOTT   [     408     ]     =     40.8     ;
	personalTripVOTT   [     409     ]     =     40.9     ;
	personalTripVOTT   [     410     ]     =     41     ;
	personalTripVOTT   [     411     ]     =     41.1     ;
	personalTripVOTT   [     412     ]     =     41.2     ;
	personalTripVOTT   [     413     ]     =     41.3     ;
	personalTripVOTT   [     414     ]     =     41.4     ;
	personalTripVOTT   [     415     ]     =     41.5     ;
	personalTripVOTT   [     416     ]     =     41.6     ;
	personalTripVOTT   [     417     ]     =     41.7     ;
	personalTripVOTT   [     418     ]     =     41.8     ;
	personalTripVOTT   [     419     ]     =     41.9     ;
	personalTripVOTT   [     420     ]     =     42     ;
	personalTripVOTT   [     421     ]     =     42.1     ;
	personalTripVOTT   [     422     ]     =     42.2     ;
	personalTripVOTT   [     423     ]     =     42.3     ;
	personalTripVOTT   [     424     ]     =     42.4     ;
	personalTripVOTT   [     425     ]     =     42.5     ;
	personalTripVOTT   [     426     ]     =     42.6     ;
	personalTripVOTT   [     427     ]     =     42.7     ;
	personalTripVOTT   [     428     ]     =     42.8     ;
	personalTripVOTT   [     429     ]     =     42.9     ;
	personalTripVOTT   [     430     ]     =     43     ;
	personalTripVOTT   [     431     ]     =     43.1     ;
	personalTripVOTT   [     432     ]     =     43.2     ;
	personalTripVOTT   [     433     ]     =     43.3     ;
	personalTripVOTT   [     434     ]     =     43.4     ;
	personalTripVOTT   [     435     ]     =     43.5     ;
	personalTripVOTT   [     436     ]     =     43.6     ;
	personalTripVOTT   [     437     ]     =     43.7     ;
	personalTripVOTT   [     438     ]     =     43.8     ;
	personalTripVOTT   [     439     ]     =     43.9     ;
	personalTripVOTT   [     440     ]     =     44     ;
	personalTripVOTT   [     441     ]     =     44.1     ;
	personalTripVOTT   [     442     ]     =     44.2     ;
	personalTripVOTT   [     443     ]     =     44.3     ;
	personalTripVOTT   [     444     ]     =     44.4     ;
	personalTripVOTT   [     445     ]     =     44.5     ;
	personalTripVOTT   [     446     ]     =     44.6     ;
	personalTripVOTT   [     447     ]     =     44.7     ;
	personalTripVOTT   [     448     ]     =     44.8     ;
	personalTripVOTT   [     449     ]     =     44.9     ;
	personalTripVOTT   [     450     ]     =     45     ;
	personalTripVOTT   [     451     ]     =     45.1     ;
	personalTripVOTT   [     452     ]     =     45.2     ;
	personalTripVOTT   [     453     ]     =     45.3     ;
	personalTripVOTT   [     454     ]     =     45.4     ;
	personalTripVOTT   [     455     ]     =     45.5     ;
	personalTripVOTT   [     456     ]     =     45.6     ;
	personalTripVOTT   [     457     ]     =     45.7     ;
	personalTripVOTT   [     458     ]     =     45.8     ;
	personalTripVOTT   [     459     ]     =     45.9     ;
	personalTripVOTT   [     460     ]     =     46     ;
	personalTripVOTT   [     461     ]     =     46.1     ;
	personalTripVOTT   [     462     ]     =     46.2     ;
	personalTripVOTT   [     463     ]     =     46.3     ;
	personalTripVOTT   [     464     ]     =     46.4     ;
	personalTripVOTT   [     465     ]     =     46.5     ;
	personalTripVOTT   [     466     ]     =     46.6     ;
	personalTripVOTT   [     467     ]     =     46.7     ;
	personalTripVOTT   [     468     ]     =     46.8     ;
	personalTripVOTT   [     469     ]     =     46.9     ;
	personalTripVOTT   [     470     ]     =     47     ;
	personalTripVOTT   [     471     ]     =     47.1     ;
	personalTripVOTT   [     472     ]     =     47.2     ;
	personalTripVOTT   [     473     ]     =     47.3     ;
	personalTripVOTT   [     474     ]     =     47.4     ;
	personalTripVOTT   [     475     ]     =     47.5     ;
	personalTripVOTT   [     476     ]     =     47.6     ;
	personalTripVOTT   [     477     ]     =     47.7     ;
	personalTripVOTT   [     478     ]     =     47.8     ;
	personalTripVOTT   [     479     ]     =     47.9     ;
	personalTripVOTT   [     480     ]     =     48     ;
	personalTripVOTT   [     481     ]     =     48.1     ;
	personalTripVOTT   [     482     ]     =     48.2     ;
	personalTripVOTT   [     483     ]     =     48.3     ;
}

void Simulator::setPersonalTripProbability()
{
	personalTripProbability = new double[484];
	personalTripProbability  [     0     ] =   0.0000   ;
	personalTripProbability  [     1     ] =   0.0056   ;
	personalTripProbability  [     2     ] =   0.0112   ;
	personalTripProbability  [     3     ] =   0.0168   ;
	personalTripProbability  [     4     ] =   0.0224   ;
	personalTripProbability  [     5     ] =   0.0280   ;
	personalTripProbability  [     6     ] =   0.0337   ;
	personalTripProbability  [     7     ] =   0.0393   ;
	personalTripProbability  [     8     ] =   0.0449   ;
	personalTripProbability  [     9     ] =   0.0505   ;
	personalTripProbability  [     10     ] =   0.0561   ;
	personalTripProbability  [     11     ] =   0.0617   ;
	personalTripProbability  [     12     ] =   0.0673   ;
	personalTripProbability  [     13     ] =   0.0729   ;
	personalTripProbability  [     14     ] =   0.0784   ;
	personalTripProbability  [     15     ] =   0.0838   ;
	personalTripProbability  [     16     ] =   0.0893   ;
	personalTripProbability  [     17     ] =   0.0948   ;
	personalTripProbability  [     18     ] =   0.1003   ;
	personalTripProbability  [     19     ] =   0.1057   ;
	personalTripProbability  [     20     ] =   0.1112   ;
	personalTripProbability  [     21     ] =   0.1167   ;
	personalTripProbability  [     22     ] =   0.1221   ;
	personalTripProbability  [     23     ] =   0.1276   ;
	personalTripProbability  [     24     ] =   0.1331   ;
	personalTripProbability  [     25     ] =   0.1385   ;
	personalTripProbability  [     26     ] =   0.1452   ;
	personalTripProbability  [     27     ] =   0.1518   ;
	personalTripProbability  [     28     ] =   0.1585   ;
	personalTripProbability  [     29     ] =   0.1652   ;
	personalTripProbability  [     30     ] =   0.1718   ;
	personalTripProbability  [     31     ] =   0.1785   ;
	personalTripProbability  [     32     ] =   0.1851   ;
	personalTripProbability  [     33     ] =   0.1918   ;
	personalTripProbability  [     34     ] =   0.1984   ;
	personalTripProbability  [     35     ] =   0.2051   ;
	personalTripProbability  [     36     ] =   0.2118   ;
	personalTripProbability  [     37     ] =   0.2184   ;
	personalTripProbability  [     38     ] =   0.2251   ;
	personalTripProbability  [     39     ] =   0.2314   ;
	personalTripProbability  [     40     ] =   0.2378   ;
	personalTripProbability  [     41     ] =   0.2442   ;
	personalTripProbability  [     42     ] =   0.2505   ;
	personalTripProbability  [     43     ] =   0.2569   ;
	personalTripProbability  [     44     ] =   0.2632   ;
	personalTripProbability  [     45     ] =   0.2696   ;
	personalTripProbability  [     46     ] =   0.2760   ;
	personalTripProbability  [     47     ] =   0.2823   ;
	personalTripProbability  [     48     ] =   0.2887   ;
	personalTripProbability  [     49     ] =   0.2950   ;
	personalTripProbability  [     50     ] =   0.3014   ;
	personalTripProbability  [     51     ] =   0.3078   ;
	personalTripProbability  [     52     ] =   0.3141   ;
	personalTripProbability  [     53     ] =   0.3205   ;
	personalTripProbability  [     54     ] =   0.3268   ;
	personalTripProbability  [     55     ] =   0.3332   ;
	personalTripProbability  [     56     ] =   0.3396   ;
	personalTripProbability  [     57     ] =   0.3459   ;
	personalTripProbability  [     58     ] =   0.3523   ;
	personalTripProbability  [     59     ] =   0.3587   ;
	personalTripProbability  [     60     ] =   0.3650   ;
	personalTripProbability  [     61     ] =   0.3714   ;
	personalTripProbability  [     62     ] =   0.3777   ;
	personalTripProbability  [     63     ] =   0.3841   ;
	personalTripProbability  [     64     ] =   0.3893   ;
	personalTripProbability  [     65     ] =   0.3944   ;
	personalTripProbability  [     66     ] =   0.3996   ;
	personalTripProbability  [     67     ] =   0.4048   ;
	personalTripProbability  [     68     ] =   0.4100   ;
	personalTripProbability  [     69     ] =   0.4152   ;
	personalTripProbability  [     70     ] =   0.4203   ;
	personalTripProbability  [     71     ] =   0.4255   ;
	personalTripProbability  [     72     ] =   0.4307   ;
	personalTripProbability  [     73     ] =   0.4359   ;
	personalTripProbability  [     74     ] =   0.4410   ;
	personalTripProbability  [     75     ] =   0.4462   ;
	personalTripProbability  [     76     ] =   0.4514   ;
	personalTripProbability  [     77     ] =   0.4566   ;
	personalTripProbability  [     78     ] =   0.4617   ;
	personalTripProbability  [     79     ] =   0.4669   ;
	personalTripProbability  [     80     ] =   0.4721   ;
	personalTripProbability  [     81     ] =   0.4773   ;
	personalTripProbability  [     82     ] =   0.4825   ;
	personalTripProbability  [     83     ] =   0.4876   ;
	personalTripProbability  [     84     ] =   0.4928   ;
	personalTripProbability  [     85     ] =   0.4980   ;
	personalTripProbability  [     86     ] =   0.5032   ;
	personalTripProbability  [     87     ] =   0.5083   ;
	personalTripProbability  [     88     ] =   0.5135   ;
	personalTripProbability  [     89     ] =   0.5176   ;
	personalTripProbability  [     90     ] =   0.5216   ;
	personalTripProbability  [     91     ] =   0.5257   ;
	personalTripProbability  [     92     ] =   0.5297   ;
	personalTripProbability  [     93     ] =   0.5338   ;
	personalTripProbability  [     94     ] =   0.5379   ;
	personalTripProbability  [     95     ] =   0.5419   ;
	personalTripProbability  [     96     ] =   0.5460   ;
	personalTripProbability  [     97     ] =   0.5500   ;
	personalTripProbability  [     98     ] =   0.5541   ;
	personalTripProbability  [     99     ] =   0.5582   ;
	personalTripProbability  [     100     ] =   0.5622   ;
	personalTripProbability  [     101     ] =   0.5663   ;
	personalTripProbability  [     102     ] =   0.5703   ;
	personalTripProbability  [     103     ] =   0.5744   ;
	personalTripProbability  [     104     ] =   0.5785   ;
	personalTripProbability  [     105     ] =   0.5825   ;
	personalTripProbability  [     106     ] =   0.5866   ;
	personalTripProbability  [     107     ] =   0.5906   ;
	personalTripProbability  [     108     ] =   0.5947   ;
	personalTripProbability  [     109     ] =   0.5987   ;
	personalTripProbability  [     110     ] =   0.6028   ;
	personalTripProbability  [     111     ] =   0.6069   ;
	personalTripProbability  [     112     ] =   0.6109   ;
	personalTripProbability  [     113     ] =   0.6150   ;
	personalTripProbability  [     114     ] =   0.6190   ;
	personalTripProbability  [     115     ] =   0.6231   ;
	personalTripProbability  [     116     ] =   0.6272   ;
	personalTripProbability  [     117     ] =   0.6312   ;
	personalTripProbability  [     118     ] =   0.6353   ;
	personalTripProbability  [     119     ] =   0.6393   ;
	personalTripProbability  [     120     ] =   0.6434   ;
	personalTripProbability  [     121     ] =   0.6475   ;
	personalTripProbability  [     122     ] =   0.6515   ;
	personalTripProbability  [     123     ] =   0.6556   ;
	personalTripProbability  [     124     ] =   0.6596   ;
	personalTripProbability  [     125     ] =   0.6637   ;
	personalTripProbability  [     126     ] =   0.6662   ;
	personalTripProbability  [     127     ] =   0.6688   ;
	personalTripProbability  [     128     ] =   0.6713   ;
	personalTripProbability  [     129     ] =   0.6738   ;
	personalTripProbability  [     130     ] =   0.6764   ;
	personalTripProbability  [     131     ] =   0.6789   ;
	personalTripProbability  [     132     ] =   0.6814   ;
	personalTripProbability  [     133     ] =   0.6839   ;
	personalTripProbability  [     134     ] =   0.6865   ;
	personalTripProbability  [     135     ] =   0.6890   ;
	personalTripProbability  [     136     ] =   0.6915   ;
	personalTripProbability  [     137     ] =   0.6941   ;
	personalTripProbability  [     138     ] =   0.6966   ;
	personalTripProbability  [     139     ] =   0.6991   ;
	personalTripProbability  [     140     ] =   0.7017   ;
	personalTripProbability  [     141     ] =   0.7042   ;
	personalTripProbability  [     142     ] =   0.7067   ;
	personalTripProbability  [     143     ] =   0.7093   ;
	personalTripProbability  [     144     ] =   0.7118   ;
	personalTripProbability  [     145     ] =   0.7143   ;
	personalTripProbability  [     146     ] =   0.7169   ;
	personalTripProbability  [     147     ] =   0.7194   ;
	personalTripProbability  [     148     ] =   0.7219   ;
	personalTripProbability  [     149     ] =   0.7245   ;
	personalTripProbability  [     150     ] =   0.7270   ;
	personalTripProbability  [     151     ] =   0.7295   ;
	personalTripProbability  [     152     ] =   0.7321   ;
	personalTripProbability  [     153     ] =   0.7346   ;
	personalTripProbability  [     154     ] =   0.7371   ;
	personalTripProbability  [     155     ] =   0.7397   ;
	personalTripProbability  [     156     ] =   0.7422   ;
	personalTripProbability  [     157     ] =   0.7447   ;
	personalTripProbability  [     158     ] =   0.7473   ;
	personalTripProbability  [     159     ] =   0.7498   ;
	personalTripProbability  [     160     ] =   0.7523   ;
	personalTripProbability  [     161     ] =   0.7549   ;
	personalTripProbability  [     162     ] =   0.7574   ;
	personalTripProbability  [     163     ] =   0.7599   ;
	personalTripProbability  [     164     ] =   0.7625   ;
	personalTripProbability  [     165     ] =   0.7650   ;
	personalTripProbability  [     166     ] =   0.7675   ;
	personalTripProbability  [     167     ] =   0.7701   ;
	personalTripProbability  [     168     ] =   0.7726   ;
	personalTripProbability  [     169     ] =   0.7751   ;
	personalTripProbability  [     170     ] =   0.7776   ;
	personalTripProbability  [     171     ] =   0.7802   ;
	personalTripProbability  [     172     ] =   0.7827   ;
	personalTripProbability  [     173     ] =   0.7852   ;
	personalTripProbability  [     174     ] =   0.7878   ;
	personalTripProbability  [     175     ] =   0.7903   ;
	personalTripProbability  [     176     ] =   0.7928   ;
	personalTripProbability  [     177     ] =   0.7954   ;
	personalTripProbability  [     178     ] =   0.7979   ;
	personalTripProbability  [     179     ] =   0.8004   ;
	personalTripProbability  [     180     ] =   0.8030   ;
	personalTripProbability  [     181     ] =   0.8055   ;
	personalTripProbability  [     182     ] =   0.8080   ;
	personalTripProbability  [     183     ] =   0.8106   ;
	personalTripProbability  [     184     ] =   0.8131   ;
	personalTripProbability  [     185     ] =   0.8156   ;
	personalTripProbability  [     186     ] =   0.8182   ;
	personalTripProbability  [     187     ] =   0.8207   ;
	personalTripProbability  [     188     ] =   0.8232   ;
	personalTripProbability  [     189     ] =   0.8238   ;
	personalTripProbability  [     190     ] =   0.8244   ;
	personalTripProbability  [     191     ] =   0.8250   ;
	personalTripProbability  [     192     ] =   0.8256   ;
	personalTripProbability  [     193     ] =   0.8262   ;
	personalTripProbability  [     194     ] =   0.8268   ;
	personalTripProbability  [     195     ] =   0.8274   ;
	personalTripProbability  [     196     ] =   0.8280   ;
	personalTripProbability  [     197     ] =   0.8286   ;
	personalTripProbability  [     198     ] =   0.8292   ;
	personalTripProbability  [     199     ] =   0.8298   ;
	personalTripProbability  [     200     ] =   0.8304   ;
	personalTripProbability  [     201     ] =   0.8310   ;
	personalTripProbability  [     202     ] =   0.8316   ;
	personalTripProbability  [     203     ] =   0.8322   ;
	personalTripProbability  [     204     ] =   0.8328   ;
	personalTripProbability  [     205     ] =   0.8334   ;
	personalTripProbability  [     206     ] =   0.8340   ;
	personalTripProbability  [     207     ] =   0.8346   ;
	personalTripProbability  [     208     ] =   0.8352   ;
	personalTripProbability  [     209     ] =   0.8358   ;
	personalTripProbability  [     210     ] =   0.8364   ;
	personalTripProbability  [     211     ] =   0.8370   ;
	personalTripProbability  [     212     ] =   0.8376   ;
	personalTripProbability  [     213     ] =   0.8382   ;
	personalTripProbability  [     214     ] =   0.8388   ;
	personalTripProbability  [     215     ] =   0.8394   ;
	personalTripProbability  [     216     ] =   0.8400   ;
	personalTripProbability  [     217     ] =   0.8406   ;
	personalTripProbability  [     218     ] =   0.8412   ;
	personalTripProbability  [     219     ] =   0.8418   ;
	personalTripProbability  [     220     ] =   0.8424   ;
	personalTripProbability  [     221     ] =   0.8430   ;
	personalTripProbability  [     222     ] =   0.8436   ;
	personalTripProbability  [     223     ] =   0.8442   ;
	personalTripProbability  [     224     ] =   0.8448   ;
	personalTripProbability  [     225     ] =   0.8454   ;
	personalTripProbability  [     226     ] =   0.8460   ;
	personalTripProbability  [     227     ] =   0.8466   ;
	personalTripProbability  [     228     ] =   0.8472   ;
	personalTripProbability  [     229     ] =   0.8478   ;
	personalTripProbability  [     230     ] =   0.8484   ;
	personalTripProbability  [     231     ] =   0.8490   ;
	personalTripProbability  [     232     ] =   0.8496   ;
	personalTripProbability  [     233     ] =   0.8502   ;
	personalTripProbability  [     234     ] =   0.8508   ;
	personalTripProbability  [     235     ] =   0.8514   ;
	personalTripProbability  [     236     ] =   0.8520   ;
	personalTripProbability  [     237     ] =   0.8526   ;
	personalTripProbability  [     238     ] =   0.8532   ;
	personalTripProbability  [     239     ] =   0.8538   ;
	personalTripProbability  [     240     ] =   0.8544   ;
	personalTripProbability  [     241     ] =   0.8550   ;
	personalTripProbability  [     242     ] =   0.8556   ;
	personalTripProbability  [     243     ] =   0.8562   ;
	personalTripProbability  [     244     ] =   0.8568   ;
	personalTripProbability  [     245     ] =   0.8574   ;
	personalTripProbability  [     246     ] =   0.8580   ;
	personalTripProbability  [     247     ] =   0.8586   ;
	personalTripProbability  [     248     ] =   0.8592   ;
	personalTripProbability  [     249     ] =   0.8598   ;
	personalTripProbability  [     250     ] =   0.8604   ;
	personalTripProbability  [     251     ] =   0.8610   ;
	personalTripProbability  [     252     ] =   0.8616   ;
	personalTripProbability  [     253     ] =   0.8622   ;
	personalTripProbability  [     254     ] =   0.8628   ;
	personalTripProbability  [     255     ] =   0.8634   ;
	personalTripProbability  [     256     ] =   0.8640   ;
	personalTripProbability  [     257     ] =   0.8646   ;
	personalTripProbability  [     258     ] =   0.8652   ;
	personalTripProbability  [     259     ] =   0.8658   ;
	personalTripProbability  [     260     ] =   0.8664   ;
	personalTripProbability  [     261     ] =   0.8670   ;
	personalTripProbability  [     262     ] =   0.8676   ;
	personalTripProbability  [     263     ] =   0.8682   ;
	personalTripProbability  [     264     ] =   0.8688   ;
	personalTripProbability  [     265     ] =   0.8694   ;
	personalTripProbability  [     266     ] =   0.8700   ;
	personalTripProbability  [     267     ] =   0.8706   ;
	personalTripProbability  [     268     ] =   0.8712   ;
	personalTripProbability  [     269     ] =   0.8718   ;
	personalTripProbability  [     270     ] =   0.8724   ;
	personalTripProbability  [     271     ] =   0.8730   ;
	personalTripProbability  [     272     ] =   0.8736   ;
	personalTripProbability  [     273     ] =   0.8742   ;
	personalTripProbability  [     274     ] =   0.8748   ;
	personalTripProbability  [     275     ] =   0.8754   ;
	personalTripProbability  [     276     ] =   0.8760   ;
	personalTripProbability  [     277     ] =   0.8766   ;
	personalTripProbability  [     278     ] =   0.8772   ;
	personalTripProbability  [     279     ] =   0.8778   ;
	personalTripProbability  [     280     ] =   0.8784   ;
	personalTripProbability  [     281     ] =   0.8790   ;
	personalTripProbability  [     282     ] =   0.8796   ;
	personalTripProbability  [     283     ] =   0.8802   ;
	personalTripProbability  [     284     ] =   0.8808   ;
	personalTripProbability  [     285     ] =   0.8814   ;
	personalTripProbability  [     286     ] =   0.8820   ;
	personalTripProbability  [     287     ] =   0.8826   ;
	personalTripProbability  [     288     ] =   0.8832   ;
	personalTripProbability  [     289     ] =   0.8838   ;
	personalTripProbability  [     290     ] =   0.8844   ;
	personalTripProbability  [     291     ] =   0.8850   ;
	personalTripProbability  [     292     ] =   0.8856   ;
	personalTripProbability  [     293     ] =   0.8862   ;
	personalTripProbability  [     294     ] =   0.8868   ;
	personalTripProbability  [     295     ] =   0.8873   ;
	personalTripProbability  [     296     ] =   0.8879   ;
	personalTripProbability  [     297     ] =   0.8885   ;
	personalTripProbability  [     298     ] =   0.8891   ;
	personalTripProbability  [     299     ] =   0.8897   ;
	personalTripProbability  [     300     ] =   0.8903   ;
	personalTripProbability  [     301     ] =   0.8909   ;
	personalTripProbability  [     302     ] =   0.8915   ;
	personalTripProbability  [     303     ] =   0.8921   ;
	personalTripProbability  [     304     ] =   0.8927   ;
	personalTripProbability  [     305     ] =   0.8933   ;
	personalTripProbability  [     306     ] =   0.8939   ;
	personalTripProbability  [     307     ] =   0.8945   ;
	personalTripProbability  [     308     ] =   0.8951   ;
	personalTripProbability  [     309     ] =   0.8957   ;
	personalTripProbability  [     310     ] =   0.8963   ;
	personalTripProbability  [     311     ] =   0.8969   ;
	personalTripProbability  [     312     ] =   0.8975   ;
	personalTripProbability  [     313     ] =   0.8981   ;
	personalTripProbability  [     314     ] =   0.8987   ;
	personalTripProbability  [     315     ] =   0.8993   ;
	personalTripProbability  [     316     ] =   0.8999   ;
	personalTripProbability  [     317     ] =   0.9005   ;
	personalTripProbability  [     318     ] =   0.9011   ;
	personalTripProbability  [     319     ] =   0.9017   ;
	personalTripProbability  [     320     ] =   0.9023   ;
	personalTripProbability  [     321     ] =   0.9029   ;
	personalTripProbability  [     322     ] =   0.9035   ;
	personalTripProbability  [     323     ] =   0.9041   ;
	personalTripProbability  [     324     ] =   0.9047   ;
	personalTripProbability  [     325     ] =   0.9053   ;
	personalTripProbability  [     326     ] =   0.9059   ;
	personalTripProbability  [     327     ] =   0.9065   ;
	personalTripProbability  [     328     ] =   0.9071   ;
	personalTripProbability  [     329     ] =   0.9077   ;
	personalTripProbability  [     330     ] =   0.9083   ;
	personalTripProbability  [     331     ] =   0.9089   ;
	personalTripProbability  [     332     ] =   0.9095   ;
	personalTripProbability  [     333     ] =   0.9101   ;
	personalTripProbability  [     334     ] =   0.9107   ;
	personalTripProbability  [     335     ] =   0.9113   ;
	personalTripProbability  [     336     ] =   0.9119   ;
	personalTripProbability  [     337     ] =   0.9125   ;
	personalTripProbability  [     338     ] =   0.9131   ;
	personalTripProbability  [     339     ] =   0.9137   ;
	personalTripProbability  [     340     ] =   0.9143   ;
	personalTripProbability  [     341     ] =   0.9149   ;
	personalTripProbability  [     342     ] =   0.9155   ;
	personalTripProbability  [     343     ] =   0.9161   ;
	personalTripProbability  [     344     ] =   0.9167   ;
	personalTripProbability  [     345     ] =   0.9173   ;
	personalTripProbability  [     346     ] =   0.9179   ;
	personalTripProbability  [     347     ] =   0.9185   ;
	personalTripProbability  [     348     ] =   0.9191   ;
	personalTripProbability  [     349     ] =   0.9197   ;
	personalTripProbability  [     350     ] =   0.9203   ;
	personalTripProbability  [     351     ] =   0.9209   ;
	personalTripProbability  [     352     ] =   0.9215   ;
	personalTripProbability  [     353     ] =   0.9221   ;
	personalTripProbability  [     354     ] =   0.9227   ;
	personalTripProbability  [     355     ] =   0.9233   ;
	personalTripProbability  [     356     ] =   0.9239   ;
	personalTripProbability  [     357     ] =   0.9245   ;
	personalTripProbability  [     358     ] =   0.9251   ;
	personalTripProbability  [     359     ] =   0.9257   ;
	personalTripProbability  [     360     ] =   0.9263   ;
	personalTripProbability  [     361     ] =   0.9269   ;
	personalTripProbability  [     362     ] =   0.9275   ;
	personalTripProbability  [     363     ] =   0.9281   ;
	personalTripProbability  [     364     ] =   0.9287   ;
	personalTripProbability  [     365     ] =   0.9293   ;
	personalTripProbability  [     366     ] =   0.9299   ;
	personalTripProbability  [     367     ] =   0.9305   ;
	personalTripProbability  [     368     ] =   0.9311   ;
	personalTripProbability  [     369     ] =   0.9317   ;
	personalTripProbability  [     370     ] =   0.9323   ;
	personalTripProbability  [     371     ] =   0.9329   ;
	personalTripProbability  [     372     ] =   0.9335   ;
	personalTripProbability  [     373     ] =   0.9341   ;
	personalTripProbability  [     374     ] =   0.9347   ;
	personalTripProbability  [     375     ] =   0.9353   ;
	personalTripProbability  [     376     ] =   0.9359   ;
	personalTripProbability  [     377     ] =   0.9365   ;
	personalTripProbability  [     378     ] =   0.9371   ;
	personalTripProbability  [     379     ] =   0.9377   ;
	personalTripProbability  [     380     ] =   0.9383   ;
	personalTripProbability  [     381     ] =   0.9389   ;
	personalTripProbability  [     382     ] =   0.9395   ;
	personalTripProbability  [     383     ] =   0.9401   ;
	personalTripProbability  [     384     ] =   0.9407   ;
	personalTripProbability  [     385     ] =   0.9413   ;
	personalTripProbability  [     386     ] =   0.9419   ;
	personalTripProbability  [     387     ] =   0.9425   ;
	personalTripProbability  [     388     ] =   0.9431   ;
	personalTripProbability  [     389     ] =   0.9437   ;
	personalTripProbability  [     390     ] =   0.9443   ;
	personalTripProbability  [     391     ] =   0.9449   ;
	personalTripProbability  [     392     ] =   0.9455   ;
	personalTripProbability  [     393     ] =   0.9461   ;
	personalTripProbability  [     394     ] =   0.9467   ;
	personalTripProbability  [     395     ] =   0.9473   ;
	personalTripProbability  [     396     ] =   0.9479   ;
	personalTripProbability  [     397     ] =   0.9485   ;
	personalTripProbability  [     398     ] =   0.9491   ;
	personalTripProbability  [     399     ] =   0.9497   ;
	personalTripProbability  [     400     ] =   0.9503   ;
	personalTripProbability  [     401     ] =   0.9509   ;
	personalTripProbability  [     402     ] =   0.9515   ;
	personalTripProbability  [     403     ] =   0.9521   ;
	personalTripProbability  [     404     ] =   0.9527   ;
	personalTripProbability  [     405     ] =   0.9533   ;
	personalTripProbability  [     406     ] =   0.9539   ;
	personalTripProbability  [     407     ] =   0.9545   ;
	personalTripProbability  [     408     ] =   0.9551   ;
	personalTripProbability  [     409     ] =   0.9557   ;
	personalTripProbability  [     410     ] =   0.9563   ;
	personalTripProbability  [     411     ] =   0.9569   ;
	personalTripProbability  [     412     ] =   0.9575   ;
	personalTripProbability  [     413     ] =   0.9581   ;
	personalTripProbability  [     414     ] =   0.9587   ;
	personalTripProbability  [     415     ] =   0.9593   ;
	personalTripProbability  [     416     ] =   0.9599   ;
	personalTripProbability  [     417     ] =   0.9605   ;
	personalTripProbability  [     418     ] =   0.9611   ;
	personalTripProbability  [     419     ] =   0.9617   ;
	personalTripProbability  [     420     ] =   0.9623   ;
	personalTripProbability  [     421     ] =   0.9629   ;
	personalTripProbability  [     422     ] =   0.9635   ;
	personalTripProbability  [     423     ] =   0.9640   ;
	personalTripProbability  [     424     ] =   0.9646   ;
	personalTripProbability  [     425     ] =   0.9652   ;
	personalTripProbability  [     426     ] =   0.9658   ;
	personalTripProbability  [     427     ] =   0.9664   ;
	personalTripProbability  [     428     ] =   0.9670   ;
	personalTripProbability  [     429     ] =   0.9676   ;
	personalTripProbability  [     430     ] =   0.9682   ;
	personalTripProbability  [     431     ] =   0.9688   ;
	personalTripProbability  [     432     ] =   0.9694   ;
	personalTripProbability  [     433     ] =   0.9700   ;
	personalTripProbability  [     434     ] =   0.9706   ;
	personalTripProbability  [     435     ] =   0.9712   ;
	personalTripProbability  [     436     ] =   0.9718   ;
	personalTripProbability  [     437     ] =   0.9724   ;
	personalTripProbability  [     438     ] =   0.9730   ;
	personalTripProbability  [     439     ] =   0.9736   ;
	personalTripProbability  [     440     ] =   0.9742   ;
	personalTripProbability  [     441     ] =   0.9748   ;
	personalTripProbability  [     442     ] =   0.9754   ;
	personalTripProbability  [     443     ] =   0.9760   ;
	personalTripProbability  [     444     ] =   0.9766   ;
	personalTripProbability  [     445     ] =   0.9772   ;
	personalTripProbability  [     446     ] =   0.9778   ;
	personalTripProbability  [     447     ] =   0.9784   ;
	personalTripProbability  [     448     ] =   0.9790   ;
	personalTripProbability  [     449     ] =   0.9796   ;
	personalTripProbability  [     450     ] =   0.9802   ;
	personalTripProbability  [     451     ] =   0.9808   ;
	personalTripProbability  [     452     ] =   0.9814   ;
	personalTripProbability  [     453     ] =   0.9820   ;
	personalTripProbability  [     454     ] =   0.9826   ;
	personalTripProbability  [     455     ] =   0.9832   ;
	personalTripProbability  [     456     ] =   0.9838   ;
	personalTripProbability  [     457     ] =   0.9844   ;
	personalTripProbability  [     458     ] =   0.9850   ;
	personalTripProbability  [     459     ] =   0.9856   ;
	personalTripProbability  [     460     ] =   0.9862   ;
	personalTripProbability  [     461     ] =   0.9868   ;
	personalTripProbability  [     462     ] =   0.9874   ;
	personalTripProbability  [     463     ] =   0.9880   ;
	personalTripProbability  [     464     ] =   0.9886   ;
	personalTripProbability  [     465     ] =   0.9892   ;
	personalTripProbability  [     466     ] =   0.9898   ;
	personalTripProbability  [     467     ] =   0.9904   ;
	personalTripProbability  [     468     ] =   0.9910   ;
	personalTripProbability  [     469     ] =   0.9916   ;
	personalTripProbability  [     470     ] =   0.9922   ;
	personalTripProbability  [     471     ] =   0.9928   ;
	personalTripProbability  [     472     ] =   0.9934   ;
	personalTripProbability  [     473     ] =   0.9940   ;
	personalTripProbability  [     474     ] =   0.9946   ;
	personalTripProbability  [     475     ] =   0.9952   ;
	personalTripProbability  [     476     ] =   0.9958   ;
	personalTripProbability  [     477     ] =   0.9964   ;
	personalTripProbability  [     478     ] =   0.9970   ;
	personalTripProbability  [     479     ] =   0.9976   ;
	personalTripProbability  [     480     ] =   0.9982   ;
	personalTripProbability  [     481     ] =   0.9988   ;
	personalTripProbability  [     482     ] =   0.9994   ;
	personalTripProbability  [     483     ] =   1.0000   ;
}

// distribution of dwell times before returning
void Simulator::setDwTimes (double dwLookup [][288])
{
    dwLookup	[	0	]	[	0	]	=	0.167582418	;
    dwLookup	[	0	]	[	1	]	=	0.370879121	;
    dwLookup	[	0	]	[	2	]	=	0.510989011	;
    dwLookup	[	0	]	[	3	]	=	0.585164835	;
    dwLookup	[	0	]	[	4	]	=	0.648351648	;
    dwLookup	[	0	]	[	5	]	=	0.673076923	;
    dwLookup	[	0	]	[	6	]	=	0.711538462	;
    dwLookup	[	0	]	[	7	]	=	0.730769231	;
    dwLookup	[	0	]	[	8	]	=	0.788461538	;
    dwLookup	[	0	]	[	9	]	=	0.821428571	;
    dwLookup	[	0	]	[	10	]	=	0.826923077	;
    dwLookup	[	0	]	[	11	]	=	0.837912088	;
    dwLookup	[	0	]	[	12	]	=	0.843406593	;
    dwLookup	[	0	]	[	13	]	=	0.854395604	;
    dwLookup	[	0	]	[	14	]	=	0.873626374	;
    dwLookup	[	0	]	[	15	]	=	0.89010989	;
    dwLookup	[	0	]	[	16	]	=	0.901098901	;
    dwLookup	[	0	]	[	17	]	=	0.906593407	;
    dwLookup	[	0	]	[	18	]	=	0.914835165	;
    dwLookup	[	0	]	[	19	]	=	0.914835165	;
    dwLookup	[	0	]	[	20	]	=	0.92032967	;
    dwLookup	[	0	]	[	21	]	=	0.936813187	;
    dwLookup	[	0	]	[	22	]	=	0.942307692	;
    dwLookup	[	0	]	[	23	]	=	0.947802198	;
    dwLookup	[	0	]	[	24	]	=	0.964285714	;
    dwLookup	[	0	]	[	25	]	=	0.967032967	;
    dwLookup	[	0	]	[	26	]	=	0.972527473	;
    dwLookup	[	0	]	[	27	]	=	0.980769231	;
    dwLookup	[	0	]	[	28	]	=	0.980769231	;
    dwLookup	[	0	]	[	29	]	=	0.980769231	;
    dwLookup	[	0	]	[	30	]	=	0.986263736	;
    dwLookup	[	0	]	[	31	]	=	0.986263736	;
    dwLookup	[	0	]	[	32	]	=	0.989010989	;
    dwLookup	[	0	]	[	33	]	=	0.991758242	;
    dwLookup	[	0	]	[	34	]	=	0.994505495	;

    for (int i = 35; i < 288; i++)
    {
        dwLookup	[	0	]	[	i	]	=	1	;
    }

    dwLookup	[	1	]	[	0	]	=	0.046613366	;
    dwLookup	[	1	]	[	1	]	=	0.10423895	;
    dwLookup	[	1	]	[	2	]	=	0.131844924	;
    dwLookup	[	1	]	[	3	]	=	0.151908282	;
    dwLookup	[	1	]	[	4	]	=	0.162618796	;
    dwLookup	[	1	]	[	5	]	=	0.16744607	;
    dwLookup	[	1	]	[	6	]	=	0.180570222	;
    dwLookup	[	1	]	[	7	]	=	0.188565394	;
    dwLookup	[	1	]	[	8	]	=	0.197465681	;
    dwLookup	[	1	]	[	9	]	=	0.209835571	;
    dwLookup	[	1	]	[	10	]	=	0.22084779	;
    dwLookup	[	1	]	[	11	]	=	0.233066828	;
    dwLookup	[	1	]	[	12	]	=	0.249057173	;
    dwLookup	[	1	]	[	13	]	=	0.258862574	;
    dwLookup	[	1	]	[	14	]	=	0.27032735	;
    dwLookup	[	1	]	[	15	]	=	0.281942978	;
    dwLookup	[	1	]	[	16	]	=	0.292351788	;
    dwLookup	[	1	]	[	17	]	=	0.300196108	;
    dwLookup	[	1	]	[	18	]	=	0.310152361	;
    dwLookup	[	1	]	[	19	]	=	0.315130487	;
    dwLookup	[	1	]	[	20	]	=	0.321466285	;
    dwLookup	[	1	]	[	21	]	=	0.328405491	;
    dwLookup	[	1	]	[	22	]	=	0.333383617	;
    dwLookup	[	1	]	[	23	]	=	0.337758335	;
    dwLookup	[	1	]	[	24	]	=	0.345602655	;
    dwLookup	[	1	]	[	25	]	=	0.349524815	;
    dwLookup	[	1	]	[	26	]	=	0.352994418	;
    dwLookup	[	1	]	[	27	]	=	0.358425102	;
    dwLookup	[	1	]	[	28	]	=	0.362045557	;
    dwLookup	[	1	]	[	29	]	=	0.36415749	;
    dwLookup	[	1	]	[	30	]	=	0.366420275	;
    dwLookup	[	1	]	[	31	]	=	0.367777945	;
    dwLookup	[	1	]	[	32	]	=	0.370191582	;
    dwLookup	[	1	]	[	33	]	=	0.373510333	;
    dwLookup	[	1	]	[	34	]	=	0.377130789	;
    dwLookup	[	1	]	[	35	]	=	0.379242721	;
    dwLookup	[	1	]	[	36	]	=	0.384824257	;
    dwLookup	[	1	]	[	37	]	=	0.38829386	;
    dwLookup	[	1	]	[	38	]	=	0.39025494	;
    dwLookup	[	1	]	[	39	]	=	0.393422839	;
    dwLookup	[	1	]	[	40	]	=	0.396590738	;
    dwLookup	[	1	]	[	41	]	=	0.399004375	;
    dwLookup	[	1	]	[	42	]	=	0.402775683	;
    dwLookup	[	1	]	[	43	]	=	0.40518932	;
    dwLookup	[	1	]	[	44	]	=	0.407452104	;
    dwLookup	[	1	]	[	45	]	=	0.409865741	;
    dwLookup	[	1	]	[	46	]	=	0.411826822	;
    dwLookup	[	1	]	[	47	]	=	0.414693016	;
    dwLookup	[	1	]	[	48	]	=	0.418766028	;
    dwLookup	[	1	]	[	49	]	=	0.421028813	;
    dwLookup	[	1	]	[	50	]	=	0.424347564	;
    dwLookup	[	1	]	[	51	]	=	0.428873133	;
    dwLookup	[	1	]	[	52	]	=	0.431135918	;
    dwLookup	[	1	]	[	53	]	=	0.43385126	;
    dwLookup	[	1	]	[	54	]	=	0.437471715	;
    dwLookup	[	1	]	[	55	]	=	0.440036205	;
    dwLookup	[	1	]	[	56	]	=	0.442600694	;
    dwLookup	[	1	]	[	57	]	=	0.445919445	;
    dwLookup	[	1	]	[	58	]	=	0.44757882	;
    dwLookup	[	1	]	[	59	]	=	0.45014331	;
    dwLookup	[	1	]	[	60	]	=	0.45602655	;
    dwLookup	[	1	]	[	61	]	=	0.458289335	;
    dwLookup	[	1	]	[	62	]	=	0.462060643	;
    dwLookup	[	1	]	[	63	]	=	0.464775984	;
    dwLookup	[	1	]	[	64	]	=	0.467793031	;
    dwLookup	[	1	]	[	65	]	=	0.47035752	;
    dwLookup	[	1	]	[	66	]	=	0.473676271	;
    dwLookup	[	1	]	[	67	]	=	0.476089908	;
    dwLookup	[	1	]	[	68	]	=	0.478654397	;
    dwLookup	[	1	]	[	69	]	=	0.482274853	;
    dwLookup	[	1	]	[	70	]	=	0.48468849	;
    dwLookup	[	1	]	[	71	]	=	0.487252979	;
    dwLookup	[	1	]	[	72	]	=	0.492381958	;
    dwLookup	[	1	]	[	73	]	=	0.496454971	;
    dwLookup	[	1	]	[	74	]	=	0.499924574	;
    dwLookup	[	1	]	[	75	]	=	0.502639916	;
    dwLookup	[	1	]	[	76	]	=	0.504299291	;
    dwLookup	[	1	]	[	77	]	=	0.506562076	;
    dwLookup	[	1	]	[	78	]	=	0.512143611	;
    dwLookup	[	1	]	[	79	]	=	0.514858953	;
    dwLookup	[	1	]	[	80	]	=	0.51727259	;
    dwLookup	[	1	]	[	81	]	=	0.519686227	;
    dwLookup	[	1	]	[	82	]	=	0.522250717	;
    dwLookup	[	1	]	[	83	]	=	0.523457535	;
    dwLookup	[	1	]	[	84	]	=	0.527832252	;
    dwLookup	[	1	]	[	85	]	=	0.52964248	;
    dwLookup	[	1	]	[	86	]	=	0.531452708	;
    dwLookup	[	1	]	[	87	]	=	0.533715493	;
    dwLookup	[	1	]	[	88	]	=	0.535073163	;
    dwLookup	[	1	]	[	89	]	=	0.536883391	;
    dwLookup	[	1	]	[	90	]	=	0.540956404	;
    dwLookup	[	1	]	[	91	]	=	0.543671745	;
    dwLookup	[	1	]	[	92	]	=	0.546085382	;
    dwLookup	[	1	]	[	93	]	=	0.548197315	;
    dwLookup	[	1	]	[	94	]	=	0.551214361	;
    dwLookup	[	1	]	[	95	]	=	0.553024589	;
    dwLookup	[	1	]	[	96	]	=	0.560868909	;
    dwLookup	[	1	]	[	97	]	=	0.566299593	;
    dwLookup	[	1	]	[	98	]	=	0.573691356	;
    dwLookup	[	1	]	[	99	]	=	0.581233972	;
    dwLookup	[	1	]	[	100	]	=	0.587720622	;
    dwLookup	[	1	]	[	101	]	=	0.5928496	;
    dwLookup	[	1	]	[	102	]	=	0.608689093	;
    dwLookup	[	1	]	[	103	]	=	0.622567506	;
    dwLookup	[	1	]	[	104	]	=	0.63780359	;
    dwLookup	[	1	]	[	105	]	=	0.65862121	;
    dwLookup	[	1	]	[	106	]	=	0.671896214	;
    dwLookup	[	1	]	[	107	]	=	0.682154171	;
    dwLookup	[	1	]	[	108	]	=	0.698898778	;
    dwLookup	[	1	]	[	109	]	=	0.708855031	;
    dwLookup	[	1	]	[	110	]	=	0.720018102	;
    dwLookup	[	1	]	[	111	]	=	0.730125207	;
    dwLookup	[	1	]	[	112	]	=	0.737969528	;
    dwLookup	[	1	]	[	113	]	=	0.74400362	;
    dwLookup	[	1	]	[	114	]	=	0.753809021	;
    dwLookup	[	1	]	[	115	]	=	0.761049932	;
    dwLookup	[	1	]	[	116	]	=	0.771157037	;
    dwLookup	[	1	]	[	117	]	=	0.78111329	;
    dwLookup	[	1	]	[	118	]	=	0.787449087	;
    dwLookup	[	1	]	[	119	]	=	0.792578066	;
    dwLookup	[	1	]	[	120	]	=	0.805702218	;
    dwLookup	[	1	]	[	121	]	=	0.811434606	;
    dwLookup	[	1	]	[	122	]	=	0.818072107	;
    dwLookup	[	1	]	[	123	]	=	0.827123246	;
    dwLookup	[	1	]	[	124	]	=	0.833308191	;
    dwLookup	[	1	]	[	125	]	=	0.836777795	;
    dwLookup	[	1	]	[	126	]	=	0.847488309	;
    dwLookup	[	1	]	[	127	]	=	0.853824106	;
    dwLookup	[	1	]	[	128	]	=	0.862422688	;
    dwLookup	[	1	]	[	129	]	=	0.873133203	;
    dwLookup	[	1	]	[	130	]	=	0.878563886	;
    dwLookup	[	1	]	[	131	]	=	0.882636898	;
    dwLookup	[	1	]	[	132	]	=	0.891386333	;
    dwLookup	[	1	]	[	133	]	=	0.896062755	;
    dwLookup	[	1	]	[	134	]	=	0.900437472	;
    dwLookup	[	1	]	[	135	]	=	0.905113893	;
    dwLookup	[	1	]	[	136	]	=	0.909488611	;
    dwLookup	[	1	]	[	137	]	=	0.912807362	;
    dwLookup	[	1	]	[	138	]	=	0.918992307	;
    dwLookup	[	1	]	[	139	]	=	0.921405944	;
    dwLookup	[	1	]	[	140	]	=	0.924573842	;
    dwLookup	[	1	]	[	141	]	=	0.930004526	;
    dwLookup	[	1	]	[	142	]	=	0.933021572	;
    dwLookup	[	1	]	[	143	]	=	0.935284357	;
    dwLookup	[	1	]	[	144	]	=	0.940413335	;
    dwLookup	[	1	]	[	145	]	=	0.942977825	;
    dwLookup	[	1	]	[	146	]	=	0.946145723	;
    dwLookup	[	1	]	[	147	]	=	0.950972997	;
    dwLookup	[	1	]	[	148	]	=	0.954140896	;
    dwLookup	[	1	]	[	149	]	=	0.956403681	;
    dwLookup	[	1	]	[	150	]	=	0.96153266	;
    dwLookup	[	1	]	[	151	]	=	0.963795444	;
    dwLookup	[	1	]	[	152	]	=	0.966510786	;
    dwLookup	[	1	]	[	153	]	=	0.969075275	;
    dwLookup	[	1	]	[	154	]	=	0.970885503	;
    dwLookup	[	1	]	[	155	]	=	0.971790617	;
    dwLookup	[	1	]	[	156	]	=	0.974958516	;
    dwLookup	[	1	]	[	157	]	=	0.976467039	;
    dwLookup	[	1	]	[	158	]	=	0.977673857	;
    dwLookup	[	1	]	[	159	]	=	0.980087494	;
    dwLookup	[	1	]	[	160	]	=	0.981596017	;
    dwLookup	[	1	]	[	161	]	=	0.981897722	;
    dwLookup	[	1	]	[	162	]	=	0.983557098	;
    dwLookup	[	1	]	[	163	]	=	0.984462211	;
    dwLookup	[	1	]	[	164	]	=	0.985065621	;
    dwLookup	[	1	]	[	165	]	=	0.985970735	;
    dwLookup	[	1	]	[	166	]	=	0.986875849	;
    dwLookup	[	1	]	[	167	]	=	0.987328405	;
    dwLookup	[	1	]	[	168	]	=	0.98763011	;
    dwLookup	[	1	]	[	169	]	=	0.988233519	;
    dwLookup	[	1	]	[	170	]	=	0.988987781	;
    dwLookup	[	1	]	[	171	]	=	0.989892895	;
    dwLookup	[	1	]	[	172	]	=	0.990043747	;
    dwLookup	[	1	]	[	173	]	=	0.990194599	;
    dwLookup	[	1	]	[	174	]	=	0.990948861	;
    dwLookup	[	1	]	[	175	]	=	0.991250566	;
    dwLookup	[	1	]	[	176	]	=	0.99155227	;
    dwLookup	[	1	]	[	177	]	=	0.992306532	;
    dwLookup	[	1	]	[	178	]	=	0.992457384	;
    dwLookup	[	1	]	[	179	]	=	0.992759089	;
    dwLookup	[	1	]	[	180	]	=	0.993965907	;
    dwLookup	[	1	]	[	181	]	=	0.99411676	;
    dwLookup	[	1	]	[	182	]	=	0.994569317	;
    dwLookup	[	1	]	[	183	]	=	0.994720169	;
    dwLookup	[	1	]	[	184	]	=	0.995021874	;

    for (int i = 185; i < 288; i++)
    {
        dwLookup	[	1	]	[	i	]	=	1	;
    }

    dwLookup	[	2	]	[	0	]	=	0.070938424	;
    dwLookup	[	2	]	[	1	]	=	0.143035774	;
    dwLookup	[	2	]	[	2	]	=	0.182398569	;
    dwLookup	[	2	]	[	3	]	=	0.208962355	;
    dwLookup	[	2	]	[	4	]	=	0.227210345	;
    dwLookup	[	2	]	[	5	]	=	0.242479693	;
    dwLookup	[	2	]	[	6	]	=	0.262303415	;
    dwLookup	[	2	]	[	7	]	=	0.275143087	;
    dwLookup	[	2	]	[	8	]	=	0.289792308	;
    dwLookup	[	2	]	[	9	]	=	0.305671618	;
    dwLookup	[	2	]	[	10	]	=	0.318612949	;
    dwLookup	[	2	]	[	11	]	=	0.3302327	;
    dwLookup	[	2	]	[	12	]	=	0.349171978	;
    dwLookup	[	2	]	[	13	]	=	0.360466417	;
    dwLookup	[	2	]	[	14	]	=	0.372025171	;
    dwLookup	[	2	]	[	15	]	=	0.385017333	;
    dwLookup	[	2	]	[	16	]	=	0.39529517	;
    dwLookup	[	2	]	[	17	]	=	0.403285655	;
    dwLookup	[	2	]	[	18	]	=	0.415169722	;
    dwLookup	[	2	]	[	19	]	=	0.421899621	;
    dwLookup	[	2	]	[	20	]	=	0.428690516	;
    dwLookup	[	2	]	[	21	]	=	0.436304858	;
    dwLookup	[	2	]	[	22	]	=	0.442099485	;
    dwLookup	[	2	]	[	23	]	=	0.44708083	;
    dwLookup	[	2	]	[	24	]	=	0.455366129	;
    dwLookup	[	2	]	[	25	]	=	0.460011996	;
    dwLookup	[	2	]	[	26	]	=	0.464932345	;
    dwLookup	[	2	]	[	27	]	=	0.470970956	;
    dwLookup	[	2	]	[	28	]	=	0.475596491	;
    dwLookup	[	2	]	[	29	]	=	0.479368081	;
    dwLookup	[	2	]	[	30	]	=	0.485671008	;
    dwLookup	[	2	]	[	31	]	=	0.489534092	;
    dwLookup	[	2	]	[	32	]	=	0.494017303	;
    dwLookup	[	2	]	[	33	]	=	0.499252798	;
    dwLookup	[	2	]	[	34	]	=	0.503481859	;
    dwLookup	[	2	]	[	35	]	=	0.507314445	;
    dwLookup	[	2	]	[	36	]	=	0.514064676	;
    dwLookup	[	2	]	[	37	]	=	0.51757195	;
    dwLookup	[	2	]	[	38	]	=	0.522105991	;
    dwLookup	[	2	]	[	39	]	=	0.527758293	;
    dwLookup	[	2	]	[	40	]	=	0.5323025	;
    dwLookup	[	2	]	[	41	]	=	0.53632824	;
    dwLookup	[	2	]	[	42	]	=	0.543352954	;
    dwLookup	[	2	]	[	43	]	=	0.547663342	;
    dwLookup	[	2	]	[	44	]	=	0.552319375	;
    dwLookup	[	2	]	[	45	]	=	0.558215662	;
    dwLookup	[	2	]	[	46	]	=	0.563288501	;
    dwLookup	[	2	]	[	47	]	=	0.567934368	;
    dwLookup	[	2	]	[	48	]	=	0.576880458	;
    dwLookup	[	2	]	[	49	]	=	0.582644586	;
    dwLookup	[	2	]	[	50	]	=	0.588195228	;
    dwLookup	[	2	]	[	51	]	=	0.594020352	;
    dwLookup	[	2	]	[	52	]	=	0.598686551	;
    dwLookup	[	2	]	[	53	]	=	0.602661462	;
    dwLookup	[	2	]	[	54	]	=	0.609137211	;
    dwLookup	[	2	]	[	55	]	=	0.612847805	;
    dwLookup	[	2	]	[	56	]	=	0.617036201	;
    dwLookup	[	2	]	[	57	]	=	0.621732898	;
    dwLookup	[	2	]	[	58	]	=	0.625565484	;
    dwLookup	[	2	]	[	59	]	=	0.62853396	;
    dwLookup	[	2	]	[	60	]	=	0.634592902	;
    dwLookup	[	2	]	[	61	]	=	0.638232334	;
    dwLookup	[	2	]	[	62	]	=	0.641698944	;
    dwLookup	[	2	]	[	63	]	=	0.645368874	;
    dwLookup	[	2	]	[	64	]	=	0.648276353	;
    dwLookup	[	2	]	[	65	]	=	0.650838188	;
    dwLookup	[	2	]	[	66	]	=	0.65483343	;
    dwLookup	[	2	]	[	67	]	=	0.657252941	;
    dwLookup	[	2	]	[	68	]	=	0.659896103	;
    dwLookup	[	2	]	[	69	]	=	0.662722254	;
    dwLookup	[	2	]	[	70	]	=	0.665314587	;
    dwLookup	[	2	]	[	71	]	=	0.667469782	;
    dwLookup	[	2	]	[	72	]	=	0.671658178	;
    dwLookup	[	2	]	[	73	]	=	0.674342005	;
    dwLookup	[	2	]	[	74	]	=	0.677473136	;
    dwLookup	[	2	]	[	75	]	=	0.680614434	;
    dwLookup	[	2	]	[	76	]	=	0.683633739	;
    dwLookup	[	2	]	[	77	]	=	0.686541218	;
    dwLookup	[	2	]	[	78	]	=	0.691725884	;
    dwLookup	[	2	]	[	79	]	=	0.695070501	;
    dwLookup	[	2	]	[	80	]	=	0.69963504	;
    dwLookup	[	2	]	[	81	]	=	0.70501286	;
    dwLookup	[	2	]	[	82	]	=	0.71039068	;
    dwLookup	[	2	]	[	83	]	=	0.715311029	;
    dwLookup	[	2	]	[	84	]	=	0.723992802	;
    dwLookup	[	2	]	[	85	]	=	0.730214401	;
    dwLookup	[	2	]	[	86	]	=	0.737127289	;
    dwLookup	[	2	]	[	87	]	=	0.744812793	;
    dwLookup	[	2	]	[	88	]	=	0.751766344	;
    dwLookup	[	2	]	[	89	]	=	0.757815121	;
    dwLookup	[	2	]	[	90	]	=	0.765724277	;
    dwLookup	[	2	]	[	91	]	=	0.771518904	;
    dwLookup	[	2	]	[	92	]	=	0.777120376	;
    dwLookup	[	2	]	[	93	]	=	0.783545295	;
    dwLookup	[	2	]	[	94	]	=	0.788689296	;
    dwLookup	[	2	]	[	95	]	=	0.793568981	;
    dwLookup	[	2	]	[	96	]	=	0.802779387	;
    dwLookup	[	2	]	[	97	]	=	0.808777334	;
    dwLookup	[	2	]	[	98	]	=	0.814988767	;
    dwLookup	[	2	]	[	99	]	=	0.82211514	;
    dwLookup	[	2	]	[	100	]	=	0.82764545	;
    dwLookup	[	2	]	[	101	]	=	0.832982606	;
    dwLookup	[	2	]	[	102	]	=	0.844104222	;
    dwLookup	[	2	]	[	103	]	=	0.852003212	;
    dwLookup	[	2	]	[	104	]	=	0.86108146	;
    dwLookup	[	2	]	[	105	]	=	0.870993321	;
    dwLookup	[	2	]	[	106	]	=	0.878272185	;
    dwLookup	[	2	]	[	107	]	=	0.884036313	;
    dwLookup	[	2	]	[	108	]	=	0.89602204	;
    dwLookup	[	2	]	[	109	]	=	0.902843433	;
    dwLookup	[	2	]	[	110	]	=	0.910122297	;
    dwLookup	[	2	]	[	111	]	=	0.917563817	;
    dwLookup	[	2	]	[	112	]	=	0.922616325	;
    dwLookup	[	2	]	[	113	]	=	0.927221528	;
    dwLookup	[	2	]	[	114	]	=	0.934419063	;
    dwLookup	[	2	]	[	115	]	=	0.938831112	;
    dwLookup	[	2	]	[	116	]	=	0.943527809	;
    dwLookup	[	2	]	[	117	]	=	0.94806185	;
    dwLookup	[	2	]	[	118	]	=	0.951172649	;
    dwLookup	[	2	]	[	119	]	=	0.953805646	;
    dwLookup	[	2	]	[	120	]	=	0.958776826	;
    dwLookup	[	2	]	[	121	]	=	0.96138949	;
    dwLookup	[	2	]	[	122	]	=	0.963930993	;
    dwLookup	[	2	]	[	123	]	=	0.96697063	;
    dwLookup	[	2	]	[	124	]	=	0.968739516	;
    dwLookup	[	2	]	[	125	]	=	0.970345746	;
    dwLookup	[	2	]	[	126	]	=	0.973243059	;
    dwLookup	[	2	]	[	127	]	=	0.974767961	;
    dwLookup	[	2	]	[	128	]	=	0.976547013	;
    dwLookup	[	2	]	[	129	]	=	0.978519219	;
    dwLookup	[	2	]	[	130	]	=	0.97989163	;
    dwLookup	[	2	]	[	131	]	=	0.981081054	;
    dwLookup	[	2	]	[	132	]	=	0.982778777	;
    dwLookup	[	2	]	[	133	]	=	0.983541228	;
    dwLookup	[	2	]	[	134	]	=	0.984476501	;
    dwLookup	[	2	]	[	135	]	=	0.985625261	;
    dwLookup	[	2	]	[	136	]	=	0.986245387	;
    dwLookup	[	2	]	[	137	]	=	0.986753688	;
    dwLookup	[	2	]	[	138	]	=	0.987485641	;
    dwLookup	[	2	]	[	139	]	=	0.988065103	;
    dwLookup	[	2	]	[	140	]	=	0.988603902	;
    dwLookup	[	2	]	[	141	]	=	0.9891427	;
    dwLookup	[	2	]	[	142	]	=	0.989508677	;
    dwLookup	[	2	]	[	143	]	=	0.989833989	;
    dwLookup	[	2	]	[	144	]	=	0.99049478	;
    dwLookup	[	2	]	[	145	]	=	0.990820092	;
    dwLookup	[	2	]	[	146	]	=	0.991297895	;
    dwLookup	[	2	]	[	147	]	=	0.991836693	;
    dwLookup	[	2	]	[	148	]	=	0.992324662	;
    dwLookup	[	2	]	[	149	]	=	0.99266014	;
    dwLookup	[	2	]	[	150	]	=	0.993270101	;
    dwLookup	[	2	]	[	151	]	=	0.993595413	;
    dwLookup	[	2	]	[	152	]	=	0.993941057	;
    dwLookup	[	2	]	[	153	]	=	0.99446969	;
    dwLookup	[	2	]	[	154	]	=	0.994835666	;
    dwLookup	[	2	]	[	155	]	=	0.995160979	;

    for (int i = 156; i < 288; i++)
    {
        dwLookup	[	2	]	[	i	]	=	1	;
    }

    dwLookup	[	3	]	[	0	]	=	0.056012487	;
    dwLookup	[	3	]	[	1	]	=	0.139368834	;
    dwLookup	[	3	]	[	2	]	=	0.197954814	;
    dwLookup	[	3	]	[	3	]	=	0.245102159	;
    dwLookup	[	3	]	[	4	]	=	0.280502692	;
    dwLookup	[	3	]	[	5	]	=	0.307698465	;
    dwLookup	[	3	]	[	6	]	=	0.344311701	;
    dwLookup	[	3	]	[	7	]	=	0.366736707	;
    dwLookup	[	3	]	[	8	]	=	0.390590543	;
    dwLookup	[	3	]	[	9	]	=	0.418066479	;
    dwLookup	[	3	]	[	10	]	=	0.439010626	;
    dwLookup	[	3	]	[	11	]	=	0.456748914	;
    dwLookup	[	3	]	[	12	]	=	0.484645094	;
    dwLookup	[	3	]	[	13	]	=	0.50041424	;
    dwLookup	[	3	]	[	14	]	=	0.516871786	;
    dwLookup	[	3	]	[	15	]	=	0.535334494	;
    dwLookup	[	3	]	[	16	]	=	0.549350623	;
    dwLookup	[	3	]	[	17	]	=	0.560677193	;
    dwLookup	[	3	]	[	18	]	=	0.576342279	;
    dwLookup	[	3	]	[	19	]	=	0.585759741	;
    dwLookup	[	3	]	[	20	]	=	0.595365312	;
    dwLookup	[	3	]	[	21	]	=	0.606479758	;
    dwLookup	[	3	]	[	22	]	=	0.614452382	;
    dwLookup	[	3	]	[	23	]	=	0.621316363	;
    dwLookup	[	3	]	[	24	]	=	0.632422805	;
    dwLookup	[	3	]	[	25	]	=	0.63820616	;
    dwLookup	[	3	]	[	26	]	=	0.644593864	;
    dwLookup	[	3	]	[	27	]	=	0.652762602	;
    dwLookup	[	3	]	[	28	]	=	0.65886214	;
    dwLookup	[	3	]	[	29	]	=	0.664145204	;
    dwLookup	[	3	]	[	30	]	=	0.672065798	;
    dwLookup	[	3	]	[	31	]	=	0.676836565	;
    dwLookup	[	3	]	[	32	]	=	0.682367773	;
    dwLookup	[	3	]	[	33	]	=	0.68894759	;
    dwLookup	[	3	]	[	34	]	=	0.694066559	;
    dwLookup	[	3	]	[	35	]	=	0.698473114	;
    dwLookup	[	3	]	[	36	]	=	0.705625263	;
    dwLookup	[	3	]	[	37	]	=	0.709439475	;
    dwLookup	[	3	]	[	38	]	=	0.713882052	;
    dwLookup	[	3	]	[	39	]	=	0.719373236	;
    dwLookup	[	3	]	[	40	]	=	0.723439595	;
    dwLookup	[	3	]	[	41	]	=	0.727125733	;
    dwLookup	[	3	]	[	42	]	=	0.732937104	;
    dwLookup	[	3	]	[	43	]	=	0.736451142	;
    dwLookup	[	3	]	[	44	]	=	0.740373417	;
    dwLookup	[	3	]	[	45	]	=	0.745148186	;
    dwLookup	[	3	]	[	46	]	=	0.749194533	;
    dwLookup	[	3	]	[	47	]	=	0.752708571	;
    dwLookup	[	3	]	[	48	]	=	0.75900022	;
    dwLookup	[	3	]	[	49	]	=	0.762846451	;
    dwLookup	[	3	]	[	50	]	=	0.766708691	;
    dwLookup	[	3	]	[	51	]	=	0.771055212	;
    dwLookup	[	3	]	[	52	]	=	0.774593264	;
    dwLookup	[	3	]	[	53	]	=	0.777647035	;
    dwLookup	[	3	]	[	54	]	=	0.782441816	;
    dwLookup	[	3	]	[	55	]	=	0.785047327	;
    dwLookup	[	3	]	[	56	]	=	0.788245182	;
    dwLookup	[	3	]	[	57	]	=	0.79198335	;
    dwLookup	[	3	]	[	58	]	=	0.794909047	;
    dwLookup	[	3	]	[	59	]	=	0.797246403	;
    dwLookup	[	3	]	[	60	]	=	0.801761021	;
    dwLookup	[	3	]	[	61	]	=	0.804318505	;
    dwLookup	[	3	]	[	62	]	=	0.807132137	;
    dwLookup	[	3	]	[	63	]	=	0.810181906	;
    dwLookup	[	3	]	[	64	]	=	0.812627324	;
    dwLookup	[	3	]	[	65	]	=	0.814748554	;
    dwLookup	[	3	]	[	66	]	=	0.81813852	;
    dwLookup	[	3	]	[	67	]	=	0.820055632	;
    dwLookup	[	3	]	[	68	]	=	0.822380981	;
    dwLookup	[	3	]	[	69	]	=	0.82486242	;
    dwLookup	[	3	]	[	70	]	=	0.827027676	;
    dwLookup	[	3	]	[	71	]	=	0.828936783	;
    dwLookup	[	3	]	[	72	]	=	0.832334754	;
    dwLookup	[	3	]	[	73	]	=	0.834319906	;
    dwLookup	[	3	]	[	74	]	=	0.836605231	;
    dwLookup	[	3	]	[	75	]	=	0.839006624	;
    dwLookup	[	3	]	[	76	]	=	0.841243921	;
    dwLookup	[	3	]	[	77	]	=	0.843245082	;
    dwLookup	[	3	]	[	78	]	=	0.846567009	;
    dwLookup	[	3	]	[	79	]	=	0.848644214	;
    dwLookup	[	3	]	[	80	]	=	0.851369794	;
    dwLookup	[	3	]	[	81	]	=	0.854663705	;
    dwLookup	[	3	]	[	82	]	=	0.857565388	;
    dwLookup	[	3	]	[	83	]	=	0.86025895	;
    dwLookup	[	3	]	[	84	]	=	0.864805587	;
    dwLookup	[	3	]	[	85	]	=	0.868011447	;
    dwLookup	[	3	]	[	86	]	=	0.87139741	;
    dwLookup	[	3	]	[	87	]	=	0.875235637	;
    dwLookup	[	3	]	[	88	]	=	0.878573573	;
    dwLookup	[	3	]	[	89	]	=	0.881567309	;
    dwLookup	[	3	]	[	90	]	=	0.885661684	;
    dwLookup	[	3	]	[	91	]	=	0.888507334	;
    dwLookup	[	3	]	[	92	]	=	0.891276941	;
    dwLookup	[	3	]	[	93	]	=	0.894530828	;
    dwLookup	[	3	]	[	94	]	=	0.897196374	;
    dwLookup	[	3	]	[	95	]	=	0.899633788	;
    dwLookup	[	3	]	[	96	]	=	0.904144404	;
    dwLookup	[	3	]	[	97	]	=	0.907042084	;
    dwLookup	[	3	]	[	98	]	=	0.910219928	;
    dwLookup	[	3	]	[	99	]	=	0.913802005	;
    dwLookup	[	3	]	[	100	]	=	0.916631646	;
    dwLookup	[	3	]	[	101	]	=	0.919237158	;
    dwLookup	[	3	]	[	102	]	=	0.924484201	;
    dwLookup	[	3	]	[	103	]	=	0.928278401	;
    dwLookup	[	3	]	[	104	]	=	0.932604911	;
    dwLookup	[	3	]	[	105	]	=	0.93737968	;
    dwLookup	[	3	]	[	106	]	=	0.940877709	;
    dwLookup	[	3	]	[	107	]	=	0.943643313	;
    dwLookup	[	3	]	[	108	]	=	0.949046447	;
    dwLookup	[	3	]	[	109	]	=	0.952212283	;
    dwLookup	[	3	]	[	110	]	=	0.955582238	;
    dwLookup	[	3	]	[	111	]	=	0.959024234	;
    dwLookup	[	3	]	[	112	]	=	0.961389606	;
    dwLookup	[	3	]	[	113	]	=	0.963506834	;
    dwLookup	[	3	]	[	114	]	=	0.966836765	;
    dwLookup	[	3	]	[	115	]	=	0.968893958	;
    dwLookup	[	3	]	[	116	]	=	0.971115247	;
    dwLookup	[	3	]	[	117	]	=	0.973236477	;
    dwLookup	[	3	]	[	118	]	=	0.974753357	;
    dwLookup	[	3	]	[	119	]	=	0.975958056	;
    dwLookup	[	3	]	[	120	]	=	0.978343439	;
    dwLookup	[	3	]	[	121	]	=	0.97956815	;
    dwLookup	[	3	]	[	122	]	=	0.980728823	;
    dwLookup	[	3	]	[	123	]	=	0.982201677	;
    dwLookup	[	3	]	[	124	]	=	0.983082188	;
    dwLookup	[	3	]	[	125	]	=	0.983854636	;
    dwLookup	[	3	]	[	126	]	=	0.985283464	;
    dwLookup	[	3	]	[	127	]	=	0.986027896	;
    dwLookup	[	3	]	[	128	]	=	0.986952432	;
    dwLookup	[	3	]	[	129	]	=	0.988005043	;
    dwLookup	[	3	]	[	130	]	=	0.988669428	;
    dwLookup	[	3	]	[	131	]	=	0.989265774	;
    dwLookup	[	3	]	[	132	]	=	0.99019031	;
    dwLookup	[	3	]	[	133	]	=	0.990610554	;
    dwLookup	[	3	]	[	134	]	=	0.991126854	;
    dwLookup	[	3	]	[	135	]	=	0.991763223	;
    dwLookup	[	3	]	[	136	]	=	0.99210342	;
    dwLookup	[	3	]	[	137	]	=	0.992415601	;
    dwLookup	[	3	]	[	138	]	=	0.992887875	;
    dwLookup	[	3	]	[	139	]	=	0.993184047	;
    dwLookup	[	3	]	[	140	]	=	0.993476216	;
    dwLookup	[	3	]	[	141	]	=	0.99384443	;
    dwLookup	[	3	]	[	142	]	=	0.994084569	;
    dwLookup	[	3	]	[	143	]	=	0.994272678	;
    dwLookup	[	3	]	[	144	]	=	0.994676913	;
    dwLookup	[	3	]	[	145	]	=	0.994885033	;
    dwLookup	[	3	]	[	146	]	=	0.995161193	;

    for (int i = 147; i < 288; i++)
    {
        dwLookup	[	3	]	[	i	]	=	1	;
    }

    dwLookup	[	4	]	[	0	]	=	0.045919781	;
    dwLookup	[	4	]	[	1	]	=	0.134877402	;
    dwLookup	[	4	]	[	2	]	=	0.209959508	;
    dwLookup	[	4	]	[	3	]	=	0.273909545	;
    dwLookup	[	4	]	[	4	]	=	0.324458894	;
    dwLookup	[	4	]	[	5	]	=	0.363348589	;
    dwLookup	[	4	]	[	6	]	=	0.415962119	;
    dwLookup	[	4	]	[	7	]	=	0.447801944	;
    dwLookup	[	4	]	[	8	]	=	0.481198148	;
    dwLookup	[	4	]	[	9	]	=	0.518999697	;
    dwLookup	[	4	]	[	10	]	=	0.546763918	;
    dwLookup	[	4	]	[	11	]	=	0.569680942	;
    dwLookup	[	4	]	[	12	]	=	0.604218051	;
    dwLookup	[	4	]	[	13	]	=	0.621358007	;
    dwLookup	[	4	]	[	14	]	=	0.639322316	;
    dwLookup	[	4	]	[	15	]	=	0.65838796	;
    dwLookup	[	4	]	[	16	]	=	0.672639382	;
    dwLookup	[	4	]	[	17	]	=	0.683857182	;
    dwLookup	[	4	]	[	18	]	=	0.700482741	;
    dwLookup	[	4	]	[	19	]	=	0.709234077	;
    dwLookup	[	4	]	[	20	]	=	0.719548386	;
    dwLookup	[	4	]	[	21	]	=	0.731280584	;
    dwLookup	[	4	]	[	22	]	=	0.740658428	;
    dwLookup	[	4	]	[	23	]	=	0.748288642	;
    dwLookup	[	4	]	[	24	]	=	0.760607779	;
    dwLookup	[	4	]	[	25	]	=	0.767578511	;
    dwLookup	[	4	]	[	26	]	=	0.774878985	;
    dwLookup	[	4	]	[	27	]	=	0.78383476	;
    dwLookup	[	4	]	[	28	]	=	0.791009932	;
    dwLookup	[	4	]	[	29	]	=	0.796747431	;
    dwLookup	[	4	]	[	30	]	=	0.805109672	;
    dwLookup	[	4	]	[	31	]	=	0.81053062	;
    dwLookup	[	4	]	[	32	]	=	0.816822085	;
    dwLookup	[	4	]	[	33	]	=	0.824208291	;
    dwLookup	[	4	]	[	34	]	=	0.830486566	;
    dwLookup	[	4	]	[	35	]	=	0.835769023	;
    dwLookup	[	4	]	[	36	]	=	0.844500574	;
    dwLookup	[	4	]	[	37	]	=	0.849176306	;
    dwLookup	[	4	]	[	38	]	=	0.854788504	;
    dwLookup	[	4	]	[	39	]	=	0.861587771	;
    dwLookup	[	4	]	[	40	]	=	0.866771305	;
    dwLookup	[	4	]	[	41	]	=	0.871400873	;
    dwLookup	[	4	]	[	42	]	=	0.878740916	;
    dwLookup	[	4	]	[	43	]	=	0.883119881	;
    dwLookup	[	4	]	[	44	]	=	0.887610958	;
    dwLookup	[	4	]	[	45	]	=	0.893612251	;
    dwLookup	[	4	]	[	46	]	=	0.898795785	;
    dwLookup	[	4	]	[	47	]	=	0.903464922	;
    dwLookup	[	4	]	[	48	]	=	0.910870913	;
    dwLookup	[	4	]	[	49	]	=	0.914900352	;
    dwLookup	[	4	]	[	50	]	=	0.919463972	;
    dwLookup	[	4	]	[	51	]	=	0.924357334	;
    dwLookup	[	4	]	[	52	]	=	0.928360394	;
    dwLookup	[	4	]	[	53	]	=	0.931941385	;
    dwLookup	[	4	]	[	54	]	=	0.937309574	;
    dwLookup	[	4	]	[	55	]	=	0.940461902	;
    dwLookup	[	4	]	[	56	]	=	0.9437791	;
    dwLookup	[	4	]	[	57	]	=	0.947445823	;
    dwLookup	[	4	]	[	58	]	=	0.950644315	;
    dwLookup	[	4	]	[	59	]	=	0.953150349	;
    dwLookup	[	4	]	[	60	]	=	0.956447762	;
    dwLookup	[	4	]	[	61	]	=	0.958637245	;
    dwLookup	[	4	]	[	62	]	=	0.960793753	;
    dwLookup	[	4	]	[	63	]	=	0.963306382	;
    dwLookup	[	4	]	[	64	]	=	0.965152934	;
    dwLookup	[	4	]	[	65	]	=	0.96682802	;
    dwLookup	[	4	]	[	66	]	=	0.969017503	;
    dwLookup	[	4	]	[	67	]	=	0.970389227	;
    dwLookup	[	4	]	[	68	]	=	0.971952201	;
    dwLookup	[	4	]	[	69	]	=	0.973567933	;
    dwLookup	[	4	]	[	70	]	=	0.974966037	;
    dwLookup	[	4	]	[	71	]	=	0.976139916	;
    dwLookup	[	4	]	[	72	]	=	0.977722674	;
    dwLookup	[	4	]	[	73	]	=	0.978560217	;
    dwLookup	[	4	]	[	74	]	=	0.97942414	;
    dwLookup	[	4	]	[	75	]	=	0.980584829	;
    dwLookup	[	4	]	[	76	]	=	0.981389398	;
    dwLookup	[	4	]	[	77	]	=	0.981996122	;
    dwLookup	[	4	]	[	78	]	=	0.982820476	;
    dwLookup	[	4	]	[	79	]	=	0.98340082	;
    dwLookup	[	4	]	[	80	]	=	0.984146036	;
    dwLookup	[	4	]	[	81	]	=	0.984976984	;
    dwLookup	[	4	]	[	82	]	=	0.985484786	;
    dwLookup	[	4	]	[	83	]	=	0.985992587	;
    dwLookup	[	4	]	[	84	]	=	0.986770777	;
    dwLookup	[	4	]	[	85	]	=	0.987001596	;
    dwLookup	[	4	]	[	86	]	=	0.987476423	;
    dwLookup	[	4	]	[	87	]	=	0.988023794	;
    dwLookup	[	4	]	[	88	]	=	0.988432673	;
    dwLookup	[	4	]	[	89	]	=	0.988775604	;
    dwLookup	[	4	]	[	90	]	=	0.989283406	;
    dwLookup	[	4	]	[	91	]	=	0.989553794	;
    dwLookup	[	4	]	[	92	]	=	0.989929699	;
    dwLookup	[	4	]	[	93	]	=	0.990252846	;
    dwLookup	[	4	]	[	94	]	=	0.990569397	;
    dwLookup	[	4	]	[	95	]	=	0.990839785	;
    dwLookup	[	4	]	[	96	]	=	0.991373966	;
    dwLookup	[	4	]	[	97	]	=	0.991782846	;
    dwLookup	[	4	]	[	98	]	=	0.99219832	;
    dwLookup	[	4	]	[	99	]	=	0.992831423	;
    dwLookup	[	4	]	[	100	]	=	0.993246897	;
    dwLookup	[	4	]	[	101	]	=	0.993504095	;
    dwLookup	[	4	]	[	102	]	=	0.994117414	;
    dwLookup	[	4	]	[	103	]	=	0.994513104	;
    dwLookup	[	4	]	[	104	]	=	0.994928578	;
    dwLookup	[	4	]	[	105	]	=	0.995449569	;

    for (int i = 106; i < 288; i++)
    {
        dwLookup	[	4	]	[	i	]	=	1	;
    }

    dwLookup	[	5	]	[	0	]	=	0.055287854	;
    dwLookup	[	5	]	[	1	]	=	0.153690597	;
    dwLookup	[	5	]	[	2	]	=	0.23749526	;
    dwLookup	[	5	]	[	3	]	=	0.307333797	;
    dwLookup	[	5	]	[	4	]	=	0.360836388	;
    dwLookup	[	5	]	[	5	]	=	0.401091696	;
    dwLookup	[	5	]	[	6	]	=	0.450723584	;
    dwLookup	[	5	]	[	7	]	=	0.480875569	;
    dwLookup	[	5	]	[	8	]	=	0.512923407	;
    dwLookup	[	5	]	[	9	]	=	0.548968339	;
    dwLookup	[	5	]	[	10	]	=	0.576402932	;
    dwLookup	[	5	]	[	11	]	=	0.599279575	;
    dwLookup	[	5	]	[	12	]	=	0.635348205	;
    dwLookup	[	5	]	[	13	]	=	0.655934024	;
    dwLookup	[	5	]	[	14	]	=	0.67942682	;
    dwLookup	[	5	]	[	15	]	=	0.703835945	;
    dwLookup	[	5	]	[	16	]	=	0.722683898	;
    dwLookup	[	5	]	[	17	]	=	0.737447864	;
    dwLookup	[	5	]	[	18	]	=	0.759431876	;
    dwLookup	[	5	]	[	19	]	=	0.771502149	;
    dwLookup	[	5	]	[	20	]	=	0.784781029	;
    dwLookup	[	5	]	[	21	]	=	0.800105852	;
    dwLookup	[	5	]	[	22	]	=	0.812128729	;
    dwLookup	[	5	]	[	23	]	=	0.821773888	;
    dwLookup	[	5	]	[	24	]	=	0.835945399	;
    dwLookup	[	5	]	[	25	]	=	0.843584113	;
    dwLookup	[	5	]	[	26	]	=	0.853087083	;
    dwLookup	[	5	]	[	27	]	=	0.863774962	;
    dwLookup	[	5	]	[	28	]	=	0.871698054	;
    dwLookup	[	5	]	[	29	]	=	0.878088663	;
    dwLookup	[	5	]	[	30	]	=	0.887394148	;
    dwLookup	[	5	]	[	31	]	=	0.89244186	;
    dwLookup	[	5	]	[	32	]	=	0.89869818	;
    dwLookup	[	5	]	[	33	]	=	0.90586293	;
    dwLookup	[	5	]	[	34	]	=	0.911360907	;
    dwLookup	[	5	]	[	35	]	=	0.916132141	;
    dwLookup	[	5	]	[	36	]	=	0.923028311	;
    dwLookup	[	5	]	[	37	]	=	0.926875316	;
    dwLookup	[	5	]	[	38	]	=	0.931140988	;
    dwLookup	[	5	]	[	39	]	=	0.936386186	;
    dwLookup	[	5	]	[	40	]	=	0.93998041	;
    dwLookup	[	5	]	[	41	]	=	0.943021676	;
    dwLookup	[	5	]	[	42	]	=	0.94739794	;
    dwLookup	[	5	]	[	43	]	=	0.949799355	;
    dwLookup	[	5	]	[	44	]	=	0.952706332	;
    dwLookup	[	5	]	[	45	]	=	0.956016178	;
    dwLookup	[	5	]	[	46	]	=	0.958322801	;
    dwLookup	[	5	]	[	47	]	=	0.960273951	;
    dwLookup	[	5	]	[	48	]	=	0.963465306	;
    dwLookup	[	5	]	[	49	]	=	0.965139977	;
    dwLookup	[	5	]	[	50	]	=	0.966941039	;
    dwLookup	[	5	]	[	51	]	=	0.969105473	;
    dwLookup	[	5	]	[	52	]	=	0.97056686	;
    dwLookup	[	5	]	[	53	]	=	0.971854462	;
    dwLookup	[	5	]	[	54	]	=	0.973639724	;
    dwLookup	[	5	]	[	55	]	=	0.974650847	;
    dwLookup	[	5	]	[	56	]	=	0.975922649	;
    dwLookup	[	5	]	[	57	]	=	0.97747883	;
    dwLookup	[	5	]	[	58	]	=	0.978592644	;
    dwLookup	[	5	]	[	59	]	=	0.979445779	;
    dwLookup	[	5	]	[	60	]	=	0.981104651	;
    dwLookup	[	5	]	[	61	]	=	0.981791898	;
    dwLookup	[	5	]	[	62	]	=	0.98267663	;
    dwLookup	[	5	]	[	63	]	=	0.983782546	;
    dwLookup	[	5	]	[	64	]	=	0.984588284	;
    dwLookup	[	5	]	[	65	]	=	0.985267631	;
    dwLookup	[	5	]	[	66	]	=	0.986286653	;
    dwLookup	[	5	]	[	67	]	=	0.987021297	;
    dwLookup	[	5	]	[	68	]	=	0.987795437	;
    dwLookup	[	5	]	[	69	]	=	0.988609075	;
    dwLookup	[	5	]	[	70	]	=	0.989304221	;
    dwLookup	[	5	]	[	71	]	=	0.989770286	;
    dwLookup	[	5	]	[	72	]	=	0.990386438	;
    dwLookup	[	5	]	[	73	]	=	0.990694515	;
    dwLookup	[	5	]	[	74	]	=	0.99127117	;
    dwLookup	[	5	]	[	75	]	=	0.991808329	;
    dwLookup	[	5	]	[	76	]	=	0.992203299	;
    dwLookup	[	5	]	[	77	]	=	0.992440281	;
    dwLookup	[	5	]	[	78	]	=	0.992898445	;
    dwLookup	[	5	]	[	79	]	=	0.993080131	;
    dwLookup	[	5	]	[	80	]	=	0.993380308	;
    dwLookup	[	5	]	[	81	]	=	0.993719982	;
    dwLookup	[	5	]	[	82	]	=	0.994083354	;
    dwLookup	[	5	]	[	83	]	=	0.994359833	;
    dwLookup	[	5	]	[	84	]	=	0.994683708	;
    dwLookup	[	5	]	[	85	]	=	0.994968086	;
    dwLookup	[	5	]	[	86	]	=	0.995133974	;

    for (int i = 87; i < 288; i++)
    {
        dwLookup	[	5	]	[	i	]	=	1	;
    }

    dwLookup	[	6	]	[	0	]	=	0.055269312	;
    dwLookup	[	6	]	[	1	]	=	0.146422568	;
    dwLookup	[	6	]	[	2	]	=	0.219341519	;
    dwLookup	[	6	]	[	3	]	=	0.280658481	;
    dwLookup	[	6	]	[	4	]	=	0.327979975	;
    dwLookup	[	6	]	[	5	]	=	0.36453994	;
    dwLookup	[	6	]	[	6	]	=	0.409705474	;
    dwLookup	[	6	]	[	7	]	=	0.437586787	;
    dwLookup	[	6	]	[	8	]	=	0.46872031	;
    dwLookup	[	6	]	[	9	]	=	0.505554337	;
    dwLookup	[	6	]	[	10	]	=	0.536943653	;
    dwLookup	[	6	]	[	11	]	=	0.563217131	;
    dwLookup	[	6	]	[	12	]	=	0.601348389	;
    dwLookup	[	6	]	[	13	]	=	0.625831324	;
    dwLookup	[	6	]	[	14	]	=	0.64927282	;
    dwLookup	[	6	]	[	15	]	=	0.678761967	;
    dwLookup	[	6	]	[	16	]	=	0.70255061	;
    dwLookup	[	6	]	[	17	]	=	0.720949353	;
    dwLookup	[	6	]	[	18	]	=	0.74439085	;
    dwLookup	[	6	]	[	19	]	=	0.758770007	;
    dwLookup	[	6	]	[	20	]	=	0.775707082	;
    dwLookup	[	6	]	[	21	]	=	0.794836659	;
    dwLookup	[	6	]	[	22	]	=	0.809636045	;
    dwLookup	[	6	]	[	23	]	=	0.822005408	;
    dwLookup	[	6	]	[	24	]	=	0.839180004	;
    dwLookup	[	6	]	[	25	]	=	0.84922897	;
    dwLookup	[	6	]	[	26	]	=	0.861269458	;
    dwLookup	[	6	]	[	27	]	=	0.87360228	;
    dwLookup	[	6	]	[	28	]	=	0.883925309	;
    dwLookup	[	6	]	[	29	]	=	0.892110648	;
    dwLookup	[	6	]	[	30	]	=	0.902488489	;
    dwLookup	[	6	]	[	31	]	=	0.908664036	;
    dwLookup	[	6	]	[	32	]	=	0.915332895	;
    dwLookup	[	6	]	[	33	]	=	0.924230797	;
    dwLookup	[	6	]	[	34	]	=	0.931319886	;
    dwLookup	[	6	]	[	35	]	=	0.936015494	;
    dwLookup	[	6	]	[	36	]	=	0.943104582	;
    dwLookup	[	6	]	[	37	]	=	0.946448147	;
    dwLookup	[	6	]	[	38	]	=	0.950851421	;
    dwLookup	[	6	]	[	39	]	=	0.955547029	;
    dwLookup	[	6	]	[	40	]	=	0.958470365	;
    dwLookup	[	6	]	[	41	]	=	0.961229263	;
    dwLookup	[	6	]	[	42	]	=	0.96548637	;
    dwLookup	[	6	]	[	43	]	=	0.967532705	;
    dwLookup	[	6	]	[	44	]	=	0.970346415	;
    dwLookup	[	6	]	[	45	]	=	0.973196667	;
    dwLookup	[	6	]	[	46	]	=	0.975553607	;
    dwLookup	[	6	]	[	47	]	=	0.977417233	;
    dwLookup	[	6	]	[	48	]	=	0.979628006	;
    dwLookup	[	6	]	[	49	]	=	0.98057809	;
    dwLookup	[	6	]	[	50	]	=	0.98231382	;
    dwLookup	[	6	]	[	51	]	=	0.983775488	;
    dwLookup	[	6	]	[	52	]	=	0.984780384	;
    dwLookup	[	6	]	[	53	]	=	0.985693927	;
    dwLookup	[	6	]	[	54	]	=	0.986918074	;
    dwLookup	[	6	]	[	55	]	=	0.987667178	;
    dwLookup	[	6	]	[	56	]	=	0.988854783	;
    dwLookup	[	6	]	[	57	]	=	0.989914492	;
    dwLookup	[	6	]	[	58	]	=	0.990700139	;
    dwLookup	[	6	]	[	59	]	=	0.991430973	;
    dwLookup	[	6	]	[	60	]	=	0.992143536	;
    dwLookup	[	6	]	[	61	]	=	0.992490682	;
    dwLookup	[	6	]	[	62	]	=	0.992947453	;
    dwLookup	[	6	]	[	63	]	=	0.993641745	;
    dwLookup	[	6	]	[	64	]	=	0.994007162	;
    dwLookup	[	6	]	[	65	]	=	0.994244683	;
    dwLookup	[	6	]	[	66	]	=	0.99482935	;
    dwLookup	[	6	]	[	67	]	=	0.995085142	;

    for (int i = 68; i < 288; i++)
    {
        dwLookup	[	6	]	[	i	]	=	1	;
    }

    dwLookup	[	7	]	[	0	]	=	0.135261517	;
    dwLookup	[	7	]	[	1	]	=	0.294942847	;
    dwLookup	[	7	]	[	2	]	=	0.411326637	;
    dwLookup	[	7	]	[	3	]	=	0.48926221	;
    dwLookup	[	7	]	[	4	]	=	0.549186006	;
    dwLookup	[	7	]	[	5	]	=	0.591790786	;
    dwLookup	[	7	]	[	6	]	=	0.631451334	;
    dwLookup	[	7	]	[	7	]	=	0.656390717	;
    dwLookup	[	7	]	[	8	]	=	0.682715622	;
    dwLookup	[	7	]	[	9	]	=	0.711118808	;
    dwLookup	[	7	]	[	10	]	=	0.731382057	;
    dwLookup	[	7	]	[	11	]	=	0.751298926	;
    dwLookup	[	7	]	[	12	]	=	0.771562175	;
    dwLookup	[	7	]	[	13	]	=	0.785936959	;
    dwLookup	[	7	]	[	14	]	=	0.800484932	;
    dwLookup	[	7	]	[	15	]	=	0.814686526	;
    dwLookup	[	7	]	[	16	]	=	0.824731555	;
    dwLookup	[	7	]	[	17	]	=	0.833737444	;
    dwLookup	[	7	]	[	18	]	=	0.843609283	;
    dwLookup	[	7	]	[	19	]	=	0.851922411	;
    dwLookup	[	7	]	[	20	]	=	0.86196744	;
    dwLookup	[	7	]	[	21	]	=	0.873571181	;
    dwLookup	[	7	]	[	22	]	=	0.880671978	;
    dwLookup	[	7	]	[	23	]	=	0.887945965	;
    dwLookup	[	7	]	[	24	]	=	0.896778663	;
    dwLookup	[	7	]	[	25	]	=	0.902147558	;
    dwLookup	[	7	]	[	26	]	=	0.907170073	;
    dwLookup	[	7	]	[	27	]	=	0.916349151	;
    dwLookup	[	7	]	[	28	]	=	0.920678905	;
    dwLookup	[	7	]	[	29	]	=	0.92518185	;
    dwLookup	[	7	]	[	30	]	=	0.931763076	;
    dwLookup	[	7	]	[	31	]	=	0.93591964	;
    dwLookup	[	7	]	[	32	]	=	0.940595774	;
    dwLookup	[	7	]	[	33	]	=	0.948042951	;
    dwLookup	[	7	]	[	34	]	=	0.951506754	;
    dwLookup	[	7	]	[	35	]	=	0.954970558	;
    dwLookup	[	7	]	[	36	]	=	0.959300312	;
    dwLookup	[	7	]	[	37	]	=	0.960859023	;
    dwLookup	[	7	]	[	38	]	=	0.962937305	;
    dwLookup	[	7	]	[	39	]	=	0.966054728	;
    dwLookup	[	7	]	[	40	]	=	0.96761344	;
    dwLookup	[	7	]	[	41	]	=	0.969691722	;
    dwLookup	[	7	]	[	42	]	=	0.972809144	;
    dwLookup	[	7	]	[	43	]	=	0.973501905	;
    dwLookup	[	7	]	[	44	]	=	0.974541046	;
    dwLookup	[	7	]	[	45	]	=	0.977138899	;
    dwLookup	[	7	]	[	46	]	=	0.979563561	;
    dwLookup	[	7	]	[	47	]	=	0.981815033	;
    dwLookup	[	7	]	[	48	]	=	0.983893315	;
    dwLookup	[	7	]	[	49	]	=	0.985278836	;
    dwLookup	[	7	]	[	50	]	=	0.985798407	;
    dwLookup	[	7	]	[	51	]	=	0.987530308	;
    dwLookup	[	7	]	[	52	]	=	0.988049879	;
    dwLookup	[	7	]	[	53	]	=	0.988742639	;
    dwLookup	[	7	]	[	54	]	=	0.990128161	;
    dwLookup	[	7	]	[	55	]	=	0.991167302	;
    dwLookup	[	7	]	[	56	]	=	0.991860062	;
    dwLookup	[	7	]	[	57	]	=	0.993591964	;
    dwLookup	[	7	]	[	58	]	=	0.994457915	;
    dwLookup	[	7	]	[	59	]	=	0.994631105	;
    dwLookup	[	7	]	[	60	]	=	0.994977485	;
    dwLookup	[	7	]	[	61	]	=	0.995150675	;

    for (int i = 62; i < 288; i++)
    {
        dwLookup	[	7	]	[	i	]	=	1	;
    }

    return;
}

// sets shares of total trips that should originiate in each block
void Simulator::setZoneSharesS (double **zoneShares, double outerRate, double nearRate, double innerRate, double exurbanRate, int numZones, int zoneSize)
{
    int xz, yz;
    double r;
    double newTrips;
    double totTrips = 0;
    r = sqrt (pow(double (xMax - 1) / 2,2) + pow(double (yMax - 1) / 2,2));  //radius from the city center
    for (xz = 0; xz < numZones; xz++){
        for (yz = 0; yz < numZones; yz++){
            zoneShares[xz][yz] = 0;
        }
    }
    // get total trips
    for (int x = 0; x < xMax; x++){
        xz = x / zoneSize;
        for (int y = 0; y < yMax; y++){
            yz = y / zoneSize;
            newTrips = getRate (x, y, r, double(xMax - 1) / 2, double(yMax - 1) / 2, outerRate, innerRate, nearRate, exurbanRate);
            zoneShares[xz][yz] = zoneShares[xz][yz] + newTrips;
            totTrips = totTrips + newTrips;
        }
    }
    // normalize to sum of trips = 1
    for (int xz = 0; xz < numZones; xz++){
        for (int yz = 0; yz < numZones; yz++){
            zoneShares[xz][yz] = zoneShares[xz][yz] / totTrips;
        }
    }
    if (0){
        for (int yz = numZones - 1; yz >= 0; yz--){
            for (int xz = 0; xz < numZones; xz++){
                cout << int (1000 * zoneShares[xz][yz]) << " ";
            }
            cout << endl;
        }
    }
    return;
}
void Simulator::setZoneSharesL (double **zoneShares, double outerRate, double nearRate, double innerRate, double exurbanRate, int numZones, int zoneSize)
{
    int xz, yz;
    double r;
    double newTrips;
    double totTrips = 0;
    r = sqrt (pow(double (xMax - 1) / 2,2) + pow(double (yMax - 1) / 2,2));  //radius from the city center
    for (xz = 0; xz < numZones; xz++){
        for (yz = 0; yz < numZones; yz++){
            zoneShares[xz][yz] = 0;
        }
    }
    // get total trips
    for (int x = 0; x < xMax; x++){
        xz = x / zoneSize;
        for (int y = 0; y < yMax; y++){
            yz = y / zoneSize;
            newTrips = getRate (x, y, r, double(xMax - 1) / 2, double(yMax - 1) / 2, outerRate, innerRate, nearRate, exurbanRate);
            zoneShares[xz][yz] = zoneShares[xz][yz] + newTrips;
            totTrips = totTrips + newTrips;
        }
    }
    // normalize to sum of trips = 1
    for (int xz = 0; xz < numZones; xz++){
        for (int yz = 0; yz < numZones; yz++){
            zoneShares[xz][yz] = zoneShares[xz][yz] / totTrips;
        }
    }
    if (0){
        for (int yz = numZones - 1; yz >= 0; yz--){
            for (int xz = 0; xz < numZones; xz++){
                cout << int (1000 * zoneShares[xz][yz]) << " ";
            }
            cout << endl;
        }
    }
    return;
}


// restores status to previous iteration
void Simulator::restoreStatus( int* timeTripCounts, int maxTrav,  double* maxCarUse, int& totDist,
                   int& unoccDist, bool reportProcs, int& saveRate, bool& warmStart, int& tCount, char fileName[], bool& error, bool& wStart, int& startIter)
{
    int xMaxTest, yMaxTest;
    char dummyStr[20];
    char wStatCh;

    ifstream infile;
    infile.open(fileName);

    if (infile.fail())
    {
        cout << "Error opening input file! " << fileName << " does not exist." << endl;
        cin >> dummyStr;
        error = true;
        return;
    }

    infile >> dummyStr >> dummyStr; // "Writing Parameters"
    infile >> dummyStr >> wStatCh;
    if (wStatCh == 'W')
    {
        wStart = true;
    } else {
        wStart = false;
    }
    infile >> dummyStr >> tCount;
    infile >> dummyStr >> xMaxTest;
    infile >> dummyStr >> yMaxTest;
    infile >> dummyStr >> maxTrav;
    infile >> dummyStr >> maxCarUse [0];
    infile >> dummyStr >> totDist;
    infile >> dummyStr >> unoccDist;
    infile >> dummyStr >> reportProcs;
    infile >> dummyStr >> saveRate;

    if (xMaxTest != xMax || yMaxTest !=yMax)
    {
        cout << "Error! File and program x-y dimensions do not match!" << endl;
        cin >> dummyStr;
        error = true;
        return;
    }

    infile >> dummyStr >> dummyStr; // "Writing matrices"
    infile >> dummyStr; // "TimeTripCounts"
    readTimeTripCounts(infile, timeTripCounts);
    infile >> dummyStr; // "TimeTripMatrix"
    readTimeTripMatrix(infile, timeTripCounts );
    infile >> dummyStr; // "NumCars"
    readNumCars(infile);
    infile >> dummyStr; // "CarMatrix"
    readCarMx(infile);

    infile.close();

    return;
}

//****Functions called from findDistWeight*****************************************************************************************************************

// counts the difference in trips generated in the outer areas, compared to trips ending in the outer areas
int Simulator::countDiff (double outerRate, double innerRate, double nearRate, double exurbanRate, double* startTimes, double* tripDist, int* timeTripCounts, double distWt, bool reportProcs)
{
    int outerSt = 0;
    int innerSt = 0;
    int outerEnd = 0;
    int innerEnd = 0;

    double midPtX = xMax / 4;
    double midPtY = yMax / 4;

    bool printZ = false;
    char printN [20];
    strcpy(printN, "\0");

    generateTrips (true);

    // count start and end rates in inner and outer areas
    for (int t = 0; t < 288; t++)
    {
        for (int trp = 0; trp < timeTripCounts[t]; trp++)
        {
            if (TTMx[t][trp].startX < midPtX || TTMx[t][trp].startY < midPtY || TTMx[t][trp].startX >= xMax - midPtX || TTMx[t][trp].startY >= yMax - midPtY)
            {
                outerSt++;
            } else {
                innerSt++;
            }

            if (TTMx[t][trp].endX < midPtX || TTMx[t][trp].endY < midPtY || TTMx[t][trp].endX >= xMax - midPtX || TTMx[t][trp].endY >= yMax - midPtY)
            {
                outerEnd++;
            } else {
                innerEnd++;
            }
        }
    }

    // reset time trip counts
    for (int t = 0; t < 288; t++)
    {
	TTMx[t].clear();
        timeTripCounts[t] = 0;
    }

    return outerSt - outerEnd;
}


//****Functions called from generateTrips*****************************************************************************************************************


// returns the maximum trip generation rate for a given zone
double Simulator::getRate (int x, int y, double r, double xCent, double yCent, double outerRate, double innerRate, double nearRate, double exurbanRate)
{
	double rate;
	double distToCen = sqrt (pow((x - xCent),2) + pow((y - yCent),2));

	if (xMax >= 100){

		if (distToCen <= innerDist)
		{
			rate = innerRate;//nearRate * (distToCen / innerDist) + innerRate * ((nearDist - distToCen) / innerDist);
    		} 
    		else if (distToCen <= nearDist)
    		{
			rate = nearRate;//outerRate * ((distToCen - innerDist) / (nearDist - innerDist)) + nearRate * ((nearDist - distToCen)/(nearDist - innerDist));
    		}
    		else if (distToCen <= outerDist){
        		rate = outerRate;//exurbanRate * ((distToCen - nearDist) / (r - nearDist)) + outerRate * (r - distToCen) / (r - nearDist);
    		} else {
			rate = exurbanRate;
		}


    	} else {

		if (distToCen <= innerDist)
        	{
        		rate = nearRate * (distToCen / nearDist) + innerRate * ((nearDist - distToCen) / nearDist);
    		} else {
		        rate = outerRate * ((distToCen - nearDist) / (r - nearDist)) + nearRate * (r - distToCen) / (r - nearDist);
        	}
    	}

    	return rate;
}

/* returns the maximum trip generation rate for a given zone
double getRate (int x, int y, double r, double xCent, double yCent, double outerRate, double innerRate, double nearRate)
{
    double rate;
    double distToCen = sqrt (pow((x - xCent),2) + pow((y - yCent),2));

    if (distToCen <= nearDist)
    {
        rate = nearRate * (distToCen / nearDist) + innerRate * ((nearDist - distToCen) / nearDist);
    } else {
        rate = outerRate * ((distToCen - nearDist) / (r - nearDist)) + nearRate * (r - distToCen) / (r - nearDist);
    }

    return rate;
}
*/

// returns the time of day for a given trip (0 to 287 for 24 hours divided into 5 minute blocks)
int Simulator::getStartTime(double* startTimes)
{

    double rtime;
    int t = 0;
    bool found = false;

    rtime = rand();
    rtime = rtime / RAND_MAX;

    while (t < 288 && !found)
    {
        if (rtime < startTimes[t])
        {
            found = true;
        } else {
            t = t + 1;
        }
    }

    if (t == 288) // ensure that we don't exceed our max time
    {
        t = 287;
    }

    return t;
}

// assigns a destination for new trips
void Simulator::getDest (int x, int y, int &newX, int &newY, double* tripDist, double dWt)
{
    double rx, ry, xc, yc;
    int tdist, xtrav, ytrav, maxUD, maxLR; //, urbBoundLR, urbBoundUD;
    bool right, up, repeat;

    repeat = true;

    tdist = getDestDist(tripDist);

    for (int iter = 0; repeat == true; iter ++)
    {
        rx = rand();
        rx = (xMax - 1) * rx / RAND_MAX;
        ry = rand();
        ry = (yMax - 1) * ry / RAND_MAX;

        // if dWt = 0, greater attraction towards center, if dWt = 1, equal attaction all directions
        xc = (x * (1 - dWt)) + (0.5 * dWt * xMax);
        yc = (y * (1 - dWt)) + (0.5 * dWt * yMax);

        // pick x direction
        if (rx > xc)
        {
            right = true;
            maxLR = xMax - x - 1;
//            urbBoundLR = ((xMax - 1) / 2) + nearDist - x;
        } else {
            right = false;
            maxLR = x;
//            urbBoundLR = x - (((xMax - 1) / 2) - nearDist);
        }

        // pick y direction
        if (ry > yc)
        {
            up = true;
            maxUD = yMax - y - 1;
//            urbBoundUD = ((yMax - 1) / 2) + nearDist - y;
        } else {
            up = false;
            maxUD = y;
//            urbBoundUD = y - (((yMax - 1) / 2) - nearDist);
        }

        rx = rand();
        rx = rx / RAND_MAX;

        xtrav = (int) ((tdist + 1) * rx);
        ytrav = tdist - xtrav;

// verify that the trip will not exceed the bounds of the city
        if (xtrav > maxLR || ytrav > maxUD)
        {
            repeat = true;
        } else {
            repeat = false;
        }

        if (iter > 20)
        {
            iter = 0;
            tdist = getDestDist(tripDist);
        }

/* ***************************************************************

        if (xtrav > urbBoundLR || ytrav > urbBoundUD)
        {
            rx = rand();
            rx = rx / RAND_MAX;
            if (rx < 0.95)
            {
                repeat = true;
            }

        }
// ***************************************************************
*/
    }

    if (!right)
    {
        xtrav = -xtrav;
    }
    if (!up)
    {
        ytrav = -ytrav;
    }

    newX = x + xtrav;
    newY = y + ytrav;

    if (newX < 0 || newX >= xMax)
    {
        newX = xMax / 2;
    }

    if (newY < 0 || newY >= yMax)
    {
        newY = yMax / 2;
    }

// Changed to above the if statements -JPH
    // newX = x + xtrav;
    //newY = y + ytrav;

    return;
}

int Simulator::getDestDist(double* tripDist)
{
    int dist;
    double R;
    bool success = false;

    R = rand ();
    R = R / RAND_MAX;

    for (int i = 0; i < tripDistSize && !success; i++)
    {
        if (R < tripDist[i])
        {
            dist = i + 1;
            success = true;
        }
    }

    return dist;
}

// places trips in the time-trip matrix
void Simulator::placeInTTM ( Trip ntrip, int* timeTripCounts, int time)
{
    
    TTMx[time].push_back(ntrip);
//    TTMx[time][TTMx[time].size() - 1].waitPtr = &TTMx[time][TTMx[time].size() - 1];
    timeTripCounts[time]++;
	if(TTMx[time].size() != timeTripCounts[time])
		cout << "Trip # (vec first): " << TTMx[time].size() << " = " << timeTripCounts[time] << " ?"<<endl;
    return;
}

// generates trip using a Poisson distribution
int Simulator::genPoisson (double mean)  //Generated using the Box-Muller method
{
    double R;
    double sum = 0;
    int i = -1;
    double z;

    R = rand ();

    while (sum <= mean)
    {
        R = rand ();
        R = R / (RAND_MAX);
        z = -log(R);
        sum += z;
        i++;
    }
    return i;
}

//****Functions called from runSharedAV*****************************************************************************************************************


//Saves the status of the program
void Simulator::saveStatus( int* timeTripCounts, int maxTrav,  double* maxCarUse, int totDist,
                 int unoccDist, bool reportProcs, int& saveRate, bool warmStart, int tCount)
{
    ofstream saveFile;
    int firstDig, secDig, thirdDig;
    char firstDigCh, secDigCh, thirdDigCh;
    char wChar;
    char fName[19];

    // generate the savefile name and create the new file
    thirdDig = tCount % 10;
    secDig = ((tCount - thirdDig) / 10) % 10;
    firstDig = (tCount - (secDig * 10) - thirdDig) / 100;
    firstDigCh = (char) (firstDig + 48);
    secDigCh = (char) (secDig + 48);
    thirdDigCh = (char) (thirdDig + 48);

    if (warmStart)
    {
        wChar = 'W';
    } else {
        wChar = 'F';
    }

    strcpy(fName, "CarStatusXXXX.txt");

    fName[9] = firstDigCh;
    fName[10] = secDigCh;
    fName[11] = thirdDigCh;
    fName[12] = wChar;

    saveFile.open(fName);

    // begin writing to the savefile
    saveFile << "Writing Parameters" << endl << endl;
    saveFile << "WarmStart " << wChar << endl;
    saveFile << "TimeCount " << tCount << endl;
    saveFile << "xMax " << xMax << endl;
    saveFile << "yMax " << yMax << endl;
    saveFile << "maxTrav " << maxTrav << endl;
    saveFile << "maxCarUse " << maxCarUse << endl;
    saveFile << "totDist " << totDist << endl;
    saveFile << "unoccDist " << unoccDist << endl;
    saveFile << "reportProcs " << reportProcs << endl;
    saveFile << "saveRate " << saveRate << endl << endl;

//    saveFile << "Writing Matrices" << endl << endl;
    saveFile << endl << "TimeTripCounts" << endl;
    writeTimeTripCounts(saveFile, timeTripCounts);
//    saveFile << endl << "TimeTripMatrix" << endl;
//    writeTimeTripMatrix(saveFile, timeTripCounts, TTMx);
    saveFile << endl << "NumCars" << endl;
    writeNumCars(saveFile);
    saveFile << endl << "Free Cars" << endl;
    writeNumFreeCars(saveFile);
//    saveFile << endl << "CarMatrix" << endl;
//    writeCarMx(saveFile, CarMx);

/*    double avgsx, avgsy, avgdx, avgdy;

    avgsx = 0;
    avgsy = 0;
    avgdx = 0;
    avgdy = 0;

    saveFile << endl << "*** Results ***" << endl << endl;

    for (int tm = 0; tm < 288; tm++)
    {
        for (int trp = 0; trp < timeTripCounts[tm]; trp++)
        {
            avgsx = avgsx + TTMx[trp][tm].startX;
            avgsy = avgsy + TTMx[trp][tm].startY;
            avgdx = avgdx + TTMx[trp][tm].endX;
            avgdy = avgdy + TTMx[trp][tm].endY;
        }
        saveFile << TTMx[0][tm].startX << " " << TTMx[0][tm].startY << " " << endl;
    }

    saveFile << endl << " avgsx " << avgsx << " avgsy " << avgsy << " avgdx " << avgdx << " avgdy " << avgdy << endl;

*/

    saveFile.close();

    return;
}


//Finds the nearest available car to a given trip
void Simulator::findNearestCar (Trip& trp, int dist, int maxDist,  bool reportProcs,
                     int& nw, int& ne, int& se, int& sw, int& coldStart, int& hotStart, int run, bool checkStationDistance)
{
    int d;
    int x;
    int y;
    int c;
    int numveh = 0;
    double r;
    double waitTrav = 0;
    int tripDist = abs(trp.startX - trp.endX) + abs(trp.startY - trp.endY);
    bool found = false;
    int nearStationDist; 
    if (canRefuelAnywhere || !checkStationDistance){
	nearStationDist = 0;
    }
    else{
	nearStationDist = nearestStationDistance(trp.endX, trp.endY);
    }
    if (dist == 0)
    {
        found = lookForCar (trp.startX, trp.startY, run, tripDist + nearStationDist, c);
        if (found)
        {
            assignCar(trp.startX, trp.startY, c, &trp);
            x = trp.startX;
            y = trp.startY;
        }
    } else {

        r = rand();
        r = r / RAND_MAX;

        if (r < 0.25) // start with the northwest quadrant
        {
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, run, tripDist + dist + nearStationDist, c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c,  &trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, run, tripDist + dist + nearStationDist, c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                ne++;
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, run, tripDist+dist+nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                se++;
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, run,tripDist+dist+nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c,  &trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }

        } else if (r < 0.5) {// start with the northeast quadrant

            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, run, tripDist + dist + nearStationDist, c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c,  &trp);
                    numveh = CarMx[x][y].size();
                }
                ne++;
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, run, tripDist + dist + nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c,  &trp);
                    numveh = CarMx[x][y].size();
                }
                se++;
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, run, tripDist + dist + nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, run, tripDist + dist +nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }

        } else if (r < 0.75) { // start with the southeast quadrant

            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, run, tripDist + dist + nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                se++;
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, run, tripDist + dist + nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, run, tripDist + dist +nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, run, tripDist+dist + nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                ne++;
            }

        } else { // start with the southwest quadrant

            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, run, tripDist+dist+nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c,  &trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, run, tripDist + dist+nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, run, tripDist + dist +nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
               ne++;
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, run, tripDist + dist +nearStationDist,c);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, &trp);
                    numveh = CarMx[x][y].size();
                }
                se++;
            }
        }
    }

    if (found)
    {
        trp.carlink = true;	
        waitTrav = abs(x - trp.startX) + abs(y - trp.startY);
	
        trp.waitTime = trp.waitTime + (5.0 * waitTrav / maxDist) ;
        if (trp.waitPtr != NULL)
        {
            trp.waitPtr -> waitTime = trp.waitTime;
	    trp.waitPtr-> carlink = true;
        }
	//if (trp.waitPtr == &TTMx[3][0])
	//	cout << "wait list trip! "<<waitTrav<<endl;	
	//else if (&trp == &TTMx[3][0])
	//	cout << "actual trip" << endl;
    }

    if (reportProcs)
    {
        if (found)
        {
            cout << " found car! x " << x << " y " << y << " c " << c << endl;
//        } else {
//            cout << " did not find a car x " << x << " y " << y << " c " << c  << endl;
        }
    }

    return;

}

//Looks for a free car at coordinates x,y
bool Simulator::lookForCar (int x, int y, int r, int dist, int& cn)
{
    int c;
    bool found = false;
    int reqFuel = dist;
    // Start at the top of each vector. This makes replacement easier and can keep better track of which cars are actually used,
    // emphasizing heavy reuse of a few cars, rather than cycling through a larger number that were previously created during the warm start
    for (c = CarMx[x][y].size() - 1; c >=0 && !found; c--)
    {
        if (CarMx[x][y][c].inUse == false)
        {

		if (false){ // Dan's version
			found = true;
			cn = c;
		} else if (true){	// Donna's version
	        	if (CarMx[x][y][c].gas >= reqFuel){
				found = true;
            			cn = c;
//				if (CarMx[x][y][c].refuel > 0)
//					cout << "Assigning car that needs to refuel" << endl;
			} else {
				CarMx[x][y][c].numRejects++;
/*				if (CarMx[x][y][c].numRejects >= rejectLimit){
//					cout << "Car needs to refuel" << endl;
					CarMx[x][y][c].refuel = refuelTime;
					CarMx[x][y][c].destX = x;
					CarMx[x][y][c].destY = y;
					CarMx[x][y][c].pickupX = -1;
					CarMx[x][y][c].pickupY = -1;
					CarMx[x][y][c].currTrip = NULL;
					if (CHARGE_IN_PLACE)
						cellChargeCount[x][y][r] ++;
					CarMx[x][y][c].inUse = true;
				}*/
			}
		}
        }
    }

    return found;
}

void Simulator::assignCar (int x, int y, int c, Trip* trp)
{
    double randRet;
   
    CarMx[x][y][c].inUse = true;
    CarMx[x][y][c].destX = trp->endX;
    CarMx[x][y][c].destY = trp->endY;
    CarMx[x][y][c].tripCt ++;    

    revenue += trp->price;

    if (trp->waitPtr != NULL){
      CarMx[x][y][c].currTrip = trp->waitPtr;
//	trp->waitPtr->carlink = true;
    }else{
      CarMx[x][y][c].currTrip = trp;
//	trp->carlink = true;
    }
    if (x == trp->startX && y == trp->startY)
    {
        CarMx[x][y][c].pickupX = -1;
        CarMx[x][y][c].pickupY = -1;
    } else {
        CarMx[x][y][c].pickupX = trp->startX;
        CarMx[x][y][c].pickupY = trp->startY;
    }

//    if (trp == &TTMx[3][0])
//	cout << "Assigned the problem trip"<<endl;

    // for cars departing from home, assume 22% return by other means or another day and the rest will return
    if (trp->returnHome == false)
    {
        randRet = rand();
        randRet = randRet / RAND_MAX;

        if (randRet > 0.22)
        {
            CarMx[x][y][c].returnHome = true;
            CarMx[x][y][c].retHX = trp->startX;
            CarMx[x][y][c].retHY = trp->startY;
        }
    }

    // if for some reason we have an invalid destination, assign a default center location
    if (CarMx[x][y][c].destX < 0 || CarMx[x][y][c].destX >= xMax)
    {
        CarMx[x][y][c].destX = xMax / 2;
    }

    if (CarMx[x][y][c].destY < 0 || CarMx[x][y][c].destY >= yMax)
    {
        CarMx[x][y][c].destY = yMax / 2;
    }

    return;
}

// generates a new car at the location of trp
Car Simulator::genNewCar (Trip trp)
{
    Car nCar;
    double randGas;

    nCar.inUse = true;
    nCar.x = trp.startX;
    nCar.y = trp.startY;
    nCar.startX = trp.startX;
    nCar.startY = trp.startY;
    nCar.destX = trp.endX;
    nCar.destY = trp.endY;
    nCar.pickupX = -1;
    nCar.pickupY = -1;
    nCar.tElapsed = 13;
    nCar.returnHome = !trp.returnHome;
    nCar.retHX = trp.startX;
    nCar.retHY = trp.startY;
    nCar.tripCt = 0;
    nCar.numRejects = 0;
    nCar.stuck = false;
    nCar.needFuel = false;
    nCar.stationLink = false;
    randGas = rand();
    randGas = randGas / RAND_MAX;
    nCar.moved = false;
    nCar.gas = carRange;// * randGas;
    nCar.refuel = 0;
//    cout << "New Car Gas: "<<nCar.gas<<endl;
    return nCar;
}

// moves a car that is currently in use, up to maximum distance of maxTrav.  First pickup, then dropoff.
void Simulator::moveCar (int x, int y, int c, int t, int maxTrav, int& totDist, int& unoccDist, int& waitT,
              double dwLookup [][288], int* timeTripCounts, bool reportProcs, int& hotStarts, int& coldStarts, int& trackX, int& trackY, int& trackC, int iter)
{
    
    // identify target car in the car matrix
    Car tCar = CarMx[x][y][c];
    bool needFuel = false;
    if(tCar.needFuel){
	needFuel = true;
	//tCar.pickupX = -1;
	//tCar.pickupY = -1;
	//cout << "Should not be moving from: " << x <<" "<<y<<" to "<<tCar.destX << " " << tCar.destY << " through " << tCar.pickupX << " " <<tCar.pickupY<<endl;
    }

    int trav = maxTrav;
    double wait; 
    if (tCar.pickupX != -1 || tCar.pickupY != -1)
    {
        if (reportProcs)
        {
            cout << "Pickup: Moving car from (" << tCar.x << "," << tCar.y << ") to (" << tCar.pickupX << "," << tCar.pickupY << ")" << endl;
            cout << "Tot dist " << totDist << endl;
        }

        // the vehicle does not have the passenger, go to pickup location
//        unoccDist = unoccDist + abs(tCar.pickupX - tCar.x) + abs(tCar.pickupY - tCar.y);
//        totDist = totDist + abs(tCar.pickupX - tCar.x) + abs(tCar.pickupY - tCar.y);
//        waitT = waitT + abs(tCar.pickupX - tCar.x) + abs(tCar.pickupY - tCar.y);
//        trav = trav - abs(tCar.pickupX - tCar.x) - abs(tCar.pickupY - tCar.y);
//        tCar.x = tCar.pickupX;
//        tCar.y = tCar.pickupY;
	if (tCar.pickupX != tCar.x && trav > 0)
        {

                if (trav >= abs(tCar.pickupX - tCar.x))
                {
                        trav = trav - abs(tCar.pickupX - tCar.x);
                        totDist = totDist + abs(tCar.pickupX - tCar.x);
                        unoccDist += abs(tCar.pickupX - tCar.x);
                        waitT += abs(tCar.pickupX - tCar.x);
                        tCar.x = tCar.pickupX;
//                      CarMx[x][y][c].pickupX = -1;
           //             tCar.pickupX = -1;
                } else if (tCar.pickupX > tCar.x) {
                        tCar.x = tCar.x + trav;
                        totDist = totDist + trav;
			unoccDist += trav;
                        trav = 0;
                } else {
                        tCar.x = tCar.x - trav;
                        totDist = totDist + trav;
			unoccDist += trav;
                        trav = 0;
                }
        }

        // move the car in the y direction
        if (tCar.pickupY != tCar.y && trav > 0)
        {
                if (trav >= abs(tCar.pickupY - tCar.y))
                {
                        trav = trav - abs(tCar.pickupY - tCar.y);
                        totDist = totDist + abs(tCar.pickupY - tCar.y);
                        unoccDist += abs(tCar.pickupY - tCar.y);
                        waitT += abs(tCar.pickupY - tCar.y);
                        tCar.y = tCar.pickupY;
//                      CarMx[x][y][c].pickupY = -1;
         //               tCar.pickupY = -1;
                } else if (tCar.pickupY > tCar.y) {
                        tCar.y = tCar.y + trav;
                        totDist = totDist + trav;
			unoccDist += trav;
                        trav = 0;
                } else {
                        tCar.y = tCar.y - trav;
                        totDist = totDist + trav;
			unoccDist += trav;
                        trav = 0;
                }
        }

    }

    if (reportProcs)
    {
        cout << "Moving car from (" << tCar.x << "," << tCar.y << ") to (" << tCar.destX << "," << tCar.destY << ")" << endl;
        cout << "Tot dist " << totDist << endl;
    }

      if (trav > 0 || (trav == 0 && tCar.pickupX == tCar.x && tCar.pickupY == tCar.y)){ // Car reached pick up location

	// If car is not refueling and at pick up location
	if (!tCar.needFuel && tCar.currTrip != NULL){
	
		if(tCar.x == tCar.currTrip->startX && tCar.y == tCar.currTrip->startY){

			double waitTrav = abs(tCar.currTrip->startX - x) + abs(tCar.currTrip->startY - y);
			wait = 5 * (t - tCar.currTrip->startTime);
			wait +=  (5 * waitTrav / maxTrav);
			//CarMx[x][y][c].currTrip->waitTime = wait;
			/*if (abs(wait - CarMx[x][y][c].currTrip->waitTime) >= 0.000001){
				cout << "Time: " << tCar.currTrip->startTime << " " << t <<endl;
				cout << "Wait time error: " << wait << " " << CarMx[x][y][c].currTrip->waitTime<<endl;
				cout << "Wait Trav: "<<waitTrav<< " over "<< maxTrav<<endl;
				cout << "Car at: "<<x<<","<<y<<" Trip start: "<<tCar.currTrip->startX<<","<<tCar.currTrip->startY<<endl;
				cout << "Travel: "<<trav<<endl;
			}*/ // This will print out an error but it is due to speed changes in different zones
			CarMx[x][y][c].currTrip->waitTime = wait;
		}
	}

	CarMx[x][y][c].pickupX = -1;
        CarMx[x][y][c].pickupY = -1;


	}
    else { // Car did not reach pick up location
//	if (ALGORITHM == GREEDY)
//		cout << "should not be in here for GREEDY match"<<endl;
	if (CarMx[x][y][c].needFuel == false){
          waitT = waitT + trav;
	}
    }

    // move the car in the x direction
    if (tCar.destX != tCar.x && trav > 0)
    {

        if (trav >= abs(tCar.destX - tCar.x))
        {
            trav = trav - abs(tCar.destX - tCar.x);
            totDist = totDist + abs(tCar.destX - tCar.x);
            tCar.x = tCar.destX;
        } else if (tCar.destX > tCar.x) {
            tCar.x = tCar.x + trav;
            totDist = totDist + trav;
            trav = 0;
        } else {
            tCar.x = tCar.x - trav;
            totDist = totDist + trav;
            trav = 0;
        }
    }

    // move the car in the y direction
    if (tCar.destY != tCar.y && trav > 0)
    {
        if (trav >= abs(tCar.destY - tCar.y))
        {
	
	    totDist = totDist + abs(tCar.destY - tCar.y);
            tCar.y = tCar.destY;
        } else if (tCar.destY > tCar.y) {
            tCar.y = tCar.y + trav;
            totDist = totDist + trav;
        } else {
            tCar.y = tCar.y - trav;
            totDist = totDist + trav;
        }
    }

    // Moving to charging station is unoccupied travel
    if (CarMx[x][y][c].needFuel)
	unoccDist += maxTrav - trav;

    // now that we have the new destination, move the car
    if (reportProcs)
    {
        cout << endl; // << "Tot dist " << totDist << endl;
        cout << "Moving car from " << x << "," << y << " to " << tCar.x << "," << tCar.y << endl;
    }
//    if(CarMx[x][y][c].gas < abs(x - tCar.x) + abs(y - tCar.y))
//	cout << "Not enough fuel: " << CarMx[x][y][c].refuel << endl;
    move (x, y, tCar.x, tCar.y, c, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);

    return;
}

// randomizes the ordering of a given vector, numVals long
void Simulator::randOrdering(int* xRandOrd, int numVals)
{
    vector <int> vect; // vectors and vector iterators for randomizing order of vehicle reallocation
    vector <int>::iterator itv;
    int j = 0;

    // load vector with randomized ordering from 0 to numVals
    for (int i = 0; i < numVals; i++)
    {
        vect.push_back(i);
    }

    random_shuffle(vect.begin(), vect.end());

    for (itv = vect.begin(); itv != vect.end(); ++itv)
    {
        xRandOrd[j] = *itv;
        j++;
    }

    return;
}

void Simulator::runSummary(bool warmStart, bool lastWarm, int **zoneGen, int **waitZones, double **netZoneBalance, int **tripO, int **tripD, int* cardDirectLZ,
                int* cardDirect, int* cardDirect2,  int* timeTripCounts)
{

    cout << endl << endl << "****************************************" << endl << endl;

    if (lastWarm)
    {
        cout << "Generating possible final warm attempt" << endl;
    }

    for (int tim = 0; tim < 288; tim++)
    {
        for (int trp = 0; trp < timeTripCounts[tim]; trp++)
        {
            tripO[TTMx[tim][trp].startX / zoneSizeL][TTMx[tim][trp].startY / zoneSizeL]++;
            tripD[TTMx[tim][trp].endX / zoneSizeL][TTMx[tim][trp].endY / zoneSizeL]++;
        }
    }

    cout << "Total trip origins by block:" << endl << endl;
    for (int y = numZonesL - 1; y >= 0; y--)
    {
        for (int x = 0; x < numZonesL; x++)
        {
            cout << tripO[x][y] << "  ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "Total trip destinations by block:" << endl << endl;
    for (int y = numZonesL - 1; y >= 0; y--)
    {
        for (int x = 0; x < numZonesL; x++)
        {
            cout << tripD[x][y] << "  ";
        }
        cout << endl;
    }
    cout << endl;

    if (warmStart)
    {
        cout << "Total generated cars by block:" << endl << endl;
        for (int y = numZonesL - 1; y >= 0; y--)
        {
            for (int x = 0; x < numZonesL; x++)
            {
                cout << zoneGen[x][y] << "   ";
            }
            cout << endl;
        }
    } else {
        cout << "Total wait times by block:" << endl << endl;
        for (int y = numZonesL - 1; y >= 0; y--)
        {
            for (int x = 0; x < numZonesL; x++)
            {
                cout << waitZones[x][y] << "   ";
            }
            cout << endl;
        }
    }

    if (0)
    {
        cout << endl << "Net Block Balance:" << endl << endl;
        for (int y = numZonesL - 1; y >= 0; y--)
        {
            for (int x = 0; x < numZonesL; x++)
            {
                cout << int(netZoneBalance[x][y]) << "   ";
            }
            cout << endl;
        }
    }

    if (0)
    {
        cout << "Block reallocation total: " << cardDirectLZ[0] << "  North: " << cardDirectLZ[1] << "  South: " << cardDirectLZ[2] << "  West: " << cardDirectLZ[3]
             << "  East: " << cardDirectLZ[4] << endl;
    }

    ofstream saveFile;
    char fName[19];
    strcpy(fName, "TripDist.txt");
    saveFile.open(fName);
    writeTimeTripCounts(saveFile, timeTripCounts);
    saveFile.close();

    cout << endl << "****************************************" << endl << endl;

    return;
}

// reallocate vehicles if there are 3 or more cars in the target grid cell than an adjacent cell
void Simulator::reallocVehs(int x, int y, int* timeTripCounts, double dwLookup [][288],
                 bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect, int& trackX, int& trackY, int& trackC, int iter)
{
    int numfreeCars = 0;
    int nfreeCars = 0;
    int sfreeCars = 0;
    int wfreeCars = 0;
    int efreeCars = 0;
    double NRank, SRank, WRank, ERank, randDir;
    int cNum = -1;


    // find # of vehicles in cell
    if (CarMx[x][y].size() > 2)
    {
        for (int i = 0; i < CarMx[x][y].size(); i++)
        {
            if (CarMx[x][y][i].inUse == false)
            {
                numfreeCars++;
            }
            if (CarMx[x][y][i].moved == false && (canRefuelAnywhere || (!CarMx[x][y][i].inUse && CarMx[x][y][i].gas >= 1 + 8))) // 1 for move plus 2 miles
            {
                cNum = i;
            }
        }
        if (numfreeCars > 2 && cNum != -1)
        {

            // print the car matrix to the screen
            if (reportProcs)
            {
                for (int yc = yMax - 1; yc >= 0; yc--)
                {
                    for (int xc = 0; xc < xMax; xc++)
                    {
                        cout << CarMx[xc][yc].size() << " ";
                    }
                    cout << endl;
                }
                cout << endl;
            }

            // north
            if (y != yMax - 1)
            {
                for (int i = 0; i < CarMx[x][y+1].size(); i++)
                {
                    if (CarMx[x][y+1][i].inUse == false)
                    {
                        nfreeCars++;
                    }
                }
            } else {
                nfreeCars = 1000;
            }
            // south
            if (y != 0)
            {
                for (int i = 0; i < CarMx[x][y-1].size(); i++)
                {
                    if (CarMx[x][y-1][i].inUse == false)
                    {
                        sfreeCars++;
                    }
                }
            } else {
                sfreeCars = 1000;
            }
            // west
            if (x != 0)
            {
                for (int i = 0; i < CarMx[x-1][y].size(); i++)
                {
                    if (CarMx[x-1][y][i].inUse == false)
                    {
                        wfreeCars++;
                    }
                }
            } else {
                wfreeCars = 1000;
            }
            // east
            if (x != xMax - 1)
            {
                for (int i = 0; i < CarMx[x+1][y].size(); i++)
                {
                    if (CarMx[x+1][y][i].inUse == false)
                    {
                        efreeCars++;
                    }
                }
            } else {
                efreeCars = 1000;
            }



            while ((numfreeCars > nfreeCars+2 || numfreeCars > sfreeCars+2 || numfreeCars > wfreeCars+2 || numfreeCars > efreeCars+2) && cNum != -1)
            {
                NRank = nfreeCars + (((y + 1) - double(((yMax - 1) / 2))) / 1000);
                SRank = sfreeCars - (((y - 1) - double(((yMax - 1) / 2))) / 1000);
                WRank = wfreeCars - (((x - 1) - double(((xMax - 1) / 2))) / 1000);
                ERank = efreeCars + (((x + 1) - double(((xMax - 1) / 2))) / 1000);

                // update vehicle starts
                CarMx[x][y][cNum].moved = true;

                if (NRank < SRank && NRank < WRank && NRank < ERank)
                {
                    // move north
                    CarMx[x][y][cNum].destX = x;
                    CarMx[x][y][cNum].destY = y+1;
                    move (x, y, x, y + 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars--;
                    nfreeCars++;
                } else if (SRank < NRank && SRank < WRank && SRank < ERank) {
                    // move south
                    CarMx[x][y][cNum].destX = x;
                    CarMx[x][y][cNum].destY = y-1;
                    move (x, y, x, y - 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars--;
                    sfreeCars++;
                } else if (WRank < NRank && WRank < SRank && WRank < ERank) {
                    // move west
                    CarMx[x][y][cNum].destX = x-1;
                    CarMx[x][y][cNum].destY = y;
                    move (x, y, x - 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars--;
                    wfreeCars++;
                } else if (ERank < NRank && ERank < SRank && ERank < WRank) {
                    // move east
                    CarMx[x][y][cNum].destX = x+1;
                    CarMx[x][y][cNum].destY = y;
                    move (x, y, x + 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars--;
                    efreeCars++;
                } else {
                    // 2 or more have the same ranking
                    if (SRank + NRank < WRank + ERank)
                    {
                        // move north or south
                        if (SRank < NRank)
                        {
                            // move south
                            CarMx[x][y][cNum].destX = x;
                            CarMx[x][y][cNum].destY = y-1;
                            move (x, y, x, y - 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                            numfreeCars--;
                            nfreeCars++;
                        } else if (NRank < SRank) {
                            // move north
                            CarMx[x][y][cNum].destX = x;
                            CarMx[x][y][cNum].destY = y+1;
                            move (x, y, x, y + 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC,iter);
                            numfreeCars--;
                            sfreeCars++;
                        } else {
                            randDir = rand();
                            randDir = randDir / RAND_MAX;
                            if (randDir > 0.5)
                            {
                                // move south
                                CarMx[x][y][cNum].destX = x;
                                CarMx[x][y][cNum].destY = y-1;
                                move (x, y, x, y - 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                                numfreeCars--;
                                sfreeCars++;
                            } else {
                                // move north
                                CarMx[x][y][cNum].destX = x;
                                CarMx[x][y][cNum].destY = y+1;
                                move (x, y, x, y + 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                                numfreeCars--;
                                nfreeCars++;
                            }
                        }
                    } else {
                        // move east or west
                        if (ERank < WRank)
                        {
                            // move east
                            CarMx[x][y][cNum].destX = x+1;
                            CarMx[x][y][cNum].destY = y;
                            move (x, y, x + 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                            numfreeCars--;
                            efreeCars++;
                        } else if (WRank < ERank) {
                            // move west
                            move (x, y, x - 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                            CarMx[x][y][cNum].destX = x-1;
                            CarMx[x][y][cNum].destY = y;
                            numfreeCars--;
                            wfreeCars++;
                        } else {
                            randDir = rand();
                            randDir = randDir / RAND_MAX;
                            if (randDir > 0.5)
                            {
                                // move west
                                CarMx[x][y][cNum].destX = x-1;
                                CarMx[x][y][cNum].destY = y;
                                move (x, y, x - 1, y, 0, cNum, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                                numfreeCars--;
                                wfreeCars++;
                            } else {
                                // move east
                                CarMx[x][y][cNum].destX = x+1;
                                CarMx[x][y][cNum].destY = y;
                                move (x, y, x + 1, y, 0, cNum, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                                numfreeCars--;
                                efreeCars++;
                            }
                        }
                    }
                }

                totDist++;
                unoccDist++;
		reallocDist++;
                // find next non-moved vehicle
                cNum = -1;
                for (int i = 0; i < CarMx[x][y].size(); i++)
                {
                    if (CarMx[x][y][i].moved == false && (canRefuelAnywhere || (!CarMx[x][y][i].inUse && CarMx[x][y][i].gas >= 1 + 8))) // Add check here for moving out of range?
                    {
                        cNum = i;
                    } 
                }

                // print the car matrix to the screen
                if (reportProcs)
                {
                    for (int yc = yMax - 1; yc >=0; yc--)
                    {
                        for (int xc = 0; xc < xMax; xc++)
                        {
                            cout << CarMx[xc][yc].size() << " ";
                        }
                        cout << endl;
                    }
                    cout << endl;
                }

            } // end while loop
        }
    }


    // find # of vec

    return;
}

// moves a vehicle two spaces over, if that cell has no vehicles present
void Simulator::reallocVehs2(int x, int y, int* timeTripCounts, double dwLookup [][288],
                  bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect2, int& trackX, int& trackY, int& trackC, int iter)
{
    double NRank, SRank, WRank, ERank;
    int numfreeCars = 0;
    int cNum = -1;

    // find # of vehicles in cell
    if (CarMx[x][y].size() > 1)
    {
        for (int i = 0; i < CarMx[x][y].size(); i++)
        {
            if (CarMx[x][y][i].inUse == false)
            {
                numfreeCars++;
            }
            if (CarMx[x][y][i].moved == false && (canRefuelAnywhere || (!CarMx[x][y][i].inUse && CarMx[x][y][i].gas >= 2 + 8))) // Add check here for moving away from charge? +2
            {
                cNum = i;
            }
        }

        while (numfreeCars > 1 && cNum != -1)
        {

            // print the car matrix to the screen
            if (reportProcs)
            {
                for (int yc = yMax - 1; yc >= 0; yc--)
                {
                    for (int xc = 0; xc < xMax; xc++)
                    {
                        cout << CarMx[xc][yc].size() << " ";
                    }
                    cout << endl;
                }
                cout << endl;
            }

            NRank = getSurroundCars (x, y + 2);
            SRank = getSurroundCars (x, y - 2);
            WRank = getSurroundCars (x - 2, y);
            ERank = getSurroundCars (x + 2, y);

            if (NRank < 1 || SRank < 1 || WRank < 1 || ERank < 1)
            {

                // update vehicle starts
                CarMx[x][y][cNum].moved = true;

                // print the car matrix to the screen
                if (reportProcs)
                {
                    for (int yc = yMax - 1; yc >= 0; yc--)
                    {
                        for (int xc = 0; xc < xMax; xc++)
                        {
                            cout << CarMx[xc][yc].size() << " ";
                        }
                        cout << endl;
                    }
                    cout << endl;
                }

                if (NRank < SRank && NRank < WRank && NRank < ERank)
                {
                    // move north
                    CarMx[x][y][cNum].destX = x;
                    CarMx[x][y][cNum].destY = y+2;
                    move ( x, y, x, y + 2, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars --;
                    NRank = 1000;
                } else if (SRank < WRank && SRank < ERank) {
                    // move south
                    CarMx[x][y][cNum].destX = x;
                    CarMx[x][y][cNum].destY = y-2;
                    move (x, y, x, y - 2, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars --;
                    SRank = 1000;
                } else if (WRank < ERank) {
                    // move west
                    CarMx[x][y][cNum].destX = x-2;
                    CarMx[x][y][cNum].destY = y;
                    move (x, y, x - 2, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars --;
                    WRank = 1000;
                } else {
                    // move east
                    CarMx[x][y][cNum].destX = x+2;
                    CarMx[x][y][cNum].destY = y;
                    move (x, y, x + 2, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC, iter);
                    numfreeCars --;
                    ERank = 1000;
                }

                // print the car matrix to the screen
                if (reportProcs)
                {
                    for (int yc = yMax - 1; yc >= 0; yc--)
                    {
                        for (int xc = 0; xc < xMax; xc++)
                        {
                            cout << CarMx[xc][yc].size() << " ";
                        }
                        cout << endl;
                    }
                    cout << endl;
                }

                totDist = totDist + 2;
                unoccDist = unoccDist + 2;
		reallocDist += 2;

                // find next non-moved vehicle
                cNum = -1;
                for (int i = 0; i < CarMx[x][y].size(); i++)
                {
                    if (CarMx[x][y][i].moved == false && (canRefuelAnywhere || (!CarMx[x][y][i].inUse && CarMx[x][y][i].gas >= 2 + 8))) // charge dist check here +2
                    {
                        cNum = i;
                    }
                }

            } else {
                cNum = -1;
            }
        } // end while
    }
    return;
}

// reallocates vehicles between zones. Excess cars -> positive zoneBalance, excess trips -> negative zoneBalance.
void Simulator::reallocVehsLZones (int* timeTripCounts, double dwLookup [][288],
                        double **zoneShares, int t, bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav,
                        double **netZoneBalance, int* cardDirectLZ, int **waitZonesT, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC, int iter)
{
    int xb, yb;
    int tx = 0;
    int ty = 0;
    int pushSign;
    double cutoff;
    double absZB;
    double absTB = 0;
    double north, south, east, west;
    int carCt = 0;
    double tb = 0;
    bool findNextZone = true;
    double **zoneBalance = new double* [numZonesL];
    bool ablePush[numZonesL][numZonesL];
    bool ablePull[numZonesL][numZonesL];
    
    for (int x=0; x<numZonesL; x++)
	zoneBalance[x] = new double[numZonesL];

    // initialize zoneBalance
    for (xb = 0; xb < numZonesL; xb++)
    {
        for (yb = 0; yb < numZonesL; yb++)
        {
            // 1 extra car needed for each waiting trip
            zoneBalance[xb][yb] = - waitZonesT[xb][yb];
            ablePush[xb][yb] = true;
            ablePull[xb][yb] = true;
        }
    }

    // determine total number of free cars and number in each zone
    for (int xc = 0; xc < xMax; xc++)
    {
        xb = xc / zoneSize;
        for (int yc = 0; yc < yMax; yc++)
        {
            yb = yc / zoneSize;
            for (int c = 0; c < CarMx[xc][yc].size(); c++)
            {
                if (CarMx[xc][yc][c].inUse == false)
                {
			if (canRefuelAnywhere || (CarMx[xc][yc][c].gas > getCarTrav(xc,yc,85) + 8)){
	                	carCt++;
                		zoneBalance[xb][yb]++;
			}
			else {
				CarMx[xc][yc][c].refuel = refuelTime;
				CarMx[xc][yc][c].needFuel = true;
				CarMx[xc][yc][c].inUse = true;
			}
                }
            }
        }
    }

    // determine zone balance
    for (int xz = 0; xz < numZonesL; xz++)
    {
        for (int yz = 0; yz < numZonesL; yz++)
        {
            zoneBalance[xz][yz] = zoneBalance[xz][yz] - (zoneShares[xz][yz] * carCt);
        }
    }

    if (reportProcs)
    {
        for (int yz = numZonesL - 1; yz >=0; yz--)
        {
            for (int xz = 0; xz < numZonesL; xz++)
            {
                cout << int(zoneBalance[xz][yz]) << "   ";
            }
            cout << endl;
        }
        cout << endl;
    }

    if (carCt / (250) > 5)
    {
        cutoff = carCt / (250);
    } else {
        cutoff = 5;
    }

    while (findNextZone)
    {

        // find zone with greatest movement potential that hasn't been balanced already & has a discrepancy of > 5 cars
        for (int xz = numZonesL - 1; xz >= 0; xz--)
        {
            for (int yz = numZonesL - 1; yz >= 0; yz--)
            {
                absZB = sqrt(zoneBalance [xz][yz] * zoneBalance [xz][yz]);
                if (absZB > absTB && absZB > cutoff)
                {
                    if (zoneBalance[xz][yz] > 0 && ablePush[xz][yz])
                    {
                        tx = xz;
                        ty = yz;
                        tb = zoneBalance [xz][yz];
                        absTB = sqrt(tb*tb);
                    } else if (zoneBalance[xz][yz] < 0 && ablePull[xz][yz]) {
                        tx = xz;
                        ty = yz;
                        tb = zoneBalance [xz][yz];
                        absTB = sqrt(tb*tb);
                    }
                }
            }
        }

        if (zoneBalance[tx][ty] > 0)
        {
            ablePush[tx][ty] = false;
        } else {
            ablePull[tx][ty] = false;
        }

        //
        if (absTB > 0)
        {
            if (tb > 0)
            {
                pushSign = 1;
            } else {
                pushSign = -1;
            }

            if (ty + 1 < numZonesL)
            {
                north = zoneBalance [tx][ty+1];
            } else {
                north = 1000 * pushSign;
            }
            if (ty - 1 >= 0)
            {
                south = zoneBalance [tx][ty-1];
            } else {
                south = 1000 * pushSign;
            }
            if (tx + 1 < numZonesL)
            {
                east = zoneBalance [tx+1][ty];
            } else {
                east = 1000 * pushSign;
            }
            if (tx - 1 >= 0)
            {
                west = zoneBalance [tx-1][ty];
            } else {
                west = 1000 * pushSign;
            }

            if (zoneBalance[tx][ty] > 0)
            {
                pushCars( timeTripCounts, dwLookup, zoneBalance, tx, ty, north, south, west, east, t, reportProcs, totDist, unoccDist,
                          hotStarts, coldStarts, maxTrav, cardDirectLZ, numZones, zoneSize, trackX, trackY, trackC, iter);
            } else {
                pullCars( timeTripCounts, dwLookup, zoneBalance, tx, ty, north, south, west, east, t, reportProcs, totDist, unoccDist,
                          hotStarts, coldStarts, maxTrav, cardDirectLZ, numZones, zoneSize, trackX, trackY, trackC, iter);
            }
            tb = 0;
            absTB = 0;
        } else {
            findNextZone = false;
        }
    }

    if (reportProcs)
    {
        for (int y = numZonesL - 1; y >= 0; y--)
        {
            for (int x = 0; x < numZonesL; x++)
            {
                cout << int(zoneBalance[x][y]) << "   ";
            }
            cout << endl;
        }
    }

    return;
}
/*
// reallocates vehicles between zones. Excess cars -> positive zoneBalance, excess trips -> negative zoneBalance.
void Simulator::reallocVehsSZones (Car CarMx[][xMax][yMax], int numCars[xMax][yMax], Trip TTMx[][288], int* timeTripCounts, double dwLookup [][288],
                        double zoneSharesS[][10], int t, bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav)
{
    int xb, yb;
    int tx = 0;
    int ty = 0;
    int pushSign;
    int* cardDirectLZ;
    double cutoff;
    double absZB;
    double absTB = 0;
    double north, south, east, west;
    int carCt = 0;
    double tb = 0;
    bool findNextZone = true;
    double zoneBalance[10][10];
    bool ablePush[10][10];
    bool ablePull[10][10];

    // initialize zoneBalance
    for (xb = 0; xb < 10; xb++)
    {
        for (yb = 0; yb < 10; yb++)
        {
            // 1 extra car needed for each waiting trip
            zoneBalance[xb][yb] = 0;
            ablePush[xb][yb] = true;
            ablePull[xb][yb] = true;
        }
    }

    // determine total number of free cars and number in each zone
    for (int xc = 0; xc < xMax; xc++)
    {
        xb = xc / zoneSize;
        for (int yc = 0; yc < yMax; yc++)
        {
            yb = yc / zoneSize;
            for (int c = 0; c < numCars[xc][yc]; c++)
            {
//                if (CarMx[c][xc][yc].inUse == false && CarMx[c][xc][yc].moved == false)
                if (CarMx[c][xc][yc].inUse == false)
                {
                    carCt++;
                    zoneBalance[xb][yb]++;
                }
            }
        }
    }

    // determine zone balance
    for (int xz = 0; xz < 10; xz++)
    {
        for (int yz = 0; yz < 10; yz++)
        {
            zoneBalance[xz][yz] = zoneBalance[xz][yz] - (zoneSharesS[xz][yz] * carCt);
        }
    }

    if (reportProcs)
    {
        for (int yz = 9; yz >=0; yz--)
        {
            for (int xz = 0; xz < 10; xz++)
            {
                cout << zoneBalance[xz][yz] << "   ";
            }
            cout << endl;
        }
    }

    if (carCt / (250) > 5)
    {
        cutoff = carCt / (250);
    } else {
        cutoff = 5;
    }

    while (findNextZone)
    {

        // find zone with greatest movement potential that hasn't been balanced already & has a discrepancy of > 5 cars
        for (int xz = 9; xz >= 0; xz--)
        {
            for (int yz = 9; yz >= 0; yz--)
            {
                absZB = sqrt(zoneBalance [xz][yz] * zoneBalance [xz][yz]);
                if (absZB > absTB && absZB > cutoff)
                {
                    if (zoneBalance[xz][yz] > 0 && ablePush[xz][yz])
                    {
                        tx = xz;
                        ty = yz;
                        tb = zoneBalance [xz][yz];
                        absTB = sqrt(tb*tb);
                    } else if (zoneBalance[xz][yz] < 0 && ablePull[xz][yz]) {
                        tx = xz;
                        ty = yz;
                        tb = zoneBalance [xz][yz];
                        absTB = sqrt(tb*tb);
                    }
                }
            }
        }

        if (zoneBalance[tx][ty] > 0)
        {
            ablePush[tx][ty] = false;
        } else {
            ablePull[tx][ty] = false;
        }

        //
        if (absTB > 0)
        {
            if (tb > 0)
            {
                pushSign = 1;
            } else {
                pushSign = -1;
            }

            if (ty + 1 < 10)
            {
                north = zoneBalance [tx][ty+1];
            } else {
                north = 1000 * pushSign;
            }
            if (ty - 1 >= 0)
            {
                south = zoneBalance [tx][ty-1];
            } else {
                south = 1000 * pushSign;
            }
            if (tx + 1 < 10)
            {
                east = zoneBalance [tx+1][ty];
            } else {
                east = 1000 * pushSign;
            }
            if (tx - 1 >= 0)
            {
                west = zoneBalance [tx-1][ty];
            } else {
                west = 1000 * pushSign;
            }

            if (zoneBalance[tx][ty] > 0)
            {
                pushCars(CarMx, numCars, TTMx, timeTripCounts, dwLookup, zoneBalance, tx, ty, north, south, west, east, t, reportProcs, totDist, unoccDist,
                          hotStarts, coldStarts, maxTrav, cardDirectLZ);
            } else {
                pullCars(CarMx, numCars, TTMx, timeTripCounts, dwLookup, zoneBalance, tx, ty, north, south, west, east, t, reportProcs, totDist, unoccDist,
                          hotStarts, coldStarts, maxTrav, cardDirectLZ);
            }
            tb = 0;
            absTB = 0;
        } else {
            findNextZone = false;
        }
    }

    if (reportProcs)
    {
        for (int y = 9; y >= 0; y--)
        {
            for (int x = 0; x < 10; x++)
            {
                cout << zoneBalance[x][y] << " ";
            }
            cout << endl;
        }
    }

    return;
}

*/

// Function called from both reallocVehs2 *******************************************************************

// returns the number of cars surrounding a given point, with rank adjusted to give centralization preference
double Simulator::getSurroundCars (int x, int y)
{
    double rank = 0;
    double central;
    double north = 1000;
    double south = 1000;
    double east = 1000;
    double west = 1000;

    if (x >= xMax || x < 0 || y >= yMax || y < 0)
    {
        // off the grid
        rank = 1000;
    } else if (x + 1 < xMax && x - 1 >= 0 && y + 1 < yMax && y - 1 >= 0){
        // internal point
        central = findFreeCars (x, y);
        north = findFreeCars (x, y);
        south = findFreeCars (x, y);
        east = findFreeCars (x, y);
        west = findFreeCars (x, y);

        rank = (central + north + south + east + west) * 2;
        if (rank == 0)
        {
            rank = sqrt(pow(((xMax - 1) / 2) - x, 2) + pow(((yMax - 1) / 2) - y, 2)) / 1000;
        }
    } else {
        // outer edge point
        central = CarMx[x][y].size();
        if (y + 1 < yMax)
        {
            north = findFreeCars (x, y + 1);
        }
        if (y - 1 >= 0)
        {
            south = findFreeCars (x, y - 1);
        }
        if (x + 1 < xMax)
        {
            east = findFreeCars (x + 1, y);
        }
        if (x - 1 >= 0)
        {
            west = findFreeCars (x - 1, y);
        }
        rank = (north + south + west + east) * 2;

        if (rank == 0)
        {
            rank = sqrt(pow(((xMax - 1) / 2) - x, 2) + pow(((yMax - 1) / 2) - y, 2)) / 1000;
        }

    }

    return rank;
}

// returns the number of cars at a given location x,y that are not in use
double Simulator::findFreeCars (int x, int y)
{
    double numCars = 0;

    for (int c = 0; c < CarMx[x][y].size(); c++)
    {
        if (CarMx[x][y][c].inUse == false && (canRefuelAnywhere || CarMx[x][y][c].gas > getCarTrav(x,y,85) + 8))
        {
            numCars++;
        }
    }

    return numCars;
}


// Function called from both reallocVehsLZones *******************************************************************

// balances cars in zone tx, ty by pushing excess cars to adjacent zone areas
void Simulator::pushCars(int* timeTripCounts, double dwLookup [][288],
              double **zoneBalance, int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC, int iter)
{
    int moveN = 0;
    int moveS = 0;
    int moveW = 0;
    int moveE = 0;
    int nFreeCars = 0;
    bool stillMoving = true;
    int freeCarMap[xMax][yMax];
    int dist, direct, origX, origY;
    int cn = -1;
    int cardinalDirections[4];

    // initialize freeCarMap
    for (int x = 0; x < zoneSize; x++)
    {
        for (int y = 0; y < zoneSize; y++)
        {
            freeCarMap[x][y] = 0;
        }
    }

    // get number of free cars in zone tx, ty
    for (int x = tx * zoneSize; x < (tx + 1) * zoneSize; x++)
    {
        for (int y = ty * zoneSize; y < (ty + 1) * zoneSize; y++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false && (canRefuelAnywhere || CarMx[x][y][c].gas > getCarTrav(x,y,85) + 8))
                {
                    nFreeCars = nFreeCars + 1;
                    freeCarMap[x - (tx * zoneSize)][y - (ty * zoneSize)]++;
                }
            }
        }
    }

    if (reportProcs)
    {
        cout << "Pushing cars from zone " << tx << "," << ty << " zone balance = " << zoneBalance[tx][ty] << " free cars = " << nFreeCars << endl;
    }

    randOrdering(cardinalDirections, 4);

    // assign balancing directions
    while (nFreeCars > 0)
    {
        for (int i = 0; i < 4; i++)
        {
            switch (cardinalDirections[i])
            {
                case 0:
                    // north
                    north = north - 0.0001;
                    if (north < zoneBalance[tx][ty] - 2 && north < south && north < west && north < east && nFreeCars > 0)
                    {
                        moveN++;
                        north++;
                        cardDirectLZ[0]++;
                        cardDirectLZ[1]++;
                        nFreeCars--;
                        zoneBalance[tx][ty]--;
                        zoneBalance[tx][ty+1]++;
                    }
                    north = north + 0.0001;
                    break;
                case 1:
                    // south
                    south = south - 0.0001;
                    if (south < zoneBalance[tx][ty] - 2 && south < north && south < west && south < east && nFreeCars > 0)
                    {
                        moveS++;
                        south++;
                        cardDirectLZ[0]++;
                        cardDirectLZ[2]++;
                        nFreeCars--;
                        zoneBalance[tx][ty]--;
                        zoneBalance[tx][ty-1]++;
                    }
                    south = south + 0.0001;
                    break;
                case 2:
                    // west;
                    west = west - 0.0001;
                    if (west < zoneBalance[tx][ty] - 2 && west < south && west < north && west < east && nFreeCars > 0)
                    {
                        moveW++;
                        west++;
                        cardDirectLZ[0]++;
                        cardDirectLZ[3]++;
                        nFreeCars--;
                        zoneBalance[tx][ty]--;
                        zoneBalance[tx-1][ty]++;
                    }
                    west = west + 0.0001;
                    break;
                case 3:
                    // east;
                    east = east - 0.0001;
                    if (east < zoneBalance[tx][ty] - 2 && east < south && east < west && east < north && nFreeCars > 0)
                    {
                        moveE++;
                        east++;
                        cardDirectLZ[0]++;
                        cardDirectLZ[4]++;
                        nFreeCars--;
                        zoneBalance[tx][ty]--;
                        zoneBalance[tx+1][ty]++;
                    }
                    east = east + 0.0001;
                    break;
                default:
                    cout << "Error in assigning directions!";
            }
            if (zoneBalance[tx][ty] - 2 < north && zoneBalance[tx][ty] - 2 < south && zoneBalance[tx][ty] - 2 < west && zoneBalance[tx][ty] - 2 < east)
            {
                nFreeCars = 0;
            }
        }
    }

    if (reportProcs)
    {
        cout << "Pushing " << moveN << " cars north, " << moveS << " cars south, " << moveW << " cars west, " << moveE << " cars east" << endl;
    }


    // every other cycle, switch starting corner
    if (t % 2 > 0)
    {
        // start with northwest corner
        for (int lay = 0; lay < zoneSize && stillMoving && lay < maxTrav; lay++)
        {
            // north
            for (int ix = 0; ix < zoneSize && moveN > 0; ix++)
            {
                if (freeCarMap[ix][zoneSize - lay - 1] > 0)
                {
                    // find the location to move to
                    origX = (tx * zoneSize) + ix;
                    origY = ((ty + 1) * zoneSize) - 1 - lay;
                    direct = 1; // direction = 1 (north)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveN > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8)) // check here for charge dist?
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY + dist;
                            move ( origX, origY, origX, origY + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC, iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveN--;
                            freeCarMap[ix][zoneSize - lay - 1]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }

            // east
            for (int iy = zoneSize - 1; iy >= 0 && moveE > 0; iy--)
            {
                if (moveE > 0 && freeCarMap[zoneSize - lay - 1][iy] > 0)
                {
                    // find the location to move to
                    origX = ((tx + 1) * zoneSize) - 1 - lay;
                    origY = (ty * zoneSize) + iy;
                    direct = 4; // direction = 4 (east)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveE > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8))
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, zoneSize);
                            CarMx[origX][origY][c].destX = origX + dist;
                            CarMx[origX][origY][c].destY = origY;
                            move (  origX, origY, origX + dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC,iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveE--;
                            freeCarMap[zoneSize - lay - 1][iy]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }

            // south
            for (int ix = zoneSize - 1; ix >= 0 && moveS > 0; ix--)
            {
                if (freeCarMap[ix][lay] > 0)
                {
                    // find the location to move to
                    origX = (tx * zoneSize) + ix;
                    origY = (ty * zoneSize) + lay;
                    direct = 2; // direction = 2 (south)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveS > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8))
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav,  zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY - dist;
                            move (origX, origY, origX, origY - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC, iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveS--;
                            freeCarMap[ix][lay]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }

            // west
            for (int iy = 0; iy < zoneSize && moveW > 0; iy++)
            {
                if (freeCarMap[lay][iy] > 0)
                {
                    // find the location to move to
                    origX = (tx * zoneSize) + lay;
                    origY = (ty * zoneSize) + iy;
                    direct = 3; // direction = 3 (west)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveW > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8))
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav,   zoneSize);
                            CarMx[origX][origY][c].destX = origX - dist;
                            CarMx[origX][origY][c].destY = origY;
                            move ( origX, origY, origX - dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC,iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveW--;
                            freeCarMap[lay][iy]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }
        }

    } else {
        // start with southeast corner
        for (int lay = 0; lay < zoneSize && stillMoving && lay < maxTrav; lay++)
        {
            // south
            for (int ix = zoneSize - 1; ix >= 0 && moveS > 0; ix--)
            {
                if (freeCarMap[ix][lay] > 0)
                {
                    // find the location to move to
                    origX = (tx * zoneSize) + ix;
                    origY = (ty * zoneSize) + lay;
                    direct = 2; // direction = 2 (south)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveS > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8))
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav,   zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY - dist;
                            move (origX, origY, origX, origY - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC,iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveS--;
                            freeCarMap[ix][lay]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }

            // west
            for (int iy = 0; iy < zoneSize && moveW > 0; iy++)
            {
                if (freeCarMap[lay][iy] > 0)
                {
                    // find the location to move to
                    origX = (tx * zoneSize) + lay;
                    origY = (ty * zoneSize) + iy;
                    direct = 3; // direction = 3 (west)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveW > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8))
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav,   zoneSize);
                            CarMx[origX][origY][c].destX = origX - dist;
                            CarMx[origX][origY][c].destY = origY;
                            move ( origX, origY, origX - dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC,iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveW--;
                            freeCarMap[lay][iy]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }

            // north
            for (int ix = 0; ix < zoneSize && moveN > 0; ix++)
            {
                if (freeCarMap[ix][zoneSize - lay - 1] > 0)
                {
                    // find the location to move to
                    origX = (tx * zoneSize) + ix;
                    origY = ((ty + 1) * zoneSize) - 1 - lay;
                    direct = 1; // direction = 1 (north)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveN > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8))
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav,  zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY + dist;
                            move (origX, origY, origX, origY + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC,iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveN--;
                            freeCarMap[ix][zoneSize - lay - 1]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }

            // east
            for (int iy = zoneSize - 1; iy >= 0 && moveE > 0; iy--)
            {
                if (moveE > 0 && freeCarMap[zoneSize - lay - 1][iy] > 0)
                {
                    // find the location to move to
                    origX = ((tx + 1) * zoneSize) - 1 - lay;
                    origY = (ty * zoneSize) + iy;
                    direct = 4; // direction = 4 (east)

                    // find which car to move
                    for (int c = 0; c < CarMx[origX][origY].size() && moveE > 0; c++)
                    {
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false && (canRefuelAnywhere || CarMx[origX][origY][c].gas > getCarTrav(origX,origY,85)+8))
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav,   zoneSize);
                            CarMx[origX][origY][c].destX = origX + dist;
                            CarMx[origX][origY][c].destY = origY;
                            move ( origX, origY, origX + dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC,iter);
                            unoccDist = unoccDist + dist;
			    reallocDist += dist;
                            totDist = totDist + dist;
                            c--;
                            moveE--;
                            freeCarMap[zoneSize - lay - 1][iy]--;
                        }
                    }

                    if (moveN == 0 && moveS == 0 && moveW == 0 && moveE == 0)
                    {
                        stillMoving = false;
                    }
                }
            }
        }
    }

    if (reportProcs)
    {
        for (int y = numZones - 1; y >= 0; y--)
        {
            for (int x = 0; x < numZones; x++)
            {
                cout << int (zoneBalance[x][y]) << "   ";
            }
            cout << endl;
        }
        cout << endl;
    }

    return;
}

// balances cars in zone tx, ty by pulling excess cars from adjacent zone areas
void Simulator::pullCars(int* timeTripCounts, double dwLookup [][288],
              double **zoneBalance, int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC,int iter)
{
    int pullN = 0;
    int pullS = 0;
    int pullW = 0;
    int pullE = 0;
    int nFreeCarsN = 0;
    int nFreeCarsS = 0;
    int nFreeCarsW = 0;
    int nFreeCarsE = 0;
    int nFreeCarsTot;
    bool stillPulling = true;
    int dist, direct;
    int cn = -1;
    int cardinalDirections[4];

    if (reportProcs)
    {
        for (int y = numZones - 1; y >=0; y--)
        {
            for (int x = 0; x < numZones; x++)
            {
                cout << int(zoneBalance[x][y]) << " ";
            }
            cout << endl;
        }
    }

    // get number of free cars in adjacent zones

    // north
    if (ty < numZones - 1)
    {
        for (int x = tx * zoneSize; x < (tx + 1) * zoneSize; x++)
        {
            for (int y = (ty + 1) * zoneSize; y < (ty + 2) * zoneSize; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false && (canRefuelAnywhere || CarMx[x][y][c].gas > getCarTrav(x,y,85)+8))
                    {
                        nFreeCarsN = nFreeCarsN + 1;
                    }
                }
            }
        }
    } else {
        nFreeCarsN = 0;
        north = -1000;
    }

    // south
    if (ty > 0)
    {
        for (int x = tx * zoneSize; x < (tx + 1) * zoneSize; x++)
        {
            for (int y = (ty - 1) * zoneSize; y < ty * zoneSize; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false && (canRefuelAnywhere || CarMx[x][y][c].gas > getCarTrav(x,y,85)+8))
                    {
                        nFreeCarsS = nFreeCarsN + 1;
                    }
                }
            }
        }
    } else {
        nFreeCarsS = 0;
        south = -1000;
    }

    // west
    if (tx > 0)
    {
        for (int x = (tx - 1) * zoneSize; x < tx * zoneSize; x++)
        {
            for (int y = ty * zoneSize; y < (ty + 1) * zoneSize; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false && (canRefuelAnywhere || CarMx[x][y][c].gas > getCarTrav(x,y,85)+8))
                    {
                        nFreeCarsW = nFreeCarsN + 1;
                    }
                }
            }
        }
    } else {
        nFreeCarsW = 0;
        west = -1000;
    }

    // east
    if (tx < numZones - 1)
    {
        for (int x = (tx + 1) * zoneSize; x < (tx + 2) * zoneSize; x++)
        {
            for (int y = ty * zoneSize; y < (ty + 1) * zoneSize; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false && (canRefuelAnywhere || CarMx[x][y][c].gas > getCarTrav(x,y,85)+8))
                    {
                        nFreeCarsE = nFreeCarsN + 1;
                    }
                }
            }
        }
    } else {
        nFreeCarsE = 0;
        east = -1000;
    }

    nFreeCarsTot = nFreeCarsN + nFreeCarsS + nFreeCarsW + nFreeCarsE;

    if (reportProcs)
    {
        cout << "Pulling cars to zone " << tx << "," << ty << " zone balance = " << zoneBalance[tx][ty] << " adjacent free cars = " << nFreeCarsTot << endl;
    }

    randOrdering(cardinalDirections, 4);

    // assign balancing directions
    while (nFreeCarsTot > 0)
    {
        for (int i = 0; i < 4; i++)
        {
            switch (cardinalDirections[i])
            {
                case 0:
                    // north
                    north = north + 0.0001;
                    if (north > zoneBalance[tx][ty] + 2 && north > south && north > west && north > east && nFreeCarsN > 0)
                    {
                        pullN++;
                        north--;
                        cardDirectLZ[0]--;
                        cardDirectLZ[1]--;
                        nFreeCarsTot--;
                        nFreeCarsN--;
                        zoneBalance[tx][ty]++;
                        zoneBalance[tx][ty+1]--;
                    }
                    if (nFreeCarsN == 0)
                    {
                        north = -1000;
                    }
                    north = north - 0.0001;
                    break;
                case 1:
                    // south
                    south = south + 0.0001;
                    if (south > zoneBalance[tx][ty] + 2 && south > north && south > west && south > east && nFreeCarsS > 0)
                    {
                        pullS++;
                        south--;
                        cardDirectLZ[0]--;
                        cardDirectLZ[2]--;
                        nFreeCarsTot--;
                        nFreeCarsS--;
                        zoneBalance[tx][ty]++;
                        zoneBalance[tx][ty-1]--;
                    }
                    if (nFreeCarsS == 0)
                    {
                        south = -1000;
                    }
                    south = south - 0.0001;
                    break;
                case 2:
                    // west;
                    west = west + 0.0001;
                    if (west > zoneBalance[tx][ty] + 2 && west > south && west > north && west > east && nFreeCarsW > 0)
                    {
                        pullW++;
                        west--;
                        cardDirectLZ[0]--;
                        cardDirectLZ[3]--;
                        nFreeCarsTot--;
                        nFreeCarsW--;
                        zoneBalance[tx][ty]++;
                        zoneBalance[tx-1][ty]--;
                    }
                    if (nFreeCarsW == 0)
                    {
                        west = -1000;
                    }
                    west = west - 0.0001;
                    break;
                case 3:
                    // east;
                    east = east + 0.0001;
                    if (east > zoneBalance[tx][ty] + 2 && east > south && east > west && east > north && nFreeCarsE > 0)
                    {
                        pullE++;
                        east--;
                        cardDirectLZ[0]--;
                        cardDirectLZ[4]--;
                        nFreeCarsTot--;
                        nFreeCarsE--;
                        zoneBalance[tx][ty]++;
                        zoneBalance[tx+1][ty]--;
                    }
                    if (nFreeCarsE == 0)
                    {
                        east = -1000;
                    }
                    east = east - 0.0001;
                    break;
                default:
                    cout << "Error in assigning directions!";
            }

            if (zoneBalance[tx][ty] + 2 > north && zoneBalance[tx][ty] + 2 > south && zoneBalance[tx][ty] + 2 > west && zoneBalance[tx][ty] + 2 > east)
            {
                nFreeCarsTot = 0;
            }
        }
    }

    if (reportProcs)
    {
        for (int y = numZones - 1; y >=0; y--)
        {
            for (int x = 0; x < numZones; x++)
            {
                cout << int(zoneBalance[x][y]) << " ";
            }
            cout << endl;
        }
        cout << "Pulling " << pullN << " cars north, " << pullS << " cars south, " << pullW << " cars west, " << pullE << " cars east" << endl;
    }

    // every other cycle, switch pull direction
    if (t % 2 > 0)
    {
        for (int lay = 0; lay < zoneSize && stillPulling && lay < maxTrav; lay++)
        {
            for (int i = 0; i < 4; i++)
            {
                switch (cardinalDirections[i])
                {
                    case 0:
                    // pull from north
                        for (int ix = tx*zoneSize; ix < (tx + 1) * zoneSize && pullN > 0; ix++)
                        {
                            int iy = (ty + 1) * zoneSize + lay;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 2; // direction = 2 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy - dist;
                                    move ( ix, iy, ix, iy - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullN--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    case 1:
                    // pull from south
                        for (int ix = tx*zoneSize; ix < (tx + 1) * zoneSize && pullS > 0; ix++)
                        {
                            int iy = ty * zoneSize - lay - 1;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 1; // direction = 1 (move the vehicle north)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy + dist;
                                    move (ix, iy, ix, iy + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullS--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    case 2:
                    // pull from west
                        for (int iy = ty*zoneSize; iy < (ty + 1) * zoneSize && pullW > 0; iy++)
                        {
                            int ix = tx * zoneSize - lay - 1;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 4; // direction = 4 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix + dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move ( ix, iy, ix + dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullW--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    case 3:
                    // pull from east
                        for (int iy = ty*zoneSize; iy < (ty + 1) * zoneSize && pullE > 0; iy++)
                        {
                            int ix = (tx + 1) * zoneSize + lay;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 3; // direction = 3 (move the vehicle west)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix - dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move (ix, iy, ix - dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullE--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    default:
                        cout << "Error Pulling from zones!";
                        break;
                }
            }
        }
    } else {
        for (int lay = 0; lay < zoneSize && stillPulling && lay < maxTrav; lay++)
        {
            for (int i = 0; i < 4; i++)
            {
                switch (cardinalDirections[i])
                {
                    case 0:
                    // pull from north
                        for (int ix = (tx + 1) * zoneSize - 1; ix >= tx * zoneSize && pullN > 0; ix--)
                        {
                            int iy = (ty + 1) * zoneSize + lay;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 2; // direction = 2 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy - dist;
                                    move (ix, iy, ix, iy - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullN--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    case 1:
                    // pull from south
                        for (int ix = (tx + 1) * zoneSize - 1; ix >= tx * zoneSize && pullS > 0; ix--)
                        {
                            int iy = ty * zoneSize - lay - 1;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 1; // direction = 1 (move the vehicle north)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy + dist;
                                    move ( ix, iy, ix, iy + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullS--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    case 2:
                    // pull from west
                        for (int iy = (ty + 1) * zoneSize - 1; iy >= ty * zoneSize && pullW > 0; iy--)
                        {
                            int ix = tx * zoneSize - lay - 1;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 4; // direction = 4 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix + dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move (ix, iy, ix + dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullW--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    case 3:
                    // pull from east
                        for (int iy = (ty + 1) * zoneSize - 1; iy >= ty * zoneSize && pullE > 0; iy--)
                        {
                            int ix = (tx + 1) * zoneSize + lay;
                            for (int c = 0; c < CarMx[ix][iy].size(); c++)
                            {
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false && (canRefuelAnywhere || CarMx[ix][iy][c].gas > getCarTrav(ix,iy,85)+8))
                                {
                                    cn = c;
                                    direct = 3; // direction = 3 (move the vehicle west)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix - dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move ( ix, iy, ix - dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC,iter);
                                    unoccDist = unoccDist + dist;
				    reallocDist += dist;
                                    totDist = totDist + dist;
                                    c--;
                                    pullE--;
                                }
                            }
                        }

                        if (pullN == 0 && pullS == 0 && pullW == 0 && pullE == 0)
                        {
                            stillPulling = false;
                        }

                        break;

                    default:
                        cout << "Error Pulling from zones!";
                        break;
                }
            }
        }
    }

    return;
}

// determine the ideal distance to move
double Simulator::findMoveDist(int origX, int origY, int direct, int zoneEdge, int maxTrav, int zoneSize)
{

    int trueDist = getCarTrav(origX,origY,85);
    maxTrav = trueDist;

    double rank = 1000;
    double bestRank = 1000;
    double dist = zoneEdge;

    if (direct == 1) // north
    {
        for (int y = origY + zoneEdge; y <= origY + maxTrav && y <= origY + zoneEdge + zoneSize - 1; y++)
        {
            rank = getSurroundCars (origX, y) + (4 * CarMx[origX][y].size());
            if (rank < bestRank)
            {
                bestRank = rank;
                dist = y - origY;
            }
        }
    } else if (direct == 2) { // south
        for (int y = origY - zoneEdge; y >= origY - maxTrav && y >= origY - (zoneEdge + zoneSize - 1); y--)
        {
            rank = getSurroundCars (origX, y) + (4 * CarMx[origX][y].size());
            if (rank < bestRank)
            {
                bestRank = rank;
                dist = origY - y;
            }
        }
    } else if (direct == 3) { // west
        for (int x = origX - zoneEdge; x >= origX - maxTrav && x >= origX - (zoneEdge + zoneSize - 1); x--)
        {
            rank = getSurroundCars (x, origY) + (4 * CarMx[x][origY].size());
            if (rank < bestRank)
            {
                bestRank = rank;
                dist = origX - x;
            }
        }
    } else { // east
        for (int x = origX + zoneEdge; x <= origX + maxTrav && x <= origX + zoneEdge + zoneSize - 1; x++)
        {
            rank = getSurroundCars (x, origY) + (4 * CarMx[x][origY].size());
            if (rank < bestRank)
            {
                bestRank = rank;
                dist = x - origX;
            }
        }
    }

    return dist;
}

// Function called from both runSharedAV and placeInitCars *******************************************************************

//moves car from ox,oy to dx,dy
void Simulator::move (int ox, int oy, int dx, int dy, int c, int t, double dwLookup [][288],
           int* timeTripCounts, bool reportProcs, int& hotStart, int& coldStart, int& trackX, int& trackY, int& trackC, int iter)
{

    
    // identify target car in the car matrix the car will now have picked up passengers and will have moved by the end of this function
    Car tCar = CarMx[ox][oy][c];
    
    bool needFuel = false;
    if(tCar.needFuel)
	needFuel = true;
//    if(CHARGE_IN_PLACE && needFuel)
//	cout << "Should not be moving in charge_in_place" << endl;

    tCar.x = dx;
    tCar.y = dy;
//    tCar.pickupX = -1;
//    tCar.pickupY = -1;
    tCar.moved = true;
    tCar.inUse = true;

    // burn fuel driving
    tCar.gas = tCar.gas - abs (ox - dx) - abs (oy - dy);

//    if (tCar.gas < 0)
//	return;

    // update tracking info
//    if (ox == trackX && oy == trackY)
    if (reportProcs)
    {
        if (c == trackC)
        {
            cout << "Moving from " << ox << "," << oy << " to " << dx << "," << dy << endl;
            trackX = dx;
            trackY = dy;
            trackC = CarMx[dx][dy].size();
        } else if (c < trackC && trackC == CarMx[ox][oy].size()) {
            trackC = c;
        }
    }

    // insert the top car in the same x,y position into the gap left behind
    if (c != CarMx[ox][oy].size() - 1)
    {
        CarMx[ox][oy][c] = CarMx[ox][oy][CarMx[ox][oy].size() - 1];
    }
    CarMx[ox][oy].pop_back();
    

    if (dx == tCar.destX && dy == tCar.destY && tCar.pickupX == -1 && tCar.pickupY == -1)// && !needFuel)
    {
        // we have arrived at the target location, free the car
        if (reportProcs)
        {
            cout << "Arrived! Releasing car." << endl;
        }

	if (!tCar.needFuel)	
	        tCar.inUse = false;
	
        // generate a return trip, if slated
        if (tCar.returnHome == true)
        {
            if (!useCityTripData) {
              genRetTrip(dx, dy, tCar.retHX, tCar.retHY, t, dwLookup,  timeTripCounts);
	    }
            tCar.returnHome = false;
        }

        // determine if we need to refuel
        if (tCar.gas < refuelDist)//(tCar.numRejects >= 1) //tCar.gas < refuelDist)
        {
            tCar.refuel = refuelTime; // Changed to 48 from 2 (wait time is now 4 hours instead of 10 minutes)
            tCar.inUse = true;
	    tCar.needFuel = true;
            tCar.destX = dx;
            tCar.destY = dy;
            tCar.pickupX = -1;
            tCar.pickupY = -1;
            tCar.currTrip = NULL;

	    //tCar.currTrip = NULL;
	    //cellChargeCount[tCar.x][tCar.y][iter]++;
        }
    }

    if (tCar.tElapsed > 12)
    {
        coldStart++;
    } else if (tCar.tElapsed > 0) {
        hotStart++;
    }
    tCar.tElapsed = 0;
    CarMx[dx][dy].push_back(tCar);

    return;

}

int Simulator::getCarTrav(int x, int y, int t){

	float r = sqrt(pow(x - (xMax / 2),2) + pow (y - (yMax / 2),2));
	int trav;

	// assume congested speeds between 7-8 AM and 4-6:30 PM
        if (((t >= 84) && (t <= 96)) || ((t >= 192) && (t <= 222))){
        	if (r < innerDist)
			trav = 5;
		else if (r < nearDist)
			trav = 8;
		else if (r < outerDist)
			trav = 10;
		else
			trav = 11;
        } else {
        	 if (r < innerDist)
                        trav = 5;
                else if (r < nearDist)
                        trav = 8;
                else if (r < outerDist)
                        trav = 11;
                else
                        trav = 12;
        }

	return trav;

}

void Simulator::genRetTrip (int ox, int oy, int dx, int dy, int t, double dwLookup [][288], int* timeTripCounts)
{

    double randDw;
    int newT = -1;
    int dwI;
    Trip nTrip;

    randDw = rand();
    randDw = randDw / RAND_MAX;

    // matrix sorted by 3-hour increments
    dwI = t / 36;


    // find return trip time
    for (int i = 0; i < 288 && newT == -1; i++)
    {
        if (dwLookup[dwI][i] > randDw)
        {
            newT = i + t + 1;
        }
    }

    if (newT < 288 && newT != -1)
    {
        // add return trip
        nTrip.carlink = false;
        nTrip.endX = dx;
        nTrip.endY = dy;
        nTrip.startX = ox;
        nTrip.startY = oy;
        nTrip.startTime = newT;
        nTrip.returnHome = true;
        nTrip.waitTime = 0;
        nTrip.tripDist = double(abs(nTrip.startY - nTrip.endY) + abs(nTrip.startX - nTrip.endX)) / 4;
        nTrip.waitPtr = NULL;
	nTrip.price = 0.0;
//        cout << "Generating a return trip at T = " << newT << " from " << nTrip.startX << "," << nTrip.startY << " to " << nTrip.endX << "," << nTrip.endY << endl;

        placeInTTM ( nTrip, timeTripCounts, newT);
    }

    return;
}

void Simulator::findCOV (double& COVF, vector<double> COVray)
{
    double avgRay = 0;

    COVF = 0;

    for (int i = 0; i < COVray.size(); i++)
    {
        avgRay = avgRay + COVray[i];
    }

    avgRay = avgRay / COVray.size();

    for (int i = 0; i < COVray.size(); i++)
    {
        COVF = COVF + pow(COVray[i] - avgRay, 2);
    }

    // coefficient of variation, std. deviation divided by the mean
    COVF = sqrt(COVF / COVray.size()) / avgRay;
}


// writes time trip counts to output file
void Simulator::writeTimeTripCounts(ofstream& outfile, int* timeTripCounts)
{
    for (int t = 0; t < 288; t++)
    {
        outfile << timeTripCounts[t] << endl;
    }

    return;
}

// writes time trip matrix to output file
void Simulator::writeTimeTripMatrix(ofstream& outfile, int* timeTripCounts)
{
    for (int t = 0; t < 288; t++)
    {
        for (int trp = 0; trp < timeTripCounts[t]; trp++)
        {
            outfile << TTMx[t][trp].startTime << " ";
            outfile << TTMx[t][trp].startX << " ";
            outfile << TTMx[t][trp].startY << " ";
            outfile << TTMx[t][trp].endX << " ";
            outfile << TTMx[t][trp].endY << " ";
            outfile << TTMx[t][trp].carlink << " ";
        }
        outfile << endl;
    }
    return;
}

// writes num cars to output file
void Simulator::writeNumCars(ofstream& outfile)
{
    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            outfile << CarMx[x][y].size() << " ";
        }
        outfile << endl;
    }

    return;
}

void Simulator::writeNumFreeCars(ofstream& outfile)
{
    int numFC;

    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            numFC = 0;
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                if (CarMx[x][y][c].inUse == false)
                {
                    numFC ++;
                }
            }
            outfile << numFC << " ";
        }
        outfile << endl;
    }
    return;
}

void Simulator::writeMaxCarUse(double* maxCarUse, double* maxCarOcc)
{
    ofstream occOutFile, useOutFile;

    occOutFile.open("maxCarOcc.txt");

    for (int t = 0; t < 288; t++)
    {
        occOutFile << maxCarOcc[t] << endl;
    }

    occOutFile.close();

    useOutFile.open("maxCarUse.txt");


    for (int t = 0; t < 288; t++)
    {
        useOutFile << maxCarUse[t] << endl;
    }

    useOutFile.close();

    return;
}

void Simulator::writeChargeStats(int stat){

	ofstream avgfile, sdfile;
	float average[xMax][yMax];
	float sd[xMax][yMax];
	float zoneAverage[numZonesL][numZonesL];

	for (int x=0; x< xMax; x++){
		for (int y=0; y<yMax; y++){
			average[x][y] = 0.0;
			sd[x][y] = 0.0;
			zoneAverage[x / zoneSizeL][y / zoneSizeL] = 0.0;
		}
	}

	// Compute average
	for (int x=0; x<xMax; x++){

		for (int y=0; y<yMax; y++){

			for (int n=0; n < numRuns; n++){

				if (stat == 0){
					average[x][y] += (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].numCharges : 0;
					zoneAverage[x / zoneSizeL][y / zoneSizeL] += (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].numCharges : 0;
				}
				if (stat == 1){
					average[x][y] += (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].chargeTime : 0;
					zoneAverage[x / zoneSizeL][y / zoneSizeL] += (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].chargeTime : 0;
				}
				if (stat == 2){
					average[x][y] += (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].congestTime : 0;
					zoneAverage[x / zoneSizeL][y / zoneSizeL] += (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].congestTime : 0;
				}
			}

			average[x][y] /= numRuns;

		}

	}
	int cellValue;
	// Compute standard deviation
	for (int x=0; x < xMax; x++) {

		for (int y=0; y<yMax; y++) {

			for (int n=0; n<numRuns; n++){

				if (stat == 0){
					cellValue = (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].numCharges : 0;
					sd[x][y] += pow(cellValue - average[x][y],2);
				}
				if (stat == 1){
					cellValue = (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].chargeTime : 0;
					sd[x][y] += pow(cellValue - average[x][y],2);
				}
				if (stat == 2){
					cellValue = (ChStMx[x][y].size() > 0) ? ChStMx[x][y][0].congestTime : 0;
					sd[x][y] += pow(cellValue - average[x][y],2);
				}
			}
			
			sd[x][y] /= numRuns;
			sd[x][y] = sqrt(sd[x][y]);
		}

	}

	if (stat == 0){
		avgfile.open("cellChargeCountAvg.txt");
		sdfile.open("cellChargeCountSD.txt");
	}
	else if (stat == 1){
		avgfile.open("cellChargeTimeAvg.txt");
		sdfile.open("cellChargeTimeSD.txt");
	}
	else if (stat == 2){
		avgfile.open("cellCongestionAvg.txt");	
		sdfile.open("cellCongestionSD.txt");
	}

	for (int x=0; x< xMax; x++) {

		for (int y=0; y<yMax; y++) {

			avgfile << average[x][y] << " ";
			sdfile << sd[x][y] << " ";
		}
		avgfile << endl;
		sdfile << endl;
	}

	for (int x=0; x<numZonesL; x++) {
		for (int y=0; y<numZonesL; y++) {

			cout << zoneAverage[x][y] / numRuns << " ";

		}
		cout << endl;

	}

	avgfile.close();
	sdfile.close();

}


// writes car matrix to output file
void Simulator::writeCarMx(ofstream& outfile)
{
    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                outfile << CarMx[x][y][c].startX << " ";
                outfile << CarMx[x][y][c].startY << " ";
                outfile << CarMx[x][y][c].x << " ";
                outfile << CarMx[x][y][c].y << " ";
                outfile << CarMx[x][y][c].pickupX << " ";
                outfile << CarMx[x][y][c].pickupY << " ";
                outfile << CarMx[x][y][c].destX << " ";
                outfile << CarMx[x][y][c].destY << " ";
                outfile << CarMx[x][y][c].inUse << " ";
                outfile << CarMx[x][y][c].moved << " ";
            }
        }
        outfile << endl;
    }

    return;
}

/*// writes car matrix to output file
void Simulator::printZones(ofstream& outfile, int zones[xMax][yMax], int destZones[xMax][yMax])
{
    outfile << "Origin Zones:" << endl;

    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            outfile << zones[x][y] << " ";
        }
        outfile << endl;
    }

    outfile << endl << "Destination Zones:" << endl;

    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            outfile << destZones[x][y] << " ";
        }
        outfile << endl;
    }

    return;
}
*/

// reads time trip counts from a file created during a previous iteration
void Simulator::readTimeTripCounts(ifstream& infile, int* timeTripCounts)
{
    for (int t = 0; t < 288; t++)
    {
        infile >> timeTripCounts[t];
    }

    return;
}

// reads time trip matrix from a file created during a previous iteration
void Simulator::readTimeTripMatrix(ifstream& infile, int* timeTripCounts)
{
    int dummy;

    for (int t = 0; t < 288; t++)
    {
        for (int trp = 0; trp < timeTripCounts[t]; trp++)
        {
            infile >> TTMx[t][trp].startTime;
            infile >> TTMx[t][trp].startX;
            infile >> TTMx[t][trp].startY;
            infile >> TTMx[t][trp].endX;
            infile >> TTMx[t][trp].endY;
            infile >> TTMx[t][trp].carlink;

            if (TTMx[t][trp].startTime != t)
            {
                cout << "Read error, matrix time " << t << " read time " << TTMx[t][trp].startTime << endl;
                cin >> dummy;
            }
        }
    }

    return;
}

// reads num cars from a file created during a previous iteration
void Simulator::readNumCars(ifstream& infile)
{
	int a;
    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            infile >> a;//CarMx[x][y].size();
        }
    }
    return;
}

// reads car matrix from a file created during a previous iteration
void Simulator::readCarMx(ifstream& infile)
{
    int dummy;

    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                infile >> CarMx[x][y][c].startX;
                infile >> CarMx[x][y][c].startY;
                infile >> CarMx[x][y][c].x;
                infile >> CarMx[x][y][c].y;
                infile >> CarMx[x][y][c].pickupX;
                infile >> CarMx[x][y][c].pickupY;
                infile >> CarMx[x][y][c].destX;
                infile >> CarMx[x][y][c].destY;
                infile >> CarMx[x][y][c].inUse;
                infile >> CarMx[x][y][c].moved;

                if (CarMx[x][y][c].x != x || CarMx[x][y][c].y != y)
                {
                    cout << "Read error, car location " << x << "," << y << " read location " << CarMx[x][y][c].x << "," << CarMx[x][y][c].y << endl;
                    cin >> dummy;
                }

            }
        }
    }
    return;
}

void Simulator::showWaitCars(int t, vector<Trip> waitList [6], int* waitListI)
{
    ofstream outfile;
    char waitName [28];
    int firstDig, secDig, thirdDig;
    char firstDigCh, secDigCh, thirdDigCh;
    int zonesW[xMax][yMax];
    int zonesFC[xMax][yMax];


    // initialize zones
    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            zonesW[x][y] = 0;
            zonesFC[x][y] = 0;
        }
    }

    // generate waitlist by zone
    for (int w = 0; w < 6; w++)
    {
        for (int c = 0; c < waitListI[w]; c++)
        {
            zonesW[waitList[w][c].startX][waitList[w][c].startY]++;
        }
    }

    // determine free car locations
    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false)
                {
                    zonesFC[x][y]++;
                }
            }
        }
    }

    strcpy(waitName, "WaitXXX.txt\0");

    // generate the savefile name and create the new file
    thirdDig = t % 10;
    secDig = ((t - thirdDig) / 10) % 10;
    firstDig = (t - (secDig * 10) - thirdDig) / 100;
    firstDigCh = (char) (firstDig + 48);
    secDigCh = (char) (secDig + 48);
    thirdDigCh = (char) (thirdDig + 48);

    waitName[4] = firstDigCh;
    waitName[5] = secDigCh;
    waitName[6] = thirdDigCh;

    outfile.open(waitName);

    outfile << "T = " << t << endl << endl << "Waitlist" << endl;

    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            outfile << zonesW[x][y] << " ";
        }
        outfile << endl;
    }

    outfile << endl << "Free Cars" << endl;

    for (int y = 0; y < yMax; y++)
    {
        for (int x = 0; x < xMax; x++)
        {
            outfile << zonesFC[x][y] << " ";
        }
        outfile << endl;
    }

    outfile.close();


    return;
}


// Matching functions...conveniently at the end of the code
void Simulator::matchTripsToCarsGreedy(vector<Trip> &tripList, int time, int trav, bool reportProcs, int &nw, int &ne, int &se, int &sw, int &coldStarts, int &hotStarts, int run, bool checkStationDistance, bool warmstart)
{
//	cout << "using greedy match"<<endl;
	int trueTrav;
	double x;
	int tripsLeft = 1;

	if (!limitGreedySearch && !warmstart)
		trav = xMax;

        for (int d = 0; d < trav && tripsLeft > 0; d++)
        {
		tripsLeft = 0;
                if (time % 2 == 0){

                        for (int i = 0; i < tripList.size(); i++)
                        {
                                if (tripList[i].carlink == false && tripList[i].modeOfTransit == 2)
                                {
					if (carSpeed == VARY){
						trueTrav = getCarTrav(tripList[i].startX, tripList[i].startY, time);
						x = (1.0 * trueTrav) / (trav);
					}
					else
						x = 1;
					for (int j = (int)(d*x); j < (1+d)*x; j++)
	                                        findNearestCar(tripList[i],  j, trav,  reportProcs, nw, ne, se, sw, coldStarts, hotStarts, run, checkStationDistance);
                                }
				if (tripList[i].carlink == false && tripList[i].modeOfTransit == 2)
					tripsLeft++;
                        }

                }else {

                        for (int i = tripList.size() - 1; i >= 0; i--)
                        {
//				if(i==0 && time ==3)
//					cout<< " matching?"<<endl;
                                if (tripList[i].carlink == false && tripList[i].modeOfTransit == 2)
                                {
					if (carSpeed == VARY){
						trueTrav = getCarTrav(tripList[i].startX, tripList[i].startY, time);
						x = (1.0 * trueTrav) / (trav);
					} else
						x = 1;
					for (int j = (int)(d*x); j < (1+d)*x; j++)
	                                        findNearestCar(tripList[i],  j, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts, run, checkStationDistance);
		                }
				if (tripList[i].carlink == false && tripList[i].modeOfTransit == 2)
					tripsLeft++;
			
                        }

                }
        }
	
}

void Simulator::matchTripsToCarsDecentralized(vector<Trip> &tripList, int time, int trav, bool reportProcs, int &nw, int &ne, int &se, int &sw, int &coldStarts, int &hotStarts, int run, bool checkStationDistance, bool warmstart)
{
    vector<int> xvect;
    for (int x = 0; x < tripList.size(); x++)
    {
        xvect.push_back(x);
    }
    random_shuffle(xvect.begin(), xvect.end());
    for (int i=0; i < xvect.size(); i++) {
      
      if (tripList[i].carlink == false) {
        for (int d=0; d < xMax && tripList[i].carlink == false; d++) {
          findNearestCar(tripList[i], d, trav, reportProcs, nw, ne, se, sw, coldStarts, hotStarts, run, checkStationDistance);
        }
      }
    }	
}

void Simulator::matchTripsToCarsScram(vector<Trip> &tripList, int time, int trav, bool reportProcs, int &nw, int &ne, int &se, int &sw, int &coldStarts, int &hotStarts)
{
        Matching matching;

	if (matchAlgorithm == SCRAM)
		matching.setMinimizeMakespan(true);
	else 
		matching.setMinimizeMakespan(false);


        int trpX, trpY;
	vector<int> reassigned;
        vector<Car*> cars;
        Car curr;
        if (tripList.size() == 0){
	        return;
	}

        for (int trp=0; trp < tripList.size(); trp++)
                matching.addTrip(tripList[trp].startX, tripList[trp].startY, trp, -1, tripList[trp].tripDist);

        for (int x = 0; x < xMax; x++)
        {
                for (int y = 0; y < yMax; y++)
                {

                        for (int c=0; c < CarMx[x][y].size(); c++)
                        {

                                if (CarMx[x][y][c].inUse == false)
                                {

                                        curr = CarMx[x][y][c];
                                        int carIndex = cars.size();
                                        /*for (int i=0; i<cars.size(); i++)
                                        {
                                                if (*cars[i] == curr){
                                                        carIndex = i;
                                                        cout << "equal car"<<endl;
                                                        break;
                                                }
                                        }*/
                                        if (carIndex == cars.size())
                                                cars.push_back(&CarMx[x][y][c]);
                                        matching.addCar(x,y,carIndex,CarMx[x][y][c].gas);
                                } else { 
					// If car has not picked up add trip and car
					// Else do nothing
					
					if (reassignTrips && (CarMx[x][y][c].pickupX != -1 || CarMx[x][y][c].pickupY != -1) && CarMx[x][y][c].refuel == 0){
												
						int carIndex = cars.size();
						curr = CarMx[x][y][c];
						Trip *trip = curr.currTrip;
						cars.push_back(&CarMx[x][y][c]);
						matching.addCar(x,y,carIndex, CarMx[x][y][c].gas);
						int dist = abs(curr.x - trip->startX) + abs(curr.y - trip->startY);
						CarMx[x][y][c].inUse = false;
						CarMx[x][y][c].currTrip->carlink = false;
						tripList.push_back(*(CarMx[x][y][c].currTrip));
						tripList[tripList.size() - 1].waitPtr = CarMx[c][y][c].currTrip;
						matching.addTrip(trip->startX,trip->startY,tripList.size() - 1, dist, trip->tripDist);
						reassigned.push_back(tripList.size() - 1);
						//cout << "Trip readded to list" << endl;
						totTripsReassigned++;
						tripsReassigned++;
					}

				}
                        }
                }

        }
	//if (time == 71 && tripList.size() == 77)
	//	cout << "right call"<<endl;
//      cout << "Time of day: "<<time<<endl;
	//cout << "num cars: " << cars.size() <<" num trips: "<<tripList.size()<< endl;
        if (cars.size() == 0){
        //	if (time == 71 || time == 72)
	//		cout << "None matched cars: "<< time <<endl; 
	       return;
	}
//      cout << "call match func"<<endl;
        matching.setDimensions(tripList.size(),cars.size());
	clock_t t1,t2;
	t1 = clock();
        vector<Edge> result = matching.findMatching();
	t2 = clock();
	float diff = ((float)t2) - ((float)t1);
	float seconds = diff / CLOCKS_PER_SEC;
	//cout << "SCRAM took: " << seconds << endl;
        int carIndex,trip;
        double waitTrav;
	char a;
//      cout << "found match"<<endl;
        for (vector<Edge>::iterator it = result.begin(); it != result.end(); it++)
        {
                carIndex = (*it).second.first;
                trip = (*it).second.second;
                waitTrav = abs(cars[carIndex]->x - tripList[trip].startX) + abs(cars[carIndex]->y - tripList[trip].startY);

		if (waitTrav <= trav){

	                assignCar(cars[carIndex], &tripList[trip]);
			cars[carIndex]->currTrip = &tripList[trip];

                	tripList[trip].carlink = true;
                	//waitTrav = abs(cars[carIndex]->x - tripList[trip].startX) + abs(cars[carIndex]->y - tripList[trip].startY);
			if (waitTrav > trav)
				cout << "Exceeding max travel: "<<waitTrav <<" "<<trav<<endl;

	                tripList[trip].waitTime = tripList[trip].waitTime + (5.0 * waitTrav / trav);
        	        if (tripList[trip].waitPtr != NULL){
                	        tripList[trip].waitPtr -> waitTime = tripList[trip].waitTime;
				tripList[trip].waitPtr -> carlink = true;
				cars[carIndex]->currTrip = tripList[trip].waitPtr;
			}
		}
        }
//	for (int i=0; i<reassigned.size(); i++)
//	{
//		if ( tripList[reassigned[i]].carlink == false )
//			cout << "Trip lost car: " << reassigned[i]<<" Time: "<<time<<endl;
//	}
	
}

void Simulator::assignCar (Car* c, Trip* trp)
{
    double randRet;

    c->inUse = true;
    c->destX = trp->endX;
    c->destY = trp->endY;
    c->tripCt ++;

    if (trp->waitPtr != NULL)
      c->currTrip = trp->waitPtr;
    else
      c->currTrip = trp;


    if (c->x == trp->startX && c->y == trp->startY)
    {
        c->pickupX = -1;
        c->pickupY = -1;
    } else {
        c->pickupX = trp->startX;
        c->pickupY = trp->startY;
    }

    // for cars departing from home, assume 22% return by other means or another day and the rest will return
    if (trp->returnHome == false)
    {
        randRet = rand();
        randRet = randRet / RAND_MAX;

        if (randRet > 0.22)
        {
            c->returnHome = true;
	    c->retHX = trp->startX;
            c->retHY = trp->startY;
        }
    }

    // if for some reason we have an invalid destination, assign a default center location
    if (c->destX < 0 || c->destX >= xMax)
    {
        c->destX = xMax / 2;
    }

    if (c->destY < 0 || c->destY >= yMax)
    {
        c->destY = yMax / 2;
    }

    return;
}

void Simulator::reportMatchingResults()
{

    int nCars = 0;
    int nTrips = 0;
    int maxTripGen = 0;
    char dummyStr[20];

    double waitCOV = 0;
    double tripDistCOV = 0;
    double carTripsCOV = 0;
    double avgWait = 0;
    double avgDist = 0;
    double avgTrips = 0;

    double totWaitCount = 0;
    double maxCarUseT = 20000;
    double maxCarOccT = 20000;

    // determine num cars
    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            nCars = nCars + CarMx[x][y].size();
        }
    }
	
   // get coefficient of variation
    for (int t = 0; t < 288; t++)
    {

        nTrips = nTrips + TTMx[t].size();

        if (TTMx[t].size() > maxTripGen)
                maxTripGen = TTMx[t].size();

        for (int trp = 0; trp < TTMx[t].size(); trp++)
        {
            avgWait = avgWait + TTMx[t][trp].waitTime;
            avgDist = avgDist + TTMx[t][trp].tripDist;
        }
    }

/*
    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                avgTrips = avgTrips + CarMx[x][y][c].tripCt;
            }
        }
    }*/

    avgWait = avgWait;// / nTrips;
    avgDist = avgDist;// / nTrips;
//    avgTrips = avgTrips / nCars;


	cout << "Number of trips: " << nTrips << endl;
	cout << "Number of cars: " << nCars << endl;
	cout << "Unoccupied travel: " << unoccDist / 4.0<< endl;
        cout << "Relocation travel: " << reallocDist / 4.0 << endl;
	cout << "Total Travel: " << avgDist << endl;
	cout << "Total wait time: " << avgWait << endl;
	cout << "Unserved trips: " << unservedT << endl;
	cout << "Reassigned trips: " << tripsReassigned << endl;
	
	for (int i=0; i < 6; i++){
		cout << "Wait " << 5*(i+1) << " " << waitCount[i] << endl;
	}
}

// generates a new charging station at the location of car that needs it
Station* Simulator::genNewStation (Car car)
{

    if(ChStMx[car.x][car.y].size() > 0)
	return &ChStMx[car.x][car.y][0];
    Station nStat;

    nStat.x = car.x;
    nStat.y = car.y;
    nStat.startX = car.x;
    nStat.startY = car.y;
    nStat.numCharges = 0;
    nStat.chargeTime = 0;
    ChStMx[car.x][car.y].push_back(nStat);
    
    return &ChStMx[car.x][car.y][0];
}

//Finds the nearest available car to a given trip
void Simulator::findNearestStation (Car* car, int dist, int maxDist)
{
    int x,y;
    int carX = x = car->x;
    int carY = y = car->y;
    bool found = false;
    int d,r;
 
/*    if (car->gas == carRange){
	car->inUse = false;
	car->refuel = 0;
	car->needFuel = false;
	return;
    }*/

    if (dist == 0)
    {
        found = lookForStation (x, y);
        if (found)
        {
            assignStation(x, y, car);
        }
    } else {

        r = rand();
        r = r / RAND_MAX;

        if (r < 0.25) // start with the northwest quadrant
        {
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, carX - dist + d);
                y = min(yMax - 1, carY + d);
                found = lookForStation (x,y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, carX + d);
                y = min(yMax - 1, carY + dist - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, carX + dist - d);
                y = max(0, carY - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, carX - d);
                y = max(0, carY - dist + d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }

        } else if (r < 0.5) {// start with the northeast quadrant

            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, carX + d);
                y = min(yMax - 1, carY + dist - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, carX + dist - d);
                y = max(0, carY - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, carX - d);
                y = max(0, carY - dist + d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, carX - dist + d);
                y = min(yMax - 1, carY + d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }

        } else if (r < 0.75) { // start with the southeast quadrant

            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, carX + dist - d);
                y = max(0, carY - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, carX - d);
                y = max(0, carY - dist + d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, carX - dist + d);
                y = min(yMax - 1, carY + d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, carX + d);
                y = min(yMax - 1, carY + dist - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }

        } else { // start with the southwest quadrant

            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, carX - d);
                y = max(0, carY - dist + d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, carX - dist + d);
                y = min(yMax - 1, carY + d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, carX + d);
                y = min(yMax - 1, carY + dist - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, carX + dist - d);
                y = max(0, carY - d);
                found = lookForStation (x, y);
                if (found)
                {
                    assignStation(x, y, car);
                }
            }
        }
    }

//    if (found)
//    {
//	cout << "found station!!!!"<<endl;
//    }


    return;
}

void Simulator::assignStation (int x, int y, Car* car)
{
      float dist = 1.0 * abs(car->x - x) + abs(car->y - y);
      car->inUse = true; // Already set as true when refuel is changed so probably redundant
      car->destX = x;
      car->destY = y;
//      if (car->gas < 0)
//	cout << "Negative gas: " << car->gas<< endl;
      car->refuel = ceil(min((carRange - (car->gas - dist)) / carRange, 1.0f) * refuelTime);
      if (car->refuel > refuelTime)
  	    cout << "Percent: " << ((carRange - (car->gas - dist)) / carRange) << " gas left at station: " << car->gas - dist << endl;
//    cout << car->refuel << endl;
      car->pickupX = -1;
      car->pickupY = -1;
      car->stationLink = true;
      ChStMx[x][y][0].unoccupiedDist += dist;
      ChStMx[x][y][0].numCharges++;

    return;
}


//Looks for a free car at coordinates x,y
bool Simulator::lookForStation (int x, int y)
{
   
    bool found = false;

    if (ChStMx[x][y].size() > 0)
	return true;

    return found;
}

int Simulator::nearestStationDistance(int x, int y)
{

	if (lookForStation(x,y))
		return 0;

	int x1,y1;

	for(int dist=1; dist < xMax; dist++)
	{
		for (int d = 0; d < dist; d++)  //southwest quadrant
		{
                	x1 = max(0, x - d);
	                y1 = max(0, y - dist + d);
        	        if(lookForStation (x1, y1))
				return dist;
		}
		for (int d = 0; d < dist; d++)  //northwest quadrant
		{
                	x1 = max(0, x - dist + d);
	                y1 = min(yMax - 1, y + d);
        	        if(lookForStation (x1, y1))
				return dist;
		}
		for (int d = 0; d < dist; d++)  //northeast quadrant
		{
                	x1 = min(xMax - 1, x + d);
	                y1 = min(yMax - 1, y + dist - d);
        	        if(lookForStation (x1, y1))
				return dist;
		}
		for (int d = 0; d < dist; d++)  //southeast quadrant
		{
                	x1 = min(xMax - 1, x + dist - d);
	                y1 = max(0, y - d);
        	        if(lookForStation (x1, y1))
				return dist;
		}
	}
	return xMax * xMax; // return very large number if no station found
}

void Simulator::writeStationLocation()
{
	ofstream outfile;
 	outfile.open("stationLocation.txt");

	for(int x=0; x<xMax; x++){
		for(int y=0; y<yMax; y++){
			if (ChStMx[x][y].size() > 0)
				outfile << "1";
			else
				outfile << "0";
			if(y != yMax - 1)
				outfile << " ";
		}
		outfile << endl;
	}
	outfile.close();
}

void Simulator::writeHeatMap(int run, int time){

/*	ofstream outfile;
	stringstream stream;
	stream << "heatmaps/heatmap_160mi_r_" << run << "_t_" << time << ".csv";
	std::string file_name(stream.str());
	outfile.open(file_name.c_str());


	for(int x=0; x<xMax; x++){
		for(int y=0; y<yMax; y++){
			if(y > 0)
				outfile << ",";
			outfile << CarMx[x][y].size();
		}
		outfile << endl;
	}
	outfile.close();*/

	if (run == -1)
	{
		ofstream outfile2;
		outfile2.open("heatmaps/chargemap_400mi.csv");

		for(int x=0; x<xMax; x++){
			for(int y=0; y<yMax; y++){
				if (y > 0)
					outfile2 << ",";
				if(ChStMx[x][y].size() > 0)
					outfile2 << "1";
				else
					outfile2 << "0";
			}
			outfile2 << endl;
		}
		outfile2.close();
	}
}
