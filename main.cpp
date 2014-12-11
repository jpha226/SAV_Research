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

using namespace std;

struct Trip
{
    int startTime;
    int startX;
    int startY;
    int endX;
    int endY;
    bool carlink;
    bool returnHome;
    double waitTime;
    double tripDist;
    Trip* waitPtr;
};

struct Car
{
    int startX;
    int startY;
    int x;
    int y;
    int pickupX;
    int pickupY;
    int destX;
    int destY;
    bool inUse;
    bool moved;
    int tElapsed; // between moves, for vehicle starts evaluation (emissions)
    bool returnHome;
    int retHX;
    int retHY;
    int tripCt;
    int gas;
    int refuel;
};

const int xMax = 400; // 40
const int yMax = xMax; // 40
const int TTMxSize = 4000;
const int CarMxSize = 500;
const int WaitListSize = 4000;
const int numZonesL = 50;
const int numZonesS = 100;
const int zoneSizeL = xMax / numZonesL; // 8
const int zoneSizeS = xMax / numZonesS; // 4
const int maxNumRuns = 1005;
const double nearDist = 30;  // 10
const double innerDist = 10;
const int numWarmRuns = 20; // 20
const int tripDistSize = 601;
const bool largeSize = true;
const bool matchGreedy = true;
const int carRange = 320;

 long totDistRun, totUnoccDistRun, totCarsRun, totTripsRun, totHSRun, totCSRun, totWaitTRun, totUnservedTRun, totUnusedRun, totUnoccRun, maxAvailCars;
    long totWaitCountRun[6];
//    std::vector<Car>** CarMx;
   std::vector<Car> CarMx[xMax][yMax];
    //Car CarMx[CarMxSize][xMax][yMax];
    std::vector<Trip> TTMx[288];
    //Trip TTMx [TTMxSize][288];
    int timeTripCounts [288]; // array noting number of trips in each time bin
    //int numCars[xMax][yMax];
    double startTimes [288];
    double tripDist [tripDistSize];
    double dwLookup [8][288];
    double zoneSharesL [xMax][yMax]; // These are declared bigger than they need to be. 
    double zoneSharesS [xMax][yMax]; // This allows only one function to be used for them
					// Two functions this is done for reallocVehsZonesL which could be changed only S zones
					// Other is setZoneShares. This could be divided into two functions easily
// arrays for calculating COVs
    double totDistRunCOV [maxNumRuns]; // This does not seem to be accessed 
    double totUnoccDistRunCOV [maxNumRuns]; // This does not seem to be accessed
    double totCarsRunCOV [maxNumRuns]; // This does not seem to be accessed
    double totTripsRunCOV [maxNumRuns];
    double totHSRunCOV [maxNumRuns];
    double totCSRunCOV [maxNumRuns];
    double totWaitTRunCOV [maxNumRuns];
    double totUnservedTRunCOV [maxNumRuns];
    double totWaitCountRunCOV [maxNumRuns];
    double totUnusedRunCOV [maxNumRuns];
    double totUnoccRunCOV [maxNumRuns];
    double totAvgWaitCOV [maxNumRuns];
    double totAvgTripDistCOV [maxNumRuns];
    double totStartsPerTripCOV [maxNumRuns];
    double totAvgTripsPerCarCOV [maxNumRuns];

    double totPctMaxWaitFiveCOV [maxNumRuns];
    double totPctInducedTCOV [maxNumRuns];
    double totPctMaxInUseCOV [maxNumRuns];
    double totPctMaxOccCOV [maxNumRuns];
    double totPctColdShareCOV [maxNumRuns];

    bool error, reportProcs, readFile, wStart;
    int maxTrav, maxTravC, totDist, unoccDist, waitT, saveRate, startIter, unservedT, coldStarts, hotStarts, numRuns, nCars;
    double maxCarUse [288];
    double maxCarOcc [288];
    int waitCount[6];
    double outerRate, innerRate, nearRate, exurbanRate, distWt, netDistWt, totAvgWait, totAvgTripDist, totWaitCOV, totTripDistCOV, totCarTripsCOV;
    bool printZ = true;
    char zName [16];

//functions in main
void initVars (int runNum, bool warmStart);
void findDistWeight();

void generateTrips ();
void runSharedAV (int* timeTripCounts, std::vector<Car> CarMx[][yMax], int maxTrav, int maxTravC, double dwLookup [][288],
                  double zoneSharesL[][yMax], double zoneSharesS [][yMax], double* maxCarUse, double* maxCarOcc, int& totDist, int& unoccDist, int& waitT, bool reportProcs,
                  int saveRate, bool warmStart, bool lastWarm, long& maxAvailCars, bool& readFile, int startIter, int& unservedT, int* waitCount, int& hotStarts,
                  int& coldStarts, int nRuns);
void placeInitCars (std::vector<Car> CarMx[][yMax], int* timeTripCounts, double* maxCarUse, double* maxCarOcc, int& totDist,
                    int& unoccDist, int& waitT, double dwLookup [][288], bool reportProcs, int& hotStarts, int& coldStarts);
void reportResults (int* timeTripCounts, std::vector<Car> CarMx[][yMax],  double* maxCarUse, double* maxCarOcc, int totDist,
                    int unoccDist, int waitT, int unservedT, int* waitCount, int hotStarts, int coldStarts, long& totDistRun, long& totUnoccDistRun,
                    long& totCarsRun, long& totTripsRun, long& totHSRun, long& totCSRun, long& totWaitTRun, long& totUnservedTRun, long* totWaitCountRun,
                    long& totUnusedRun, long& totUnoccRun, double& totAvgWait, double& totAvgTripDist, double* totDistRunCOV, double* totUnoccDistRunCOV,
                    double* totCarsRunCOV, double* totTripsRunCOV, double* totHSRunCOV, double* totCSRunCOV, double* totWaitTRunCOV, double* totUnservedTRunCOV,
                    double* totWaitCountRunCOV, double* totUnusedRunCOV, double* totUnoccRunCOV, double* totAvgWaitCOV, double* totAvgTripDistCOV,
                    double* totStartsPerTripCOV, double* totAvgTripsPerCarCOV, double& totWaitCOV, double& totTripDistCOV, double& totCarTripsCOV,
                    double* totPctMaxWaitFiveCOV, double* totPctInducedTCOV, double* totPctMaxInUseCOV, double* totPctMaxOccCOV, double* totPctColdShareCOV,
                    int nRuns, int runNum);
void reportFinalResults (long totDistRun, long totUnoccDistRun, long totCarsRun, long totTripsRun, long totHSRun, long totCSRun, long totWaitTRun,
                         long totUnservedTRun, long* totWaitCountRun, long totUnusedRun, long totUnoccRun, double totAvgWait, double totAvgTripDist,
                         double* totDistRunCOV, double* totUnoccDistRunCOV, double* totCarsRunCOV, double* totTripsRunCOV, double* totHSRunCOV,
                         double* totCSRunCOV, double* totWaitTRunCOV, double* totUnservedTRunCOV, double* totWaitCountRunCOV, double* totUnusedRunCOV,
                         double* totUnoccRunCOV, double* totAvgWaitCOV, double* totAvgTripDistCOV,  double* totStartsPerTripCOV, double* totAvgTripsPerCarCOV,
                         double totWaitCOV, double totTripDistCOV, double totCarTripsCOV, double* totPctMaxWaitFiveCOV, double* totPctInducedTCOV,
                         double* totPctMaxInUseCOV, double* totPctMaxOccCOV, double* totPctColdShareCOV, int numRuns);

//functions called in initVars
void setStartTimes(double startTimes [288]);
void setTripDist(double tripDist [tripDistSize]);
void setTripDistLarge(double tripDist [tripDistSize]);
void setDwTimes (double dwLookup [][288]);
void setZoneShares (double zoneShares[][yMax], double outerRate, double innerRate, double nearRate, double exurbanRate, int numZones, int zoneSize);

void restoreStatus(int* timeTripCounts, std::vector<Car> CarMx[][yMax], int maxTrav,  double* maxCarUse, int& totDist,
                   int& unoccDist, bool reportProcs, int& saveRate, bool& warmStart, int& tCount, char fileName[], bool& error, bool& wStart, int& startIter);

//functions called in findDistWeight
int countDiff (double outerRate, double innerRate, double nearRate, double exurbanRate, double* startTimes, double* tripDist, int* timeTripCounts, double distWt, bool reportProcs);

//functions called in generateTrips
double getRate (int x, int y, double r, double xCent, double yCent, double outerRate, double innerRate, double nearRate, double exurbanRate);
int getStartTime(double* StartTimes);
int genPoisson (double mean);
void getDest (int x, int y, int &newX, int &newY, double* tripDist, double dWt);
int getDestDist(double* tripDist);
void placeInTTM (Trip ntrip, int* timeTripCounts, int time);

//functions called in runSharedAV
void saveStatus(int* timeTripCounts, std::vector<Car> CarMx[][yMax], int maxTrav,  double* maxCarUse, int totDist,
                 int unoccDist, bool reportProcs, int& saveRate, bool warmStart, int tCount);

void findNearestCar (Trip& trp, std::vector<Car> CarMx[][yMax], int dist, int maxDist,  bool reportProcs,
                     int& nw, int& ne, int& se, int& sw, int& coldStart, int& hotStart);
bool lookForCar (int x, int y, int dist, int& cn, std::vector<Car> CarMx[][yMax]);
void assignCar (int x, int y, int c, std::vector<Car> CarMx[][yMax], Trip trp);
Car genNewCar (Trip trp);
void moveCar (std::vector<Car> CarMx[][yMax], int x, int y, int c, int t, int maxTrav, int& totDist, int& unoccDist, int& waitT,
              double dwLookup [][288], int* timeTripCounts, bool reportProcs, int& hotStarts, int& coldStarts, int& trackX, int& trackY, int& trackC);
void move (std::vector<Car> CarMx[][yMax], int ox, int oy, int dx, int dy, int c, int t, double dwLookup [][288], int* timeTripCounts,
           bool reportProcs, int& hotStart, int& coldStart, int& trackX, int& trackY, int& trackC);
void genRetTrip (int ox, int oy, int dx, int dy, int t, double dwLookup [][288],  int* timeTripCounts);
void randOrdering(int* xRandOrd, int numVals);
void runSummary(bool warmStart, bool lastWarm, int zoneGen[][numZonesL], int waitZones[][numZonesL], double netZoneBalance[][numZonesL], int tripO[][numZonesL], int tripD[][numZonesL],
                int* cardDirectLZ, int* cardDirect, int* cardDirect2,  int* timeTripCounts);
void reallocVehs(int x, int y, std::vector<Car> CarMx[][yMax],  int* timeTripCounts, double dwLookup [][288],
                 bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect, int& trackX, int& trackY, int& trackC);
void reallocVehs2(int x, int y, std::vector<Car> CarMx[][yMax],  int* timeTripCounts, double dwLookup [][288],
                  bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect2, int& trackX, int& trackY, int& trackC);
void reallocVehsLZones (std::vector<Car> CarMx[][yMax],  int* timeTripCounts, double dwLookup [][288],
                        double zoneShares[][yMax], int t, bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav,
                        double netZoneBalance[][numZonesL], int* cardDirectLZ, int waitZonesT[][numZonesS], int numZones, int zoneSize, int& trackX, int& trackY, int& trackC);

// function called from reallocVehs2, pushCars & pullCars
double getSurroundCars (int x, int y, std::vector<Car> CarMx[][yMax]);
double findFreeCars (int x, int y, std::vector<Car> CarMx[][yMax]);

// function called from reallocVehsLZones
void pushCars(std::vector<Car> CarMx[][yMax], int* timeTripCounts, double dwLookup [][288],
              double zoneBalance[][yMax], int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC);
void pullCars(std::vector<Car> CarMx[][yMax], int* timeTripCounts, double dwLookup [][288],
              double zoneBalance[][yMax], int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC);
double findMoveDist(int origX, int origY, int direct, int zoneEdge, int maxTrav, std::vector<Car> CarMx[][yMax],  int zoneSize);


//functions called in reportFinalResults
void findCOV (double& COVF, double* COVray, int numRuns);

//File I/O called from saveStatus and restoreStatus
void writeTimeTripCounts(ofstream& outfile, int* timeTripCounts);
void writeTimeTripMatrix(ofstream& outfile, int* timeTripCounts);
void writeNumCars(ofstream& outfile);
void writeCarMx(ofstream& outfile,  std::vector<Car> CarMx[][yMax]);
void writeNumFreeCars(ofstream& outfile,  std::vector<Car> CarMx[][yMax]);
void writeMaxCarUse(double* maxCarUse, double* maxCarOcc);

void readTimeTripCounts(ifstream& infile, int* timeTripCounts);
void readTimeTripMatrix(ifstream& infile, int* timeTripCounts);
void readNumCars(ifstream& infile);
void readCarMx(ifstream& infile,  std::vector<Car> CarMx[][yMax]);

void printZones(ofstream& outfile, int zones[xMax][yMax], int destZones[xMax][yMax]);
void showWaitCars(int t, vector<Trip> waitList [6], int* waitListI, std::vector<Car> CarMx[][yMax]);


// Begin Program **************************************************************************************************

int main()
{
//	cout << "main begins"<<endl;
  
	    
strcpy(zName, "Warm Zones.txt");
clock_t t1,t2;
float time_diff, seconds;
    // warm start, get # of vehs
    for (int i = 1; i <= numWarmRuns; i++)
    {
	
	initVars(i,true);
       
	findDistWeight();
	
        if (!error)
        {

            if (!readFile)
            {
                generateTrips ();
            }
            if (!readFile || wStart) // run sharedAV if not restoring a previous run or if continuing a previous run still on warm start
            {
		cout << "About to run program"<<endl;
              t1 = clock();
		  runSharedAV (timeTripCounts, CarMx, maxTrav, maxTravC,  dwLookup, zoneSharesL, zoneSharesS, maxCarUse, maxCarOcc, totDist, unoccDist,
                             waitT, reportProcs, saveRate, true, false, maxAvailCars, readFile, startIter, unservedT, waitCount, hotStarts, coldStarts, numRuns);
         
		placeInitCars (CarMx,   timeTripCounts, maxCarUse, maxCarOcc, totDist, unoccDist, waitT, dwLookup, reportProcs, hotStarts, coldStarts);
		t2 = clock();
            }
		time_diff = ((float)t2 - (float)t1);
		seconds = time_diff / CLOCKS_PER_SEC;
            cout << "Warm Run " << i << " completed. Time: " << seconds << endl;
        }
    }

    // now we have the right # of vehicles
    distWt = netDistWt / numWarmRuns;
    maxAvailCars = maxAvailCars / numWarmRuns;
    nCars = 0;

    for (int i = 2; i < 100 && nCars < maxAvailCars; i++)
    {
        initVars (i,true);
        cout << "Max avail cars is " << maxAvailCars << endl << endl;

        cout << "Possible last warm start before running scenario" << endl;

        generateTrips ();
        runSharedAV ( timeTripCounts, CarMx, maxTrav, maxTravC,  dwLookup, zoneSharesL, zoneSharesS, maxCarUse, maxCarOcc, totDist, unoccDist,
                     waitT, reportProcs, saveRate, true, true, maxAvailCars, readFile, startIter, unservedT, waitCount, hotStarts, coldStarts, numRuns);
        placeInitCars (CarMx,   timeTripCounts, maxCarUse, maxCarOcc, totDist, unoccDist, waitT, dwLookup, reportProcs, hotStarts, coldStarts);

        nCars = 0;
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                nCars = nCars + CarMx[x][y].size();
            }
        }

    }

    cout << "nCars is " << nCars << endl;

    strcpy(zName, "Zones.txt");

    // run the program
    for (int i = 1; i <= numRuns; i++)
    {
	initVars(i,false);
        placeInitCars (CarMx,   timeTripCounts, maxCarUse, maxCarOcc, totDist, unoccDist, waitT, dwLookup, reportProcs, hotStarts, coldStarts);
        generateTrips ();
	t1 = clock();
        runSharedAV ( timeTripCounts, CarMx, maxTrav, maxTravC,  dwLookup, zoneSharesL, zoneSharesS, maxCarUse, maxCarOcc, totDist, unoccDist, waitT,
                             reportProcs, saveRate, false, false, maxAvailCars, readFile, startIter, unservedT, waitCount, hotStarts, coldStarts, numRuns);
	t2 = clock();
	time_diff = ((float)t2 - (float)t1);
	seconds = time_diff / CLOCKS_PER_SEC;
        reportResults ( timeTripCounts, CarMx,  maxCarUse, maxCarOcc, totDist, unoccDist, waitT, unservedT, waitCount, hotStarts, coldStarts,
                       totDistRun, totUnoccDistRun, totCarsRun, totTripsRun, totHSRun, totCSRun, totWaitTRun, totUnservedTRun, totWaitCountRun,
                       totUnusedRun, totUnoccRun, totAvgWait, totAvgTripDist, totDistRunCOV, totUnoccDistRunCOV, totCarsRunCOV, totTripsRunCOV,
                       totHSRunCOV, totCSRunCOV, totWaitTRunCOV, totUnservedTRunCOV, totWaitCountRunCOV, totUnusedRunCOV, totUnoccRunCOV,
                       totAvgWaitCOV, totAvgTripDistCOV, totStartsPerTripCOV, totAvgTripsPerCarCOV, totWaitCOV, totTripDistCOV, totCarTripsCOV,
                       totPctMaxWaitFiveCOV, totPctInducedTCOV, totPctMaxInUseCOV, totPctMaxOccCOV, totPctColdShareCOV, numRuns, i);

        if (numRuns > 1)
        {
            cout << "Run " << i << " completed. Time: " << seconds << endl;
        }
    }

    if (numRuns > 1)
    {
        reportFinalResults(totDistRun, totUnoccDistRun, totCarsRun, totTripsRun, totHSRun, totCSRun, totWaitTRun, totUnservedTRun, totWaitCountRun,
                           totUnusedRun, totUnoccRun, totAvgWait, totAvgTripDist, totDistRunCOV, totUnoccDistRunCOV, totCarsRunCOV, totTripsRunCOV,
                           totHSRunCOV, totCSRunCOV, totWaitTRunCOV, totUnservedTRunCOV, totWaitCountRunCOV, totUnusedRunCOV, totUnoccRunCOV,
                           totAvgWaitCOV, totAvgTripDistCOV, totStartsPerTripCOV, totAvgTripsPerCarCOV, totWaitCOV, totTripDistCOV, totCarTripsCOV,
                           totPctMaxWaitFiveCOV, totPctInducedTCOV, totPctMaxInUseCOV, totPctMaxOccCOV, totPctColdShareCOV, numRuns);
    }

    return 0;
}

//****Major Functions called from Main*****************************************************************************************************************

// initializes all variables
void initVars (int runNum, bool warmStart)
{
    double inputVal;
    char comment;
    char* varStr;
    char* valStr;
    char instring [80];
    char readName[20];
    int rseed;

    error = false;
    wStart = true;
    outerRate = -1;
    innerRate = -1;
    nearRate = -1;
    exurbanRate = -1;
    maxTrav = -1;

    for(int t = 0; t < 288; t++)
    {
        maxCarUse[t] = 0;
        maxCarOcc[t] = 0;
    }

    totDist = 0;
    unoccDist = 0;
    waitT = 0;
    reportProcs = false;
    saveRate = 0;
    startIter = 0;
    readFile = false;
    rseed = 0;
    hotStarts = 0;
    coldStarts = 0;

    if (runNum == 1)
    {
        totDistRun = 0;
        totUnoccDistRun = 0;
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


        for (int i = 0; i < 6; i++)
        {
            totWaitCountRun[i] = 0;
        }

        for (int i = 0; i < numRuns; i++)
        {
            totDistRunCOV[i] = 0;
            totUnoccDistRunCOV[i] = 0;
            totCarsRunCOV[i] = 0;
            totTripsRunCOV[i] = 0;
            totHSRunCOV[i] = 0;
            totCSRunCOV[i] = 0;
            totWaitTRunCOV[i] = 0;
            totUnservedTRunCOV[i] = 0;
            totWaitCountRunCOV[i] = 0;
            totUnusedRunCOV[i] = 0;
            totUnoccRunCOV[i] = 0;
            totAvgWaitCOV[i] = 0;
            totAvgTripDistCOV[i] = 0;
            totStartsPerTripCOV[i] = 0;
            totAvgTripsPerCarCOV[i] = 0;

            totPctMaxWaitFiveCOV[i] = 0;
            totPctInducedTCOV[i] = 0;
            totPctMaxInUseCOV[i] = 0;
            totPctMaxOccCOV[i] = 0;
            totPctColdShareCOV[i] = 0;
        }
    }


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
                maxTravC = (int) inputVal;
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

    if (runNum == 1)
    {
        srand(rseed);
    }

    fclose(inputfile);

    if (readFile)
    {
        restoreStatus( timeTripCounts, CarMx, maxTrav,  maxCarUse, totDist, unoccDist, reportProcs, saveRate, wStart, startIter, readName, error, wStart, startIter);
    } else {

        if (outerRate < 0 || innerRate < 0 || maxTrav <= 0 || saveRate < 0)
        {
            cout << "Input file read error! At least one of outerRate, innerRate, maxTrav, or saveRate not defined or invalid entry." << endl;
            cin >> maxTrav; // dummy input
            error = true;
        }

        for (int t = 0; t < 288; t++)
        {
            	TTMx[t].clear();
		timeTripCounts[t] = 0;
        	if(TTMx[t].size() != timeTripCounts[t])
			cout <<"problem"<<endl;
	}

        if (warmStart)
        {
            for (int x = 0; x < xMax; x++)
            {
                for (int y = 0; y < yMax; y++)
                {
                    CarMx[x][y].clear();
					
                }
            }
        }
	
        setZoneShares (zoneSharesS, outerRate, innerRate, nearRate, exurbanRate, numZonesS, zoneSizeS);
	setZoneShares (zoneSharesL, outerRate, innerRate, nearRate, exurbanRate, numZonesL, zoneSizeL);
        setStartTimes (startTimes);
        setTripDistLarge (tripDist);
	setDwTimes (dwLookup);

    }
    return;
}

// determines weights for AM-PM trip balancing
void findDistWeight()
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
void generateTrips ()
{
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

                nTrip.startTime = time;
                nTrip.startX = *itx;
                nTrip.startY = *ity;
                nTrip.endX = newX;
                nTrip.endY = newY;
                nTrip.carlink = false;
                nTrip.returnHome = false;
                nTrip.waitTime = 0;
                nTrip.tripDist = (abs(nTrip.startY - nTrip.endY) + abs(nTrip.startX - nTrip.endX));
                nTrip.tripDist = nTrip.tripDist / 4;
                nTrip.waitPtr = NULL;
		sum += nTrip.tripDist; // in miles
		count++;	
                
		if (timeTripCounts[time] > TTMxSize - 2)
                {
                    //cout << "Warning, about to exceed memory adding trip " << TTMx[time].size() << " to time " << time << endl;
                }

                placeInTTM ( nTrip, timeTripCounts, time);
		if(nTrip.startX >= xMax)
			cout << "exceeded x boundary"<<endl;
            }
        }
    }
//double avg = 1.0 * sum;
//	cout << "Average: "<< avg / count << endl;

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

    if (printZ)
    {

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

    }

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

    return;

}

//Runs the main thrust of the program - pairs all trips with vehicles and moves travelers from origin to destination
void runSharedAV ( int* timeTripCounts, std::vector<Car> CarMx[][yMax], int maxTrav, int maxTravC,  double dwLookup [][288],
                  double zoneSharesL[][yMax], double zoneSharesS[][yMax], double* maxCarUse, double* maxCarOcc, int& totDist, int& unoccDist, int& waitT,
                  bool reportProcs, int saveRate, bool warmStart, bool lastWarm, long& maxAvailCars, bool& readFile, int startIter, int& unservedT, int* waitCount,
                  int& hotStarts, int& coldStarts, int nRuns)
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
//    Trip waitList [WaitListSize][6]; // wait list
    int waitListI[6]; // wait list index

    int nw, ne, sw, se;

    int zoneGen[numZonesL][numZonesL]; //xmas, ymax
    int waitZones[numZonesL][numZonesL]; //xmax,ymax
    int waitZonesTL[numZonesL][numZonesL];
    int waitZonesTS[numZonesS][numZonesS];
    double netZoneBalance[numZonesL][numZonesL];
    int tripO[numZonesL][numZonesL];
    int tripD[numZonesL][numZonesL];
    int cardDirectLZ[numZonesL];
    int cardDirect[numZonesL];
    int cardDirect2[numZonesL];

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

    for (t = startT; t < 288; t++)
    {

        carCt = 0;
        for (int xc = 0; xc < xMax; xc++)
        {
            for (int yc = 0; yc < yMax; yc++)
            {
                carCt = carCt + CarMx[xc][yc].size();//numCars[xc][yc];
            }
        }

        if (nRuns == 1)
        {
            cout << "T = " << t << ", trips = " << timeTripCounts[t] << ", numCars = " << carCt << ", Delay = ";
            for (int w = 0; w < 6; w++)
            {
                cout << waitListI[w] << ",";
            }
            cout << endl;
        }

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
                saveStatus( timeTripCounts, CarMx, maxTrav,  maxCarUse, totDist, unoccDist, reportProcs, saveRate, warmStart, t);
            }
        }

        // assume congested speeds between 7-8 AM and 4-6:30 PM
        if (((t >= 84) && (t <= 96)) || ((t >= 192) && (t <= 222)))
        {
            trav = maxTravC;
        } else {
            trav = maxTrav;
        }
	
// cout << "moving" << endl;


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

// ************* New Function *****************
// for all waitlisted trips from previous time period, find the nearest car
        for (int w = 5; w >= 0; w--)
        {
            for (int d = 0; d < trav; d++)
            {
                for (int i = 0; i < waitListI[w]; i++)
                {
                    if (waitList[w][i].carlink == false)
                    {
			findNearestCar(waitList[w][i], CarMx, d, trav,  reportProcs, nw, ne, se, sw, coldStarts, hotStarts);
                    }
                }
            }
        }
//*************************************************


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

        for (int w = 5; w >= 0; w--)
        {
//            if (waitListI[0] > 0)
//            {
//                reportProcs = false;
//            }

           waitCount[w] = waitCount[w] + waitListI[w];

            for (int i = 0; i < waitListI[w]; i++)
            {
                if (waitList[w][i].carlink == false)
                {

//                    if (warmStart && totNumCars < 1500)
                    if ((warmStart && !lastWarm) || (lastWarm && totNumCars < maxAvailCars))
                    {
                        // generate a new car
                        newCarCt ++;
                        nCar = genNewCar (waitList[w][i]);
                        cn = CarMx[nCar.x][nCar.y].size();
                        CarMx [nCar.x][nCar.y].push_back(nCar);

                        zoneGen[nCar.x / zoneSizeL][nCar.y / zoneSizeL]++;

                        totNumCars++;
                    }

                    else if (w == 5)
                    {
                        unservedT++;
                    } else {
                        // move car to the next wait time interval
                        waitList[w][i].waitTime = waitList[w][i].waitTime + 5;
                        waitZonesTL[waitList[w][i].startX / zoneSizeL][waitList[w][i].startY / zoneSizeL]++;
                        waitZonesTS[waitList[w][i].startX / zoneSizeS][waitList[w][i].startY / zoneSizeS]++;
                        //waitList[waitListI[w+1]][w+1] = waitList[i][w];
                        waitList[w+1].push_back(waitList[w][i]);
			waitListI[w+1]++;
                    }
			if(waitList[w][i].startX>=xMax)
				cout <<"About to exceed in move"<<endl;
                }
            }
            waitListI[w] = 0;
	    waitList[w].clear();
        }


//	Scram(TTMx[t],trav);


// **************** New Function *****************************************************
// for all trips, find the nearest car
        for (int d = 0; d < trav; d++)
        {

            if (t % 2 == 0) // iterate forwards through timetrip counts and then backwards every other interval in order to ensure equal generation for new cars
            {
                for (int trp = 0; trp < timeTripCounts[t]; trp++)
                {
                    if (TTMx[t][trp].carlink == false)
                    {
                        if (reportProcs)
                        {
                            cout << "Trip " << trp << " at distance " << d << endl;
                        }
                        findNearestCar(TTMx[t][trp], CarMx, d, trav,  reportProcs, nw, ne, se, sw, coldStarts, hotStarts);
                    }
                }
            } else {
                for (int trp = timeTripCounts[t] - 1; trp >= 0; trp--)
                {
                    if (TTMx[t][trp].carlink == false)
                    {
                        if (reportProcs)
                        {
                            cout << "Trip " << trp << " at distance " << d << endl;
                        }
                        findNearestCar(TTMx[t][trp], CarMx, d, trav,  reportProcs, nw, ne, se, sw, coldStarts, hotStarts);
                    }
                }
            }
        }
// **************************************************************************************


// for each trip that hasn't found a car, create a new one if in warm start or put on the wait list if running normal run
        for (int trp = 0; trp < timeTripCounts[t]; trp++)
        {
            if (TTMx[t][trp].carlink == false)
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

                    waitZonesTL[TTMx[t][trp].startX / zoneSizeL][TTMx[t][trp].startY / zoneSizeL]++;
                    waitZonesTS[TTMx[t][trp].startX / zoneSizeS][TTMx[t][trp].startY / zoneSizeS]++;

                    TTMx[t][trp].waitTime = 5;
			waitList[0].push_back(TTMx[t][trp]);
//                    waitList[waitListI[0]][0] = TTMx[t][trp];
                    waitList[0][waitListI[0]].waitPtr = &TTMx[t][trp];
                    waitListI[0]++;
         	}
	     //}
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
            showWaitCars(t, waitList, waitListI, CarMx);
        }

//        if (!warmStart)
//        {
//            reportProcs = false;
//        }

// move all cars that are in use
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].inUse && !CarMx[x][y][c].moved)
                    {
                        moveCar (CarMx,  x, y, c, t, trav, totDist, unoccDist, waitT, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                 trackX, trackY, trackC);
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
                    if (CarMx[x][y][c].moved == false)
                    {
                        tempCarOcc++;
                    }
                }
            }
        }

        maxCarOcc[t] = maxCarOcc[t] + tempCarOcc;

//        if (tempCarOcc < maxCarOcc)
//        {
//            maxCarOcc = tempCarOcc;

        


// **************** vehicle reallocation portion ****************************************************************

// reallocate vehicles based on zones

 // cout << "reloc" << endl;

        reallocVehsLZones (CarMx,   timeTripCounts, dwLookup, zoneSharesS, t, reportProcs, totDist, unoccDist, hotStarts, coldStarts, trav,
                           netZoneBalance, cardDirectLZ, waitZonesTS, numZonesS, zoneSizeS, trackX, trackY, trackC);

        randOrdering(xRandOrd, xMax);
        randOrdering(yRandOrd, yMax);

        // shift vehicles two spaces, if current space has 3 or my vehicles and nearby space is unoccupied with no neighbors
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                reallocVehs2(xRandOrd[x], yRandOrd[y], CarMx,   timeTripCounts, dwLookup, reportProcs, totDist, unoccDist, hotStarts, coldStarts,
                             cardDirect2, trackX, trackY, trackC);
            }
        }

        randOrdering(xRandOrd, xMax);
        randOrdering(yRandOrd, yMax);

        // shift vehicles one space if difference between current space and new space is more than 2
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                reallocVehs(xRandOrd[x], yRandOrd[y], CarMx,   timeTripCounts, dwLookup, reportProcs, totDist, unoccDist, hotStarts, coldStarts,
                            cardDirect, trackX, trackY, trackC);
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

// refuel all cars that are low on gas
        for (int x = 0; x < xMax; x++)
        {
            for (int y = 0; y < yMax; y++)
            {
                for (int c = 0; c < CarMx[x][y].size(); c++)
                {
                    if (CarMx[x][y][c].refuel > 0)
                    {
                        CarMx[x][y][c].refuel--;
                        CarMx[x][y][c].gas = carRange;

                        if (CarMx[x][y][c].refuel == 0)
                        {
                            CarMx[x][y][c].inUse = false;
                        }
                    }
                }
            }
        }

        maxCarUse[t] = maxCarUse[t] + tempMinUnused;
//        if (tempMinUnused < maxCarUse)
//        {
//            maxCarUse = tempMinUnused;
//        }
    }

    runSummary(warmStart, lastWarm, zoneGen, waitZones, netZoneBalance, tripO, tripD, cardDirectLZ, cardDirect, cardDirect2,  timeTripCounts);

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

    return;

}

// resets all variables for the daily run, now that we know how many cars there will be
void placeInitCars (std::vector<Car> CarMx[][yMax],   int* timeTripCounts, double* maxCarUse, double* maxCarOcc, int& totDist,
                    int& unoccDist, int& waitT, double dwLookup [][288], bool reportProcs, int& hotStarts, int& coldStarts)
{
//    int dx, dy;


//    maxCarUse = 20000;
//    maxCarOcc = 20000;
    totDist = 0;
    unoccDist = 0;
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
                CarMx[x][y][c].inUse = false;
                CarMx[x][y][c].pickupX = -1;
                CarMx[x][y][c].pickupY = -1;
                CarMx[x][y][c].destX = CarMx[x][y][c].x;
                CarMx[x][y][c].destY = CarMx[x][y][c].y;
                CarMx[x][y][c].returnHome = false;
                CarMx[x][y][c].tripCt = 0;
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
void reportResults ( int* timeTripCounts, std::vector<Car> CarMx[][yMax],  double* maxCarUse, double* maxCarOcc, int totDist,
                    int unoccDist, int waitT, int unservedT, int* waitCount, int hotStarts, int coldStarts, long& totDistRun, long& totUnoccDistRun,
                    long& totCarsRun, long& totTripsRun, long& totHSRun, long& totCSRun, long& totWaitTRun, long& totUnservedTRun, long* totWaitCountRun,
                    long& totUnusedRun, long& totUnoccRun, double& totAvgWait, double& totAvgTripDist, double* totDistRunCOV, double* totUnoccDistRunCOV,
                    double* totCarsRunCOV, double* totTripsRunCOV, double* totHSRunCOV, double* totCSRunCOV, double* totWaitTRunCOV, double* totUnservedTRunCOV,
                    double* totWaitCountRunCOV, double* totUnusedRunCOV, double* totUnoccRunCOV, double* totAvgWaitCOV, double* totAvgTripDistCOV,
                    double* totStartsPerTripCOV, double* totAvgTripsPerCarCOV, double& totWaitCOV, double& totTripDistCOV, double& totCarTripsCOV,
                    double* totPctMaxWaitFiveCOV, double* totPctInducedTCOV, double* totPctMaxInUseCOV, double* totPctMaxOccCOV, double* totPctColdShareCOV,
                    int nRuns, int runNum)
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
            nCars = nCars + CarMx[x][y].size();
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
            avgWait = avgWait + TTMx[t][trp].waitTime;
            avgDist = avgDist + TTMx[t][trp].tripDist;
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

    avgWait = avgWait / nTrips;
    avgDist = avgDist / nTrips;
    avgTrips = avgTrips / nCars;

    for (int t = 0; t < 288; t++)
    {
        for (int trp = 0; trp < timeTripCounts[t]; trp++)
        {
            waitCOV = waitCOV + pow(TTMx[t][trp].waitTime - avgWait, 2);
            tripDistCOV = tripDistCOV + pow(TTMx[t][trp].tripDist - avgDist, 2);
        }
    }

    for (int x = 0; x < xMax; x++)
    {
        for (int y = 0; y < yMax; y++)
        {
            for (int c = 0; c < CarMx[x][y].size(); c++)
            {
                carTripsCOV = carTripsCOV + pow(CarMx[x][y][c].tripCt - avgDist, 2);
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
    waitCOV = sqrt(waitCOV / nTrips) / avgWait;
    tripDistCOV = sqrt(tripDistCOV / nTrips) / avgDist;
    carTripsCOV = sqrt(carTripsCOV / nTrips) / avgTrips;

    cout << "Total number of trips " << nTrips << endl;
    cout << "Total number of unserved trips " << unservedT << endl;
    cout << "Total number of cars " << nCars << endl;
//    cout << "5-minute traveler wait time intervals elapsed " << waitCount << endl;
    cout << "Average wait time " << avgWait << endl;
    cout << "Total miles traveled " << totDist / 4 << endl;
    cout << "Total unoccupied miles traveled " << unoccDist / 4 << endl;
    cout << "Average trip miles " << avgDist << endl;
//    cout << "Maximum number of trips starting during any 5 minute period " << maxTripGen << endl;
    cout << "Maximum number of cars in use during any 5 minute period " << maxCarUseT << endl;
    cout << "Maximum number of cars occupied during any 5 minute period " << maxCarOccT << endl;
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

    if (nRuns == 1)
    {
        cout << "Press any key and enter to exit." << endl;
        cin >> dummyStr;
    }

    totTripsRun = totTripsRun + nTrips;
    totUnservedTRun = totUnservedTRun + unservedT;
    totCarsRun = totCarsRun + nCars;
    totWaitTRun = totWaitTRun + waitT;
    totHSRun =  totHSRun + hotStarts;
    totCSRun = totCSRun + coldStarts;
    totDistRun = totDistRun + totDist;
    totUnoccDistRun = totUnoccDistRun + unoccDist;
    totUnusedRun = totUnusedRun + maxCarUseT;
    totUnoccRun = totUnoccRun + maxCarOccT;
    totAvgWait = totAvgWait + avgWait;
    totAvgTripDist = totAvgTripDist + avgDist;
    totWaitCOV = totWaitCOV + waitCOV;
    totTripDistCOV = totTripDistCOV + tripDistCOV;
    totCarTripsCOV = totCarTripsCOV + carTripsCOV;

    for (int i = 0; i < 6; i++)
    {
        totWaitCountRun[i] = totWaitCountRun[i] + waitCount[i];
    }

    totDistRunCOV[runNum-1] = totDist;
    totUnoccDistRunCOV[runNum-1] = unoccDist;
    totCarsRunCOV[runNum-1] = nCars;
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
    totAvgTripsPerCarCOV[runNum-1] = double(nTrips) / double(nCars);

    totPctMaxWaitFiveCOV[runNum-1] = totWaitCount / double(nTrips);
    totPctInducedTCOV[runNum-1] = double (unoccDist) / double(totDist);
    totPctMaxInUseCOV[runNum-1] = 1 - double (maxCarUseT) / double (nCars);
    totPctMaxOccCOV[runNum-1] = 1 - double (maxCarOccT) / double (nCars);
    totPctColdShareCOV[runNum-1] = double (coldStarts) / double (coldStarts + hotStarts);

    return;
}

void reportFinalResults (long totDistRun, long totUnoccDistRun, long totCarsRun, long totTripsRun, long totHSRun, long totCSRun, long totWaitTRun,
                         long totUnservedTRun, long* totWaitCountRun, long totUnusedRun, long totUnoccRun, double totAvgWait, double totAvgTripDist,
                         double* totDistRunCOV, double* totUnoccDistRunCOV, double* totCarsRunCOV, double* totTripsRunCOV, double* totHSRunCOV,
                         double* totCSRunCOV, double* totWaitTRunCOV, double* totUnservedTRunCOV, double* totWaitCountRunCOV, double* totUnusedRunCOV,
                         double* totUnoccRunCOV, double* totAvgWaitCOV, double* totAvgTripDistCOV,  double* totStartsPerTripCOV, double* totAvgTripsPerCarCOV,
                         double totWaitCOV, double totTripDistCOV, double totCarTripsCOV, double* totPctMaxWaitFiveCOV, double* totPctInducedTCOV,
                         double* totPctMaxInUseCOV, double* totPctMaxOccCOV, double* totPctColdShareCOV, int numRuns)
{
    char dummyStr[20];
    double totDistRunCOVF, totUnoccDistRunCOVF,totCarsRunCOVF, totTripsRunCOVF, totHSRunCOVF, totCSRunCOVF, totWaitTRunCOVF, totUnservedTRunCOVF,
           totWaitCountRunCOVF, totUnusedRunCOVF, totUnoccRunCOVF, totAvgWaitCOVF, totAvgTripDistCOVF, totStartsPerTripCOVF, totAvgTripsPerCarCOVF,
           totPctMaxWaitFiveCOVF, totPctInducedTCOVF, totPctMaxInUseCOVF, totPctMaxOccCOVF, totPctColdShareCOVF;


    findCOV (totDistRunCOVF, totDistRunCOV, numRuns);
    findCOV (totUnoccDistRunCOVF, totUnoccDistRunCOV, numRuns);
    findCOV (totCarsRunCOVF, totCarsRunCOV, numRuns);
    findCOV (totTripsRunCOVF, totTripsRunCOV, numRuns);
    findCOV (totHSRunCOVF, totHSRunCOV, numRuns);
    findCOV (totCSRunCOVF, totCSRunCOV, numRuns);
    findCOV (totWaitTRunCOVF, totWaitTRunCOV, numRuns);
    findCOV (totUnservedTRunCOVF, totUnservedTRunCOV, numRuns);
    findCOV (totWaitCountRunCOVF, totWaitCountRunCOV, numRuns);
    findCOV (totUnusedRunCOVF, totUnusedRunCOV, numRuns);
    findCOV (totUnoccRunCOVF, totUnoccRunCOV, numRuns);
    findCOV (totAvgWaitCOVF, totAvgWaitCOV, numRuns);
    findCOV (totAvgTripDistCOVF, totAvgTripDistCOV, numRuns);
    findCOV (totStartsPerTripCOVF, totStartsPerTripCOV, numRuns);
    findCOV (totAvgTripsPerCarCOVF, totAvgTripsPerCarCOV, numRuns);

    findCOV (totPctMaxWaitFiveCOVF, totPctMaxWaitFiveCOV, numRuns);
    findCOV (totPctInducedTCOVF, totPctInducedTCOV, numRuns);
    findCOV (totPctMaxInUseCOVF, totPctMaxInUseCOV, numRuns);
    findCOV (totPctMaxOccCOVF, totPctMaxOccCOV, numRuns);
    findCOV (totPctColdShareCOVF, totPctColdShareCOV, numRuns);


    cout << endl << endl << "****************************************" << endl << endl;

    totTripsRun = totTripsRun / numRuns;
    totCarsRun = totCarsRun / numRuns;
    totWaitTRun = totWaitTRun / numRuns;
    totHSRun =  totHSRun / numRuns;
    totCSRun = totCSRun / numRuns;
    totDistRun = totDistRun / numRuns;
    totUnoccDistRun = totUnoccDistRun / numRuns;
    totUnoccRun = totUnoccRun / numRuns;
    totUnusedRun = totUnusedRun / numRuns;
    totAvgWait = totAvgWait / numRuns;
    totAvgTripDist = totAvgTripDist / numRuns;
    totWaitCOV = totWaitCOV / numRuns;
    totTripDistCOV = totTripDistCOV / numRuns;
    totCarTripsCOV = totCarTripsCOV / numRuns;

    for (int i = 0; i < 6; i++)
    {
        totWaitCountRun[i] = totWaitCountRun[i] / numRuns;
    }

    cout << "Average number of trips " << totTripsRun << "           COV: " << totTripsRunCOVF << endl;
    cout << "Total number of unserved trips " << totUnservedTRun << "        COV: " << totUnservedTRunCOVF << endl;
    cout << "Average number of cars " << totCarsRun << "             COV: " << totCarsRunCOVF << endl;
//    cout << "5-minute wait intervals elapsed " << totWaitCountRun << "    COV: " << totWaitCountRunCOVF << endl;
    cout << "Average wait time " << totAvgWait << "              COV: " << totAvgWaitCOVF  << endl;
//    cout << "Maximum number of trips starting during any 5 minute period " << maxTripGen << endl;
    cout << "Average total miles traveled " << totDistRun / 4 << "     COV: " << totDistRunCOVF << endl;
    cout << "Average total unocc mi traveled  " << totUnoccDistRun / 4 << "  COV: " << totUnoccDistRunCOVF << endl;
    cout << "Average trip distance " << totAvgTripDist << "           COV: " << totAvgTripDistCOVF << endl;
    cout << "Average min number unused cars " << totUnusedRun << "       COV: " << totUnusedRunCOVF << endl;
    cout << "Average min number unoccupied cars " << totUnoccRun << "  COV: " << totUnoccRunCOVF << endl;
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

    cout << "Press any key and enter to exit." << endl;
    //cin >> dummyStr;

    return;
}


//****Functions called from InitVars*************************************************************************************************************

//cumulative distribution of trip start times in 5 minute increments
void setStartTimes(double startTimes [288])
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
void setTripDist(double tripDist [tripDistSize])
{
	
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
void setTripDistLarge(double tripDist [tripDistSize])
{
	
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

// distribution of dwell times before returning
void setDwTimes (double dwLookup [][288])
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
void setZoneShares (double zoneShares[][yMax], double outerRate, double nearRate, double innerRate, double exurbanRate, int numZones, int zoneSize)
{
    int xz, yz;
    double r;
    double newTrips;
    double totTrips = 0;

    r = sqrt (pow(double (xMax - 1) / 2,2) + pow(double (yMax - 1) / 2,2));  //radius from the city center

    for (xz = 0; xz < numZones; xz++)
    {
        for (yz = 0; yz < numZones; yz++)
        {
            zoneShares[xz][yz] = 0;
        }
    }

    // get total trips
    for (int x = 0; x < xMax; x++)
    {
        xz = x / zoneSize;
        for (int y = 0; y < yMax; y++)
        {
            yz = y / zoneSize;
            newTrips = getRate (x, y, r, double(xMax - 1) / 2, double(yMax - 1) / 2, outerRate, innerRate, nearRate, exurbanRate);
            zoneShares[xz][yz] = zoneShares[xz][yz] + newTrips;
            totTrips = totTrips + newTrips;
        }
    }

    // normalize to sum of trips = 1
    for (int xz = 0; xz < numZones; xz++)
    {
        for (int yz = 0; yz < numZones; yz++)
        {
            zoneShares[xz][yz] = zoneShares[xz][yz] / totTrips;
        }
    }

    if (0)
    {
        for (int yz = numZones - 1; yz >= 0; yz--)
        {
            for (int xz = 0; xz < numZones; xz++)
            {
                cout << int (1000 * zoneShares[xz][yz]) << " ";
            }
            cout << endl;
        }
    }

    return;
}


// restores status to previous iteration
void restoreStatus( int* timeTripCounts, std::vector<Car> CarMx[][yMax], int maxTrav,  double* maxCarUse, int& totDist,
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
    readCarMx(infile,  CarMx);

    infile.close();

    return;
}

//****Functions called from findDistWeight*****************************************************************************************************************

// counts the difference in trips generated in the outer areas, compared to trips ending in the outer areas
int countDiff (double outerRate, double innerRate, double nearRate, double exurbanRate, double* startTimes, double* tripDist, int* timeTripCounts, double distWt, bool reportProcs)
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

    generateTrips ();

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
double getRate (int x, int y, double r, double xCent, double yCent, double outerRate, double innerRate, double nearRate, double exurbanRate)
{
    double rate;
    double distToCen = sqrt (pow((x - xCent),2) + pow((y - yCent),2));

    if (distToCen <= innerDist)
    {
        rate = nearRate * (distToCen / innerDist) + innerRate * ((nearDist - distToCen) / innerDist);
    } 
    else if (distToCen <= nearDist)
    {
	rate = outerRate * ((distToCen - innerDist) / (nearDist - innerDist)) + nearRate * ((nearDist - distToCen)/(nearDist - innerDist));
    }
    else {
        rate = exurbanRate * ((distToCen - nearDist) / (r - nearDist)) + outerRate * (r - distToCen) / (r - nearDist);
    }

    return rate;
}

// returns the time of day for a given trip (0 to 287 for 24 hours divided into 5 minute blocks)
int getStartTime(double* startTimes)
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
void getDest (int x, int y, int &newX, int &newY, double* tripDist, double dWt)
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

int getDestDist(double* tripDist)
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
void placeInTTM ( Trip ntrip, int* timeTripCounts, int time)
{
    TTMx[time].push_back(ntrip);
    timeTripCounts[time]++;
	if(TTMx[time].size() != timeTripCounts[time])
		cout << "Trip # (vec first): " << TTMx[time].size() << " = " << timeTripCounts[time] << " ?"<<endl;
    return;
}

// generates trip using a Poisson distribution
int genPoisson (double mean)  //Generated using the Box-Muller method
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
void saveStatus( int* timeTripCounts, std::vector<Car> CarMx[][yMax], int maxTrav,  double* maxCarUse, int totDist,
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
    writeNumFreeCars(saveFile, CarMx);
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
void findNearestCar (Trip& trp, std::vector<Car> CarMx[][yMax], int dist, int maxDist,  bool reportProcs,
                     int& nw, int& ne, int& se, int& sw, int& coldStart, int& hotStart)
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
  
    if (dist == 0)
    {
        found = lookForCar (trp.startX, trp.startY, tripDist, c, CarMx);
        if (found)
        {
            assignCar(trp.startX, trp.startY, c, CarMx, trp);
            x = trp.startX;
            y = trp.startY;
        }
    } else {

        r = rand();
        r = r / RAND_MAX;

        if (r < 0.25) // start with the northwest quadrant
        {
            for (d = 0; d < dist; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, tripDist + dist, c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }
            for (d = 0; d < dist; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, tripDist, c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                ne++;
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, tripDist+dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                se++;
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, tripDist+dist, c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }

        } else if (r < 0.5) {// start with the northeast quadrant

            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, tripDist + dist, c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                ne++;
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                se++;
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }

        } else if (r < 0.75) { // start with the southeast quadrant

            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                se++;
            }
            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, tripDist+dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                ne++;
            }

        } else { // start with the southwest quadrant

            for (d = 0; d < dist && !found; d++)  //southwest quadrant
            {
                x = max(0, trp.startX - d);
                y = max(0, trp.startY - dist + d);
                found = lookForCar (x, y, tripDist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                sw++;
            }
            for (d = 0; d < dist && !found; d++)  //northwest quadrant
            {
                x = max(0, trp.startX - dist + d);
                y = min(yMax - 1, trp.startY + d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
                nw++;
            }
            for (d = 0; d < dist && !found; d++)  //northeast quadrant
            {
                x = min(xMax - 1, trp.startX + d);
                y = min(yMax - 1, trp.startY + dist - d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
                    numveh = CarMx[x][y].size();
                }
               ne++;
            }
            for (d = 0; d < dist && !found; d++)  //southeast quadrant
            {
                x = min(xMax - 1, trp.startX + dist - d);
                y = max(0, trp.startY - d);
                found = lookForCar (x, y, tripDist + dist,c, CarMx);
                if (found && CarMx[x][y].size() > numveh)
                {
                    assignCar(x, y, c, CarMx, trp);
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
        trp.waitTime = trp.waitTime + (5 * waitTrav / maxDist) ;
        if (trp.waitPtr != NULL)
        {
            trp.waitPtr -> waitTime = trp.waitTime;
        }
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
bool lookForCar (int x, int y, int dist, int& cn, std::vector<Car> CarMx[][yMax])
{
    int c;
    bool found = false;

    // Start at the top of each vector. This makes replacement easier and can keep better track of which cars are actually used,
    // emphasizing heavy reuse of a few cars, rather than cycling through a larger number that were previously created during the warm start
    for (c = CarMx[x][y].size() - 1; c >=0 && !found; c--)
    {
        if (CarMx[x][y][c].inUse == false)
        {
            if (CarMx[x][y][c].gas >= dist){

	    	found = true;
            	cn = c;

	    }
        }
    }

    return found;
}

void assignCar (int x, int y, int c, std::vector<Car> CarMx[][yMax], Trip trp)
{
    double randRet;

    CarMx[x][y][c].inUse = true;
    CarMx[x][y][c].destX = trp.endX;
    CarMx[x][y][c].destY = trp.endY;
    CarMx[x][y][c].tripCt ++;

    if (x == trp.startX && y == trp.startY)
    {
        CarMx[x][y][c].pickupX = -1;
        CarMx[x][y][c].pickupY = -1;
    } else {
        CarMx[x][y][c].pickupX = trp.startX;
        CarMx[x][y][c].pickupY = trp.startY;
    }

    // for cars departing from home, assume 22% return by other means or another day and the rest will return
    if (trp.returnHome == false)
    {
        randRet = rand();
        randRet = randRet / RAND_MAX;

        if (randRet > 0.22)
        {
            CarMx[x][y][c].returnHome = true;
            CarMx[x][y][c].retHX = trp.startX;
            CarMx[x][y][c].retHY = trp.startY;
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
Car genNewCar (Trip trp)
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

    randGas = rand();
    randGas = randGas / RAND_MAX;

    nCar.gas = carRange * randGas;
    nCar.refuel = 0;

    return nCar;
}

// moves a car that is currently in use, up to maximum distance of maxTrav.  First pickup, then dropoff.
void moveCar (std::vector<Car> CarMx[][yMax],  int x, int y, int c, int t, int maxTrav, int& totDist, int& unoccDist, int& waitT,
              double dwLookup [][288], int* timeTripCounts, bool reportProcs, int& hotStarts, int& coldStarts, int& trackX, int& trackY, int& trackC)
{
    // identify target car in the car matrix
    Car tCar = CarMx[x][y][c];
    int trav = maxTrav;

    if (tCar.pickupX != -1 && tCar.pickupY != -1)
    {
        if (reportProcs)
        {
            cout << "Pickup: Moving car from (" << tCar.x << "," << tCar.y << ") to (" << tCar.pickupX << "," << tCar.pickupY << ")" << endl;
            cout << "Tot dist " << totDist << endl;
        }

        // the vehicle does not have the passenger, go to pickup location
        unoccDist = unoccDist + abs(tCar.pickupX - tCar.x) + abs(tCar.pickupY - tCar.y);
        totDist = totDist + abs(tCar.pickupX - tCar.x) + abs(tCar.pickupY - tCar.y);
        waitT = waitT + abs(tCar.pickupX - tCar.x) + abs(tCar.pickupY - tCar.y);
        trav = trav - abs(tCar.pickupX - tCar.x) - abs(tCar.pickupY - tCar.y);
        tCar.x = tCar.pickupX;
        tCar.y = tCar.pickupY;


    }

    if (reportProcs)
    {
        cout << "Moving car from (" << tCar.x << "," << tCar.y << ") to (" << tCar.destX << "," << tCar.destY << ")" << endl;
        cout << "Tot dist " << totDist << endl;
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

    // now that we have the new destination, move the car
    if (reportProcs)
    {
        cout << endl; // << "Tot dist " << totDist << endl;
        cout << "Moving car from " << x << "," << y << " to " << tCar.x << "," << tCar.y << endl;
    }

    move (CarMx, x, y, tCar.x, tCar.y, c, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);

    return;
}

// randomizes the ordering of a given vector, numVals long
void randOrdering(int* xRandOrd, int numVals)
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

void runSummary(bool warmStart, bool lastWarm, int zoneGen[][numZonesL], int waitZones[][numZonesL], double netZoneBalance[][numZonesL], int tripO[][numZonesL], int tripD[][numZonesL], int* cardDirectLZ,
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
void reallocVehs(int x, int y, std::vector<Car> CarMx[][yMax],  int* timeTripCounts, double dwLookup [][288],
                 bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect, int& trackX, int& trackY, int& trackC)
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
            if (CarMx[x][y][i].moved == false)
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
                    move (CarMx, x, y, x, y + 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                    numfreeCars--;
                    nfreeCars++;
                } else if (SRank < NRank && SRank < WRank && SRank < ERank) {
                    // move south
                    CarMx[x][y][cNum].destX = x;
                    CarMx[x][y][cNum].destY = y-1;
                    move (CarMx,  x, y, x, y - 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                    numfreeCars--;
                    sfreeCars++;
                } else if (WRank < NRank && WRank < SRank && WRank < ERank) {
                    // move west
                    CarMx[x][y][cNum].destX = x-1;
                    CarMx[x][y][cNum].destY = y;
                    move (CarMx,  x, y, x - 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                    numfreeCars--;
                    wfreeCars++;
                } else if (ERank < NRank && ERank < SRank && ERank < WRank) {
                    // move east
                    CarMx[x][y][cNum].destX = x+1;
                    CarMx[x][y][cNum].destY = y;
                    move (CarMx, x, y, x + 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
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
                            move (CarMx, x, y, x, y - 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                            numfreeCars--;
                            nfreeCars++;
                        } else if (NRank < SRank) {
                            // move north
                            CarMx[x][y][cNum].destX = x;
                            CarMx[x][y][cNum].destY = y+1;
                            move (CarMx, x, y, x, y + 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
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
                                move (CarMx, x, y, x, y - 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                                numfreeCars--;
                                sfreeCars++;
                            } else {
                                // move north
                                CarMx[x][y][cNum].destX = x;
                                CarMx[x][y][cNum].destY = y+1;
                                move (CarMx, x, y, x, y + 1, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
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
                            move (CarMx, x, y, x + 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                            numfreeCars--;
                            efreeCars++;
                        } else if (WRank < ERank) {
                            // move west
                            move (CarMx, x, y, x - 1, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
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
                                move (CarMx, x, y, x - 1, y, 0, cNum, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                                numfreeCars--;
                                wfreeCars++;
                            } else {
                                // move east
                                CarMx[x][y][cNum].destX = x+1;
                                CarMx[x][y][cNum].destY = y;
                                move (CarMx, x, y, x + 1, y, 0, cNum, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                                numfreeCars--;
                                efreeCars++;
                            }
                        }
                    }
                }

                totDist++;
                unoccDist++;

                // find next non-moved vehicle
                cNum = -1;
                for (int i = 0; i < CarMx[x][y].size(); i++)
                {
                    if (CarMx[x][y][i].moved == false)
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
void reallocVehs2(int x, int y, std::vector<Car> CarMx[][yMax],  int* timeTripCounts, double dwLookup [][288],
                  bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect2, int& trackX, int& trackY, int& trackC)
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
            if (CarMx[x][y][i].moved == false)
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

            NRank = getSurroundCars (x, y + 2, CarMx);
            SRank = getSurroundCars (x, y - 2, CarMx);
            WRank = getSurroundCars (x - 2, y, CarMx);
            ERank = getSurroundCars (x + 2, y, CarMx);

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
                    move (CarMx,  x, y, x, y + 2, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                    numfreeCars --;
                    NRank = 1000;
                } else if (SRank < WRank && SRank < ERank) {
                    // move south
                    CarMx[x][y][cNum].destX = x;
                    CarMx[x][y][cNum].destY = y-2;
                    move (CarMx,  x, y, x, y - 2, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                    numfreeCars --;
                    SRank = 1000;
                } else if (WRank < ERank) {
                    // move west
                    CarMx[x][y][cNum].destX = x-2;
                    CarMx[x][y][cNum].destY = y;
                    move (CarMx,  x, y, x - 2, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
                    numfreeCars --;
                    WRank = 1000;
                } else {
                    // move east
                    CarMx[x][y][cNum].destX = x+2;
                    CarMx[x][y][cNum].destY = y;
                    move (CarMx,  x, y, x + 2, y, cNum, 0, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts, trackX, trackY, trackC);
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

                // find next non-moved vehicle
                cNum = -1;
                for (int i = 0; i < CarMx[x][y].size(); i++)
                {
                    if (CarMx[x][y][i].moved == false)
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
void reallocVehsLZones (std::vector<Car> CarMx[][yMax],  int* timeTripCounts, double dwLookup [][288],
                        double zoneShares[][yMax], int t, bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav,
                        double netZoneBalance[][numZonesL], int* cardDirectLZ, int waitZonesT[][numZonesS], int numZones, int zoneSize, int& trackX, int& trackY, int& trackC)
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
    double zoneBalance[xMax][yMax];
    bool ablePush[xMax][yMax];
    bool ablePull[xMax][yMax];

    // initialize zoneBalance
    for (xb = 0; xb < numZones; xb++)
    {
        for (yb = 0; yb < numZones; yb++)
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
//                if (CarMx[c][xc][yc].inUse == false && CarMx[c][xc][yc].moved == false)
                if (CarMx[xc][yc][c].inUse == false)
                {
                    carCt++;
                    zoneBalance[xb][yb]++;
                }
            }
        }
    }

    // determine zone balance
    for (int xz = 0; xz < numZones; xz++)
    {
        for (int yz = 0; yz < numZones; yz++)
        {
            zoneBalance[xz][yz] = zoneBalance[xz][yz] - (zoneShares[xz][yz] * carCt);
        }
    }

    if (reportProcs)
    {
        for (int yz = numZones - 1; yz >=0; yz--)
        {
            for (int xz = 0; xz < numZones; xz++)
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
        for (int xz = numZones - 1; xz >= 0; xz--)
        {
            for (int yz = numZones - 1; yz >= 0; yz--)
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

            if (ty + 1 < numZones)
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
            if (tx + 1 < numZones)
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
                pushCars(CarMx,   timeTripCounts, dwLookup, zoneBalance, tx, ty, north, south, west, east, t, reportProcs, totDist, unoccDist,
                          hotStarts, coldStarts, maxTrav, cardDirectLZ, numZones, zoneSize, trackX, trackY, trackC);
            } else {
                pullCars(CarMx,   timeTripCounts, dwLookup, zoneBalance, tx, ty, north, south, west, east, t, reportProcs, totDist, unoccDist,
                          hotStarts, coldStarts, maxTrav, cardDirectLZ, numZones, zoneSize, trackX, trackY, trackC);
            }
            tb = 0;
            absTB = 0;
        } else {
            findNextZone = false;
        }
    }

    if (reportProcs)
    {
        for (int y = numZones - 1; y >= 0; y--)
        {
            for (int x = 0; x < numZones; x++)
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
void reallocVehsSZones (Car CarMx[][xMax][yMax], int numCars[xMax][yMax], Trip TTMx[][288], int* timeTripCounts, double dwLookup [][288],
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
double getSurroundCars (int x, int y, std::vector<Car> CarMx[][yMax])
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
        central = findFreeCars (x, y, CarMx);
        north = findFreeCars (x, y, CarMx);
        south = findFreeCars (x, y, CarMx);
        east = findFreeCars (x, y, CarMx);
        west = findFreeCars (x, y, CarMx);

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
            north = findFreeCars (x, y + 1, CarMx);
        }
        if (y - 1 >= 0)
        {
            south = findFreeCars (x, y - 1, CarMx);
        }
        if (x + 1 < xMax)
        {
            east = findFreeCars (x + 1, y, CarMx);
        }
        if (x - 1 >= 0)
        {
            west = findFreeCars (x - 1, y, CarMx);
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
double findFreeCars (int x, int y, std::vector<Car> CarMx[][yMax])
{
    double nCars = 0;

    for (int c = 0; c < CarMx[x][y].size(); c++)
    {
        if (CarMx[x][y][c].inUse == false)
        {
            nCars++;
        }
    }

    return nCars;
}


// Function called from both reallocVehsLZones *******************************************************************

// balances cars in zone tx, ty by pushing excess cars to adjacent zone areas
void pushCars(std::vector<Car> CarMx[][yMax], int* timeTripCounts, double dwLookup [][288],
              double zoneBalance[][xMax], int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC)
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
                if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false)
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx, zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY + dist;
                            move (CarMx, origX, origY, origX, origY + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx, zoneSize);
                            CarMx[origX][origY][c].destX = origX + dist;
                            CarMx[origX][origY][c].destY = origY;
                            move (CarMx,  origX, origY, origX + dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx, zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY - dist;
                            move (CarMx, origX, origY, origX, origY - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx,  zoneSize);
                            CarMx[origX][origY][c].destX = origX - dist;
                            CarMx[origX][origY][c].destY = origY;
                            move (CarMx,  origX, origY, origX - dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx,  zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY - dist;
                            move (CarMx,  origX, origY, origX, origY - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx,  zoneSize);
                            CarMx[origX][origY][c].destX = origX - dist;
                            CarMx[origX][origY][c].destY = origY;
                            move (CarMx,  origX, origY, origX - dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx,  zoneSize);
                            CarMx[origX][origY][c].destX = origX;
                            CarMx[origX][origY][c].destY = origY + dist;
                            move (CarMx,  origX, origY, origX, origY + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
                        if (CarMx[origX][origY][c].inUse == false && CarMx[origX][origY][c].moved == false)
                        {
                            cn = c;
                            dist = findMoveDist(origX, origY, direct, lay + 1, maxTrav, CarMx,  zoneSize);
                            CarMx[origX][origY][c].destX = origX + dist;
                            CarMx[origX][origY][c].destY = origY;
                            move (CarMx,  origX, origY, origX + dist, origY, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                  trackX, trackY, trackC);
                            unoccDist = unoccDist + dist;
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
void pullCars(std::vector<Car> CarMx[][yMax],  int* timeTripCounts, double dwLookup [][288],
              double zoneBalance[][yMax], int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC)
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
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false)
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
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false)
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
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false)
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
                    if (CarMx[x][y][c].inUse == false && CarMx[x][y][c].moved == false)
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 2; // direction = 2 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx,  zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy - dist;
                                    move (CarMx,  ix, iy, ix, iy - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 1; // direction = 1 (move the vehicle north)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx, zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy + dist;
                                    move (CarMx, ix, iy, ix, iy + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 4; // direction = 4 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx, zoneSize);
                                    CarMx[ix][iy][c].destX = ix + dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move (CarMx, ix, iy, ix + dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 3; // direction = 3 (move the vehicle west)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx, zoneSize);
                                    CarMx[ix][iy][c].destX = ix - dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move (CarMx, ix, iy, ix - dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 2; // direction = 2 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx, zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy - dist;
                                    move (CarMx, ix, iy, ix, iy - dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 1; // direction = 1 (move the vehicle north)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx, zoneSize);
                                    CarMx[ix][iy][c].destX = ix;
                                    CarMx[ix][iy][c].destY = iy + dist;
                                    move (CarMx, ix, iy, ix, iy + dist, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 4; // direction = 4 (move the vehicle south)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx, zoneSize);
                                    CarMx[ix][iy][c].destX = ix + dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move (CarMx, ix, iy, ix + dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
                                if (CarMx[ix][iy][c].inUse == false  && CarMx[ix][iy][c].moved == false)
                                {
                                    cn = c;
                                    direct = 3; // direction = 3 (move the vehicle west)
                                    dist = findMoveDist(ix, iy, direct, lay + 1, maxTrav, CarMx, zoneSize);
                                    CarMx[ix][iy][c].destX = ix - dist;
                                    CarMx[ix][iy][c].destY = iy;
                                    move (CarMx, ix, iy, ix - dist, iy, cn, t, dwLookup,  timeTripCounts, reportProcs, hotStarts, coldStarts,
                                          trackX, trackY, trackC);
                                    unoccDist = unoccDist + dist;
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
double findMoveDist(int origX, int origY, int direct, int zoneEdge, int maxTrav, std::vector<Car> CarMx[][yMax],  int zoneSize)
{
    double rank = 1000;
    double bestRank = 1000;
    double dist = zoneEdge;

    if (direct == 1) // north
    {
        for (int y = origY + zoneEdge; y <= origY + maxTrav && y <= origY + zoneEdge + zoneSize - 1; y++)
        {
            rank = getSurroundCars (origX, y, CarMx) + (4 * CarMx[origX][y].size());
            if (rank < bestRank)
            {
                bestRank = rank;
                dist = y - origY;
            }
        }
    } else if (direct == 2) { // south
        for (int y = origY - zoneEdge; y >= origY - maxTrav && y >= origY - (zoneEdge + zoneSize - 1); y--)
        {
            rank = getSurroundCars (origX, y, CarMx) + (4 * CarMx[origX][y].size());
            if (rank < bestRank)
            {
                bestRank = rank;
                dist = origY - y;
            }
        }
    } else if (direct == 3) { // west
        for (int x = origX - zoneEdge; x >= origX - maxTrav && x >= origX - (zoneEdge + zoneSize - 1); x--)
        {
            rank = getSurroundCars (x, origY, CarMx) + (4 * CarMx[x][origY].size());
            if (rank < bestRank)
            {
                bestRank = rank;
                dist = origX - x;
            }
        }
    } else { // east
        for (int x = origX + zoneEdge; x <= origX + maxTrav && x <= origX + zoneEdge + zoneSize - 1; x++)
        {
            rank = getSurroundCars (x, origY, CarMx) + (4 * CarMx[x][origY].size());
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
void move (std::vector<Car> CarMx[][yMax],  int ox, int oy, int dx, int dy, int c, int t, double dwLookup [][288],
           int* timeTripCounts, bool reportProcs, int& hotStart, int& coldStart, int& trackX, int& trackY, int& trackC)
{


    // identify target car in the car matrix the car will now have picked up passengers and will have moved by the end of this function
    Car tCar = CarMx[ox][oy][c];
    tCar.x = dx;
    tCar.y = dy;
    tCar.pickupX = -1;
    tCar.pickupY = -1;
    tCar.moved = true;
    tCar.inUse = true;

    // burn fuel driving
    tCar.gas = tCar.gas - abs (ox - dx) - abs (oy - dy);

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
    

    if (dx == tCar.destX && dy == tCar.destY)
    {
        // we have arrived at the target location, free the car
        if (reportProcs)
        {
            cout << "Arrived! Releasing car." << endl;
        }
        tCar.inUse = false;

        // generate a return trip, if slated
        if (tCar.returnHome == true)
        {
            genRetTrip(dx, dy, tCar.retHX, tCar.retHY, t, dwLookup,  timeTripCounts);
            tCar.returnHome = false;
        }

        // determine if we need to refuel
        if (tCar.gas < 40)
        {
            tCar.refuel = 48; // Changed to 48 from 2 (wait time is now 4 hours instead of 10 minutes)
            tCar.inUse = true;
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

void genRetTrip (int ox, int oy, int dx, int dy, int t, double dwLookup [][288], int* timeTripCounts)
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

//        cout << "Generating a return trip at T = " << newT << " from " << nTrip.startX << "," << nTrip.startY << " to " << nTrip.endX << "," << nTrip.endY << endl;

        placeInTTM ( nTrip, timeTripCounts, newT);
    }

    return;
}

void findCOV (double& COVF, double* COVray, int numRuns)
{
    double avgRay = 0;

    COVF = 0;

    for (int i = 0; i < numRuns; i++)
    {
        avgRay = avgRay + COVray[i];
    }

    avgRay = avgRay / numRuns;

    for (int i = 0; i < numRuns; i++)
    {
        COVF = COVF + pow(COVray[i] - avgRay, 2);
    }

    // coefficient of variation, std. deviation divided by the mean
    COVF = sqrt(COVF / numRuns) / avgRay;
}


// writes time trip counts to output file
void writeTimeTripCounts(ofstream& outfile, int* timeTripCounts)
{
    for (int t = 0; t < 288; t++)
    {
        outfile << timeTripCounts[t] << endl;
    }

    return;
}

// writes time trip matrix to output file
void writeTimeTripMatrix(ofstream& outfile, int* timeTripCounts)
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
void writeNumCars(ofstream& outfile)
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

void writeNumFreeCars(ofstream& outfile,  std::vector<Car> CarMx[][yMax])
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

void writeMaxCarUse(double* maxCarUse, double* maxCarOcc)
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


// writes car matrix to output file
void writeCarMx(ofstream& outfile,  std::vector<Car> CarMx[][yMax])
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

// writes car matrix to output file
void printZones(ofstream& outfile, int zones[xMax][yMax], int destZones[xMax][yMax])
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

// reads time trip counts from a file created during a previous iteration
void readTimeTripCounts(ifstream& infile, int* timeTripCounts)
{
    for (int t = 0; t < 288; t++)
    {
        infile >> timeTripCounts[t];
    }

    return;
}

// reads time trip matrix from a file created during a previous iteration
void readTimeTripMatrix(ifstream& infile, int* timeTripCounts)
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
void readNumCars(ifstream& infile)
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
void readCarMx(ifstream& infile, std::vector<Car> CarMx[][yMax])
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

void showWaitCars(int t, vector<Trip> waitList [6], int* waitListI, std::vector<Car> CarMx[][yMax])
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
