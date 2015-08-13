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
#include <sstream>



#ifndef SIMULATOR_H
#define SIMULATOR_H


//#define LARGE 400
//#define SMALL 40
#define GREEDY 0
#define SCRAM 1
#define HUNGARIAN 2
//#define SAV 2
//#define SAEV 3
#define SEPARATE 0
#define MERGE 1
#define CONSTANT 0
#define VARY 1
#define SUPPLY_PRICING 0
#define CHARGE_PRICING 1

//#define SIZE SMALL // 40 x 40 or 400 x 400 (Changes how trip generation rates are handled)
//#define ALGORITHM GREEDY // Matching is done with either the original greedy approach or SCRAM
//#define SPEED CONSTANT
//#define SIMULATOR SAV // Sets car ranges and fuel times for either electric or gas vehicles
//#define WAIT SEPARATE // Refers to giving all unmatched trip equal priority or separate
//#define CHARGE_IN_PLACE true
//#define SUPPLY_PRICING false
//#define CHARGE_PRICING false
//#define REALLOCATION_ON true


using namespace std;

struct Station;

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
    bool isBusiness;
    int modeOfTransit;
    double price;
    double VOTT;
    double V_pv;
    double V_tr;
    double sb;
    double sB;
    double db;
    double dB;
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
    int numRejects;
    Trip* currTrip;
    bool stuck;
    bool stationLink;
    bool needFuel;
};

struct Station
{
        int x;
        int y;
        int startX;
        int startY;
        int numCharges;
        int chargeTime;
        int unoccupiedDist;
        int congestTime;
};


class Simulator{

	private:

		int xMax;// = 40; //SIZE;
		int yMax;// = 40; //xMax; // 40
		//int TTMxSize = 4000;
		//const int CarMxSize = 500;
		//const int WaitListSize = 4000;
		double base_price;// = 0.85 / 4;
		double saev_vott;// = 0.35;
		char* mode_output;// = "modeChoiceStats_supply_charge.csv";

		int numZonesL;// = 50;
		int numZonesS;// = 50;
		double nearDist;// = 60/2; // 10
		double innerDist;// = 20/2;
		double outerDist;// = SIZE;
		int tripDistSize;// = 60;
		//int personalTripSize;// = 484;
		//int businessTripSize;// = 926;

		int zoneSizeL;// = xMax / numZonesL; // 8
		int zoneSizeS;// = xMax / numZonesS; // 4
		int maxNumRuns;// = 50;
		int numWarmRuns;// = 20;

		int carRange;// = 1600; //1600;//320;
		int refuelTime;// = 2; // 48 -- should be two for normal
		int refuelDist;// = 0; //40
		int rejectLimit;// = 1000000000; // Really high means it won't be used
		float rangePercent;// = 1.0;
		//const int carRange = 320; // 320 = 80 miles or 1000 = 250 miles
		//const int refuelTime = 48; // 48 = 4 hours or 6 = 30 minutes
		//const int refuelDist = 0; // Refuels if only 40 cells = 10 miles left, set to 0 to disable
		//const int rejectLimit = 2; // Set to 1000000000 to disable
		//const float rangePercent = 0.75;
		//#endif

		// Parameters I added to input.txt
		char businessVottDistributionFile[40];
		char personalVottDistributionFile[40];
		char tripDistDistributionFile[40];
		double personalTripProb;
		double transitPrice;
		int ridePriceScheme;
		double rideBasePrice;
		int matchAlgorithm;
		bool tripPool;
		int carSpeed;
		bool reassignTrips;
		bool reallocationOn;
		bool canRefuelAnywhere;
		bool useModeChoice;
		int numRejectsToRefuel;
		double rangePercentToRefuel;
		bool useCityTripData;
		int rseed;
		char* readName;
		vector<int> random_seeds;
		float cellSize;

		long totDistRun;
		long totUnoccDistRun;
		long totReallocDistRun;
		long totCarsRun;
		long totTripsRun;
		long totHSRun;
		long totCSRun;
		long totWaitTRun;
		long totUnservedTRun;
		long totUnusedRun; 
		long totUnoccRun;
		long maxAvailCars;
		long maxAvailStations;
		long totWaitCountRun[6];
		long totUnoccChargeDistRun;
		long totChargeTimeRun;
		long totNumChargeRun;
		long totCongestTimeRun;
		long totMaxCarInUse;
		long totMaxCarCharging;
		long totMaxCarNoUse;
		long totMaxInUseCharge;
		long totMaxChargingCharge;
		long totMaxNoUseCharge;
		double revenue;
		double totRevenueRun;

		vector<Car>** CarMx;//[xMax][yMax];
		vector<Trip> TTMx[288];
		vector <Station>** ChStMx;//[xMax][yMax];
		vector<int> **cellChargeCount;
	        vector<int> **cellChargeTime;
		int timeTripCounts [288]; // array noting number of trips in each time bin
		double startTimes [288];
		double *tripDist;
		double *businessTripProbability;
		double *businessTripVOTT;
		double *personalTripProbability;
		double *personalTripVOTT;
		double dwLookup [8][288];
		double **zoneSharesL; // These are declared bigger than they need to be. 
		double **zoneSharesS; // This allows only one function to be used for them
                                        // Two functions this is done for reallocVehsZonesL which could be changed only S zones
                                        // Other is setZoneShares. This could be divided into two functions easily

		double **cityTripRates;
                int *xMap;
		int *yMap;
                int numTripOrigins;

// arrays for calculating COVs
		vector<double> totRevenueCOV;
		vector<double> totMaxCarInUseCOV;
		vector<double> totMaxCarChargingCOV;
		vector<double> totMaxCarNoUseCOV;
		vector<double> totMaxInUseChargeCOV;
		vector<double> totMaxChargingChargeCOV;
		vector<double> totMaxNoUseChargeCOV;
		vector<double> totDistRunCOV;
		vector<double> totUnoccDistRunCOV;
		vector<double> totReallocDistRunCOV;
		vector<double> totUnoccChargeDistRunCOV;
		vector<double> totChargeTimeRunCOV;
		vector<double> totCongestTimeRunCOV;
		vector<double> totNumChargeRunCOV;
		vector<double> totCarsRunCOV;
		vector<double> totTripsRunCOV;
		vector<double> totHSRunCOV;
		vector<double> totCSRunCOV;
		vector<double> totWaitTRunCOV;
		vector<double> totUnservedTRunCOV;
		vector<double> totWaitCountRunCOV;
		vector<double> totUnusedRunCOV;
		vector<double> totUnoccRunCOV;
		vector<double> totAvgWaitCOV;
		vector<double> totAvgTripDistCOV;
		vector<double> totStartsPerTripCOV;
		vector<double> totAvgTripsPerCarCOV;

		vector<double> totPctMaxWaitFiveCOV;
		vector<double> totPctInducedTCOV;
		vector<double> totPctMaxInUseCOV;
		vector<double> totPctMaxOccCOV;
		vector<double> totPctColdShareCOV;


		bool error, reportProcs, readFile, wStart;
		int maxTrav;
		int maxTravCongested;
		int totDist; 
		int unoccDist;
		int reallocDist;
		int waitT;
		int saveRate;
		int startIter; 
		int unservedT;
		int coldStarts; 
		int hotStarts; 
		int numRuns; 
		int nCars;
		int randomSeed;
		int unoccChargeDist;
		int tripsReassigned;
		int totTripsReassigned;
		int nStations;

		double maxCarUse [288];
		double maxCarOcc [288];

		int maxCarInUse, maxCarCharging, maxCarNoUse;
		int maxInUseCharge, maxChargingCharge, maxNoUseCharge;

		int prev_avail_cars;

		int waitCount[6];
		double outerRate;
		double innerRate;
		double nearRate;
		double exurbanRate; 
		double distWt;
		double netDistWt; 

		double totAvgWait;
		double totAvgTripDist; 
		double totWaitCOV;
		double totTripDistCOV; 
		double totCarTripsCOV;
		//bool printZ = true;
		//char zName [16];

		// Functions for program

	public:
		Simulator();
		Simulator(int xDim, int yDim);
		~Simulator();

		void loadParameters(char* input);
                void printParameters();
		void loadTripRateData(char* rateData, char* cellMap);
		void initVars (int runNum, bool warmStart, bool checkStationDistance);
		void findDistWeight();
		void generateTrips (bool warmStart);
		void generateTripsWithData (bool warmStart);
		void runSharedAV (bool warmStart, bool lastWarm, int iter, bool checkStationDistance);//int* timeTripCounts, int maxTrav, int maxTravC, double dwLookup [][288],
//                  double **zoneSharesL, double **zoneSharesS, double* maxCarUse, double* maxCarOcc, int& totDist, int& unoccDist, int& waitT, bool reportProcs,
  //                int saveRate, bool warmStart, bool lastWarm, long& maxAvailCars, bool& readFile, int startIter, int& unservedT, int* waitCount, int& hotStarts,
    //              int& coldStarts, int nRuns,  int iter, bool checkStationDistance);
		void placeInitCars ();//int* timeTripCounts, double* maxCarUse, double* maxCarOcc, int& totDist,
//                    int& unoccDist, int& waitT, double dwLookup [][288], bool reportProcs, int& hotStarts, int& coldStarts);
		void reportResults (int runNum);//int* timeTripCounts, double* maxCarUse, double* maxCarOcc, int totDist, int unoccDist, int waitT, int unservedT, int* waitCount, int hotStarts, int coldStarts, long& totDistRun, long& totUnoccDistRun, long& totCarsRun, long& totTripsRun, long& totHSRun, long& totCSRun, long& totWaitTRun, long& totUnservedTRun, long* totWaitCountRun, int totUnusedRun, int totUnoccRun, double totAvgWait, double totAvgTripDist, int nRuns, int runNum);
		void reportFinalResults (long totDistRun, long totUnoccDistRun, long totCarsRun, long totTripsRun, long totHSRun, long totCSRun, long totWaitTRun, long totUnservedTRun, long* totWaitCountRun, long totUnusedRun, long totUnoccRun, double totAvgWait, double totAvgTripDist, int numRuns);

		void reportMatchingResults();

		//functions called in initVars
		void setStartTimes(double startTimes [288]);
		void setTripDist();
		void setTripDistLarge();
		void setPersonalTripVOTT();
		void setPersonalTripProbability();
		void setBusinessTripVOTT();
		void setBusinessTripProbability();
		double getBusinessVOTT();
		double getPersonalVOTT();
		void getTripTravelMode(Trip* trip, double saev_wait, double tripdemand_b, double tripdemand, double carsupply_b, double carsupply, bool warmStart);
		double getWait(double dist);
		double getAccessTime(double dist);
		void writeTripsToFile();
		void setDwTimes (double dwLookup [][288]);
		void setZoneSharesS (double **zoneShares, double outerRate, double innerRate, double nearRate, double exurbanRate, int numZones, int zoneSize);
		void setZoneSharesL (double **zoneShares, double outerRate, double innerRate, double nearRate, double exurbanRate, int numZones, int zoneSize);
		void restoreStatus(int* timeTripCounts, int maxTrav,  double* maxCarUse, int& totDist,
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
		void saveStatus(int* timeTripCounts, int maxTrav,  double* maxCarUse, int totDist,
                 int unoccDist, bool reportProcs, int& saveRate, bool warmStart, int tCount);

		void findNearestCar (Trip& trp, int dist, int maxDist,  bool reportProcs,
                     int& nw, int& ne, int& se, int& sw, int& coldStart, int& hotStart, int run, bool checkStationDistance);
		bool lookForCar (int x, int y, int r, int dist, int& cn);
		void assignCar (int x, int y, int c, Trip* trp);
		Car genNewCar (Trip trp);
		void moveCar (int x, int y, int c, int t, int maxTrav, int& totDist, int& unoccDist, int& waitT,
              double dwLookup [][288], int* timeTripCounts, bool reportProcs, int& hotStarts, int& coldStarts, int& trackX, int& trackY, int& trackC, int iter);
		void move (int ox, int oy, int dx, int dy, int c, int t, double dwLookup [][288], int* timeTripCounts,
           bool reportProcs, int& hotStart, int& coldStart, int& trackX, int& trackY, int& trackC, int iter);
		int getCarTrav(int x, int y, int t);
		void genRetTrip (int ox, int oy, int dx, int dy, int t, double dwLookup [][288],  int* timeTripCounts);
		void randOrdering(int* xRandOrd, int numVals);
		void runSummary(bool warmStart, bool lastWarm, int **zoneGen, int **waitZones, double **netZoneBalance, int **tripO, int **tripD,
                int* cardDirectLZ, int* cardDirect, int* cardDirect2,  int* timeTripCounts);
		void reallocVehs(int x, int y, int* timeTripCounts, double dwLookup [][288],
                 bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect, int& trackX, int& trackY, int& trackC, int iter);
		void reallocVehs2(int x, int y, int* timeTripCounts, double dwLookup [][288],
                  bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int* cardDirect2, int& trackX, int& trackY, int& trackC, int iter);
		void reallocVehsLZones (int* timeTripCounts, double dwLookup [][288],
                        double **zoneShares, int t, bool reportProcs, int& totDist, int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav,
                        double **netZoneBalance, int* cardDirectLZ, int **waitZonesT, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC, int iter);

		// function called from reallocVehs2, pushCars & pullCars
		double getSurroundCars (int x, int y);
		double findFreeCars (int x, int y);

		// function called from reallocVehsLZones
		void pushCars(int* timeTripCounts, double dwLookup [][288],
              double **zoneBalance, int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC,int iter);
		void pullCars(int* timeTripCounts, double dwLookup [][288],
              double **zoneBalance, int tx, int ty, double north, double south, double west, double east, int t, bool reportProcs, int& totDist,
              int& unoccDist, int& hotStarts, int& coldStarts, int maxTrav, int* cardDirectLZ, int numZones, int zoneSize, int& trackX, int& trackY, int& trackC, int iter);
		double findMoveDist(int origX, int origY, int direct, int zoneEdge, int maxTrav,  int zoneSize);

		//functions called in reportFinalResults
		void findCOV (double& COVF, vector<double> COVray);

		//File I/O called from saveStatus and restoreStatus
		void writeTimeTripCounts(ofstream& outfile, int* timeTripCounts);
		void writeTimeTripMatrix(ofstream& outfile, int* timeTripCounts);
		void writeNumCars(ofstream& outfile);
		void writeCarMx(ofstream& outfile);
		void writeNumFreeCars(ofstream& outfile);
		void writeMaxCarUse(double* maxCarUse, double* maxCarOcc);
		void writeChargeStats(int stat);

		void readTimeTripCounts(ifstream& infile, int* timeTripCounts);
		void readTimeTripMatrix(ifstream& infile, int* timeTripCounts);
		void readNumCars(ifstream& infile);
		void readCarMx(ifstream& infile);
		//void printZones(ofstream& outfile, int zones[xMax][yMax], int destZones[xMax][yMax]);
		void showWaitCars(int t, vector<Trip> waitList [6], int* waitListI);

		// Functions for matching cars
		void matchTripsToCarsGreedy(vector<Trip> &tripList, int time, int trav, bool reportProcs, int &nw, int &ne, int &se, int &sw, int &coldStarts, int &hotStarts, int run, bool checkStationDistance);
		void matchTripsToCarsScram(vector<Trip> &tripList, int time, int trav, bool reportProcs, int &nw, int &ne, int &se, int &sw, int &coldStarts, int &hotStarts);
		void assignCar (Car* c, Trip* trp);
		void findNearestStation (Car* car, int dist, int maxDist);
		Station* genNewStation(Car car);
		bool lookForStation (int x, int y);
		void assignStation (int x, int y, Car* car);
		void writeStationLocation();
		int nearestStationDistance(int x, int y);
		void writeHeatMap(int run, int time);

		void runSimulation();

};

#endif
