// main.cpp
//
// ICS 46 Spring 2017
// Project #4: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.
# include <iomanip>
# include <cmath>
# include <iostream>
# include "Digraph.hpp"
# include "RoadMapReader.hpp"	//
# include "TripReader.hpp" 		// take in cin, return vector of trips [from_vec, to_vec, DorT]
# include "InputReader.hpp"
# include "Trip.hpp"


std::function<double(const RoadSegment&)> shortestByPath = [](RoadSegment rs) {return rs.miles;};						//s
std::function<double(const RoadSegment&)> shortestByTime = [](RoadSegment rs) {return rs.miles / rs.milesPerHour;};		//t


struct FullTrip
{
	double totalDist;						// provide total travel distance
	double totalTime;						// provide total travel time	
	std::vector<int> 	tripRoutes;			// provide all vertices passed by
	std::vector<double> segLeng;			// length of each road segment
	std::vector<double> segSped;			// speed of each road segment
	std::vector<double> segTime;			// time of each road segment
	TripMetric metric;
};


void printFinal(RoadMap rdMap, std::vector<FullTrip> allTrips);
void printResultDist(RoadMap rdMap, FullTrip oneTrip);
void printResultTime(RoadMap rdMap, FullTrip oneTrip);
void printTime(double time);
FullTrip getherInfo(RoadMap rdMap, Trip oneTrip);


int main()
{
	InputReader ir = InputReader(std::cin);
	RoadMapReader rr;
	TripReader tr;
	RoadMap rdMap = rr.readRoadMap(ir);						// rMap is type of RoadMap which is Digraph.
	std::vector<Trip> tripVecs = tr.readTrips(ir);			// Trip {startVertex, endVertex, TripMetric}
	std::vector<FullTrip> allTrips;
	
	for(auto oneTrip: tripVecs)								// oneTrip{startVertex, endVertex, metric}
	{
		FullTrip singleTrip = getherInfo(rdMap, oneTrip);
		allTrips.push_back(singleTrip);						// ALL TRIPS INFO STORED HERE
	}
	
	printFinal(rdMap, allTrips);							// display all info regarding all trips

    return 0;
}


void printFinal(RoadMap rdMap, std::vector<FullTrip> allTrips)
{
	for(auto oneTrip: allTrips)
	{
		if(oneTrip.metric == TripMetric::Distance)
			printResultDist(rdMap, oneTrip);
		else
			printResultTime(rdMap, oneTrip);
	}
}


void printTime(double time)
{
	int hrs = std::floor(time);
	time = (time - hrs) * 60;
	int min = std::floor(time);
	time = (time - min) * 60;
	int sec = std::floor(time);
	int res = std::round((time-sec)*100);

	if(hrs >0)
	{
		std::cout<<hrs<<" hrs "<<min<<" mins "<<sec<<"."<<res<<" secs"<<std::endl;
	}
	else
	{
		if(min > 0)
		{
			std::cout<<min<<" mins ";
		}
		std::cout<<sec<<"."<<res<<" secs";
	}
}


void printResultDist(RoadMap rdMap, FullTrip oneTrip)
{
	int endpt = oneTrip.tripRoutes[0];
	int start = oneTrip.tripRoutes[oneTrip.tripRoutes.size()-1];

	std::cout<<"\nShortest distance from "<<rdMap.vertexInfo(start)<<" to "<<rdMap.vertexInfo(endpt)<<std::endl;
	std::cout<<"  Begin at "<<rdMap.vertexInfo(start)<<std::endl;
	for(int i=oneTrip.tripRoutes.size()-2; i>=0;i--)
	{
		std::cout<<"  Continue to ";
		std::cout<<rdMap.vertexInfo(oneTrip.tripRoutes[i])<<" (";
		std::cout<<oneTrip.segLeng[i]<<" miles)"<<std::endl;
	}
	std::cout<<"Total distance: "<<oneTrip.totalDist<<" miles\n"<<std::endl;
}


void printResultTime(RoadMap rdMap, FullTrip oneTrip)
{
	int endpt = oneTrip.tripRoutes[0];
	int start = oneTrip.tripRoutes[oneTrip.tripRoutes.size()-1];

	std::cout<<"Shortest driving time from "<<rdMap.vertexInfo(start)<<" to "<<rdMap.vertexInfo(endpt)<<std::endl;
	std::cout<<"  Begin at "<<rdMap.vertexInfo(start)<<std::endl;
	for(int i=oneTrip.tripRoutes.size()-2; i>=0;i--)
	{
		std::cout<<"  Continue to ";
		std::cout<<rdMap.vertexInfo(oneTrip.tripRoutes[i])<<" (";
		std::cout<<oneTrip.segLeng[i]<<" miles @ "<<oneTrip.segSped[i]<<" mph = ";
		printTime(oneTrip.segTime[i]);
		std::cout<<")"<<std::endl;
	}
	std::cout<<"Total time: ";
	printTime(oneTrip.totalTime);
	std::cout<<"\n"<<std::endl;
}


FullTrip getherInfo(RoadMap rdMap, Trip oneTrip)
{
	std::map<int,int> shortPaths;

	if(oneTrip.metric == TripMetric::Distance)
	{
		shortPaths = rdMap.findShortestPaths(oneTrip.startVertex, shortestByPath);
	}
	else
	{
		shortPaths = rdMap.findShortestPaths(oneTrip.startVertex, shortestByTime);	
	}

	int prePoint = oneTrip.endVertex;					// This is the end point

	// arguments for struct FullTrip
	double totalDist = 0;
	double totalTime = 0;
	std::vector<double> segLeng;						// length of each road segment
	std::vector<double> segTime;						// time of each road segment
	std::vector<double> segSped;						// speed of each road segment
	std::vector<int> 	tripRoutes;						// provide all vertices passed by
	tripRoutes.push_back(prePoint);						// push end Point to trip routes

	// in oneTrip, loop through each stops
	while(prePoint != oneTrip.startVertex)
	{
		int curPoint = prePoint;						// currentPoint 
		prePoint = shortPaths[prePoint];				// previous Point before currentPoint
		tripRoutes.push_back(prePoint);					// push selected Point to trip routes
		
		double s = rdMap.edgeInfo(prePoint, curPoint).miles;
		double v = rdMap.edgeInfo(prePoint, curPoint).milesPerHour;
		double t = s / v;
		
		segLeng.push_back(s);
		segSped.push_back(v);
		segTime.push_back(t);
		totalDist += s;
		totalTime += t;
	}
	return {totalDist, totalTime, tripRoutes, segLeng, segSped, segTime, oneTrip.metric};		//FullTrip stucture
}
