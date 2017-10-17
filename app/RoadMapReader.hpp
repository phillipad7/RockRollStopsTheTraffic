// RoadMapReader.hpp
//
// ICS 46 Spring 2017
// Project #4: Rock and Roll Stops the Traffic
//
// The RoadMapReader class provides an object that knows how to read a
// RoadMap from the standard input, using the format given in the
// project write-up.

#ifndef ROADMAPREADER_HPP
#define ROADMAPREADER_HPP

#include "RoadMap.hpp"
#include "InputReader.hpp"



class RoadMapReader 
{
public:
    // readRoadMap() reads a RoadMap from the given InputReader.  The
    // RoadMap is expected to be described in the format given in the
    // project write-up.
    RoadMap readRoadMap(InputReader& in);	

    // Note: 	
    // RoadMap.hpp
	// typedef Digraph<std::string, RoadSegment> RoadMap;
	// 
	// struct RoadSegment
	// {
	//     double miles;
	//     double milesPerHour;
	// };

    // Trip
	// struct Trip
	// {
	//     int startVertex;
	//     int endVertex;
	//     TripMetric metric;
	// };

    // TripMetric.hpp
	// enum class TripMetric
	// {
	//     Distance,
	//     Time
	// };








};



#endif // ROADMAPREADER_HPP

