// RoadMap.cpp
//
// ICS 46 Spring 2017
// Project #4: Rock and Roll Stops the Traffic

#include <algorithm>
#include <sstream>
#include "RoadMapReader.hpp"


RoadMap RoadMapReader::readRoadMap(InputReader& in)
{
    RoadMap roadMap;                // RoadMap is typeof Digraph<std::string, RoadSegment>

    int numberOfLocations = in.readIntLine();               // 1st line of input: int 15

    for (int i = 0; i < numberOfLocations; ++i)
    {
        roadMap.addVertex(i, in.readLine());                // 15 locations as Vertex
    }

    int numberOfRoadSegments = in.readIntLine();

    for (int i = 0; i < numberOfRoadSegments; ++i)
    {
        std::istringstream roadSegmentLine{in.readLine()};

        int fromLocation;
        int toLocation;
        double miles;
        double milesPerHour;

        roadSegmentLine >> fromLocation >> toLocation >> miles >> milesPerHour;

        roadMap.addEdge(fromLocation, toLocation, RoadSegment{miles, milesPerHour});
    }

    return roadMap;     // Digraph<std::string, RoadSegment>
}

