#include "PathFinder.h"
#include <iostream>

using namespace std;

int main()
{
	PathFinder pathFinder;
    PathFinder::RetVal retVal;
    retVal = pathFinder.SetMapSize(64, 64);
    if(PathFinder::SUCCESS != retVal)
    {
        cout << "SetMapSize() failed. ERROR CODE: " << retVal << endl;
        return -1;
    }

	// Uncomment the block below to try loading a map from file. It will overrite the previous map
	/*
    retVal = pathFinder.LoadMap("./map_1.txt");
    if(PathFinder::SUCCESS != retVal)
    {
        cout << "LoadMap() failed. ERROR CODE: " << retVal << endl;
        return -1;
    }
    */

	retVal = pathFinder.SetRobotRadius(4.5);
    if(PathFinder::SUCCESS != retVal)
    {
        cout << "SetRobotSize() failed. ERROR CODE: " << retVal << endl;
        return -1;
    }

    // Set the heuristic weight to 0 to attempt to find the best solution, increase this value to improve performance
	retVal = pathFinder.SetHeuristicWeight(0.75);
    if(PathFinder::SUCCESS != retVal)
    {
        cout << "SetHeuristicWeight() failed. ERROR CODE: " << retVal << endl;
        return -1;
    }

	PathFinder::Object obj1 = { 25, 12, 3.5 };
	PathFinder::Object obj2 = { 5, 5, 7.0 };
	PathFinder::Object obj3 = { 30, 30, 7.0 }; // X, Y, R

	std::vector<PathFinder::Object> objs = { obj1, obj2, obj3 };
	if( PathFinder::SUCCESS != pathFinder.AddObjects(objs))
    {
        cout << "AddObjects() failed. ERROR CODE: " << retVal << endl;
        return -1;
    }

	Coordinate robot = { 50, 10 };
	Coordinate dest = { 15, 15 };

	std::vector<Coordinate> bestPath;
	

    // Run the path finding algorithm
	if (PathFinder::SUCCESS != (retVal = pathFinder.FindPath(robot, dest, bestPath)))
	{
		cout << "FindPath() failed. ERROR CODE: " << retVal << endl;
        return -1;
	}
    
    pathFinder.DisplayMap();

    // Uncomment the below block to try saving the map to file
     
    retVal = pathFinder.SaveMap("./map_1.txt");
    if(PathFinder::SUCCESS != retVal)
    {
        cout << "SetMapSize() failed. ERROR CODE: " << retVal << endl;
        return -1;
    }
    

    // Print bestPath
    for (auto c : bestPath)
        cout << "[" << c.X << "," << c.Y << "] -> ";

	return 0;
}