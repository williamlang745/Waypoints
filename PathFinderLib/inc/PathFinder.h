#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <float.h>

struct Coordinate
{
	uint32_t X = 0;
	uint32_t Y = 0;

	// The operator overloads are needed for the sets used in the A* implementation
	bool operator==(const Coordinate& c) const { return c.X == X && c.Y == Y; }
	bool operator!=(const Coordinate& c) const { return c.X != X || c.Y != Y; }
	bool operator< (const Coordinate& c) const { return (c.X < X) || ((!(X < c.X)) && (c.Y < Y)); }
};

class PathFinder
{

public:

	enum RetVal
	{
		SUCCESS,
		FAIL,
		FAIL_MEMORY,
		FAIL_PARAM,
		FAIL_NO_PATH,
		FAIL_BAD_FILE
	};
	
	struct Object
	{
		Coordinate Origin; // Position of the object
		double Radius;	// Radius of the object
	};

	/*
	* Clears the current map and re-initializes it with m rows and n columns.
	*/
	RetVal SetMapSize(uint32_t m, uint32_t n);

	/*
	* Sets the radius of the robot traversing the map. 
	*/
	RetVal SetRobotRadius(double robotRadius);

	/*
	* In the A* algorithm, the heuristic function for a node is usually defined as the addition of the nodes distance between 
	* the start and the destination node. Applying a higher weight to that meteric will cause the A* algorithm to prioritize 
	* paths that fall on a staight line between the start and destination nodes. This can help the performance of the algorithm 
	* but the path taken becomes less ideal.
	*
	* The huristic weight is initialized to 0. To enable the heuristic call this method with a non-negetive number. ex. 1.0.
	*/
	RetVal SetHeuristicWeight(double heuristicWeight);

	/*
	* The safety margin dictates how close the robot is allowed to get to an edge before. The default safty margin is 0.0.
	*/
	RetVal SetSafetyMargin(double safetyMargin);

	/*
	* Populates the map with the objects.
	*/
	RetVal AddObjects(std::vector<Object>& objects);

	/*
	* Runs the A* algorithm to find the best path from robot to destination.
	*
	* @param in robot - The Coordinate representing the robot starting point. 
	* @param in destination - The Coordinate representing the destination.
	* @param out bestPath - The series of Coordinates representing the best path from robot to destination.
	*/
	RetVal FindPath(Coordinate robot, Coordinate destination, std::vector<Coordinate>& bestPath);

	/*
	* Saves the currently loaded map to the file defined by path as plain text.
	*/
	RetVal SaveMap(std::string path);

	/*
	* Clears the currently loaded map if any, and loads the map saves in file path.
	*/
	RetVal LoadMap(std::string path);

	/*
	* Displays the currently loaded map to the screen as text. Each character printed represents a cell of the map.
	*/
	void DisplayMap();


private:

	struct Cell
	{
		char Val;
		Coordinate Parent;
		double g_score = DBL_MAX;
		double f_cost = DBL_MAX;
	};

	static const uint32_t MAP_DIMENTION_LIMIT = 1028;
	static const uint32_t MAX_OBJECTS = 1028;
	static const char CELL_OCCUPIED_CODE	= '1';
	static const char CELL_UNOCCUPIED_CODE	= '0';
	static const char CELL_DESTINATION_CODE = 'D';
	static const char CELL_START_CODE		= 'S';
	static const char CELL_BEST_PATH_CODE	= '*';
	static const uint32_t NEIGHBOR_RANGE = 1;
	static constexpr const char* OBJECTS_TAG = "OBJECTS:";

	uint32_t M;	// map height - rows
	uint32_t N; // map width - columns

	std::vector<std::vector<Cell>> m_map;

	double m_robotRadius;
	std::vector<Object> m_objects;
	double m_heuristicWeight = 0.0;
	double m_safetyMargin = 0.0;

	static double distanceBetween(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2);
	static double distanceBetween(Coordinate& a, Coordinate& b);
	bool isPathTraversable(Coordinate& c, Coordinate& current);
	bool isTraversable(Coordinate& c);
	double heuristic(Coordinate& a, Coordinate& dest, Coordinate& start);
	RetVal constructPath(Coordinate& dest, Coordinate& start, std::vector<Coordinate>& path);
	void getNeighbors(Coordinate& c, std::vector<Coordinate>& neighbours);
};
