#include <cmath>
#include <iostream>
#include <algorithm>
#include <deque>
#include <set>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <fstream>
#include "PathFinder.h"


using namespace std;

PathFinder::RetVal PathFinder::SetMapSize(uint32_t m, uint32_t n)
{
	if (m > MAP_DIMENTION_LIMIT || n > MAP_DIMENTION_LIMIT)
		return FAIL_PARAM;

	m_map.clear();

	M = m;
	N = n;

	for (uint32_t y = 0; y < M; y++)
	{
		vector<Cell> row;
		for (uint32_t x = 0; x < N; x++)
		{
			Cell cell;
			cell.Val = CELL_UNOCCUPIED_CODE;
			row.push_back(cell);
		}
		m_map.push_back(row);
	}

	return SUCCESS;
}

PathFinder::RetVal PathFinder::SetRobotRadius(double robotRadius)
{
	if (robotRadius > M || robotRadius > N)
		return FAIL_PARAM;

	if (robotRadius <= 0.0)
		return FAIL_PARAM;

	m_robotRadius = robotRadius;
	return SUCCESS;
}

PathFinder::RetVal PathFinder::SetHeuristicWeight(double heuristicWeight)
{
	if (heuristicWeight < 0.0)
		return FAIL_PARAM;

	m_heuristicWeight = heuristicWeight;
	return SUCCESS;
}

PathFinder::RetVal PathFinder::SetSafetyMargin(double safetyMargin)
{
	if (safetyMargin < 0.0)
		return FAIL_PARAM;

	m_safetyMargin = safetyMargin;
	return SUCCESS;
}

double PathFinder::distanceBetween(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
{
	uint32_t x = x1 - x2;
	uint32_t y = y1 - y2;
	return sqrt(x * x + y * y);
}

double PathFinder::distanceBetween(Coordinate& a, Coordinate& b)
{
	return distanceBetween(a.X, a.Y, b.X, b.Y);
}

PathFinder::RetVal PathFinder::AddObjects(std::vector<Object>& objects)
{
	if (objects.size() > MAX_OBJECTS)
		return FAIL_PARAM;

	for (auto object : objects)
	{
		if (object.Origin.X > N - 1 || object.Origin.X < 0) return FAIL_PARAM;
		if (object.Origin.Y > M - 1 || object.Origin.Y < 0) return FAIL_PARAM;
		if (object.Radius > M || object.Radius > N || object.Radius <= 0) return FAIL_PARAM;

		// Paint object onto the map
		for (uint32_t y = 0; y < M; y++) // i is row number (y)
		{
			for (uint32_t x = 0; x < N; x++) // j is column number (x)
			{
				double distance = distanceBetween(object.Origin.X, object.Origin.Y, x, y);
				if (distance <= object.Radius)
				{
					m_map[y][x].Val = CELL_OCCUPIED_CODE;
				}
			}
		}
	}
	m_objects = objects;
	return SUCCESS;
}

void PathFinder::DisplayMap()
{
	for (uint32_t y = 0; y < M; y++)
	{
		for (uint32_t x = 0; x < N; x++)
		{
			cout << m_map[y][x].Val;
		}
		cout << endl;
	}
}

bool PathFinder::isTraversable(Coordinate& c)
{
	// Would the robot collide with an edge of the map if it moved to c?
	if (c.X - (m_robotRadius + m_safetyMargin) < 0 || c.Y - (m_robotRadius + m_safetyMargin) < 0) return false;
	if (c.X + (m_robotRadius + m_safetyMargin) >= N || c.Y + (m_robotRadius + m_safetyMargin) >= M) return false;

	// Would the Robot collide with any objects if it moved to c?
	for (auto object : m_objects)
	{
		if (distanceBetween(c, object.Origin) < (m_robotRadius + m_safetyMargin) + object.Radius)
			return false;
	}

	return true;
}

double PathFinder::heuristic(Coordinate& a, Coordinate& dest, Coordinate& start)
{
	return m_heuristicWeight*(distanceBetween(a, start) + distanceBetween(a, dest));
}

PathFinder::RetVal PathFinder::constructPath(Coordinate& dest, Coordinate& start, vector<Coordinate>& path)
{
	Coordinate c = dest;
	while (c != start) {
		path.push_back(c);
		m_map[c.Y][c.X].Val = CELL_BEST_PATH_CODE;
		c = m_map[c.Y][c.X].Parent;
	}
	m_map[dest.Y][dest.X].Val = CELL_DESTINATION_CODE;
	m_map[start.Y][start.X].Val = CELL_START_CODE;
	path.push_back(start);
	reverse(path.begin(), path.end());
	return SUCCESS;
}

void PathFinder::getNeighbors(Coordinate& c, vector<Coordinate>& neighbors)
{
	for (uint32_t y = c.Y - NEIGHBOR_RANGE; y <= c.Y + NEIGHBOR_RANGE; y++)
	{
		for (uint32_t x = c.X - NEIGHBOR_RANGE; x <= c.X + NEIGHBOR_RANGE; x++)
		{
			Coordinate n = { x, y };
			neighbors.push_back(n);
		}
	}
}

// Using the A* algorithm
PathFinder::RetVal PathFinder::FindPath(Coordinate start, Coordinate dest, vector<Coordinate>& bestPath)
{
	if (!isTraversable(start) || !isTraversable(dest))
		return FAIL_PARAM;

	set<Coordinate> open; // The set of nodes to be evaluated
	set<Coordinate> closed; // The set of nodes already evaluated

	open.insert(start);
	m_map[start.Y][start.X].f_cost = 0;
	m_map[start.Y][start.X].g_score = 0;

	while (1)
	{

		//Find next node
		Coordinate current;
		for (auto c : open)
			if (m_map[c.Y][c.X].f_cost < m_map[current.Y][current.X].f_cost) 
				current = c;

		if (open.empty()) return FAIL_NO_PATH;

		open.erase(current);
		closed.insert(current);

		if (current == dest) return constructPath(dest, start, bestPath);

		vector<Coordinate> neighbors;
		getNeighbors(current, neighbors);

		for (auto neighbor : neighbors)
		{
			if (false == isTraversable(neighbor) || closed.find(neighbor) != closed.end())
				continue;

			double tentativeGScore = m_map[current.Y][current.X].g_score + distanceBetween(current, neighbor);

			if (tentativeGScore >= m_map[neighbor.Y][neighbor.X].g_score)
				continue;

			if (tentativeGScore < m_map[neighbor.Y][neighbor.X].g_score)
			{
				m_map[neighbor.Y][neighbor.X].Parent = current;
				m_map[neighbor.Y][neighbor.X].g_score = tentativeGScore;
				m_map[neighbor.Y][neighbor.X].f_cost = m_map[neighbor.Y][neighbor.X].g_score 
					+ heuristic(neighbor, dest, start);
				if (open.find(neighbor) == open.end())
				{
					open.insert(neighbor);
				}
			}
		}
	}
}

PathFinder::RetVal PathFinder::SaveMap(std::string path)
{
	ofstream out;
	out.open(path);
	for (uint32_t y = 0; y < M; y++)
	{
		for (uint32_t x = 0; x < N; x++)
		{
			char c = m_map[y][x].Val;
			if (c == CELL_BEST_PATH_CODE ||
				c == CELL_DESTINATION_CODE ||
				c == CELL_START_CODE)
			{
				c = CELL_UNOCCUPIED_CODE;
			}

			out << c;
		}
		out << endl;
	}
	out << OBJECTS_TAG << endl;
	for(auto object : m_objects)
	{
		out << object.Origin.X << endl;
		out << object.Origin.Y << endl;
		out << object.Radius << endl;
	}

	out.close();
	return SUCCESS;
}

PathFinder::RetVal PathFinder::LoadMap(std::string path)
{
	vector<vector<Cell>> map;
	ifstream in(path);

	string line;
	while (getline(in, line))
	{
		if(line == OBJECTS_TAG) break;

		vector<Cell> row;
		for (char c : line)
		{
			Cell cell;
			cell.Val = c;
			row.push_back(cell);
		}
		map.push_back(row);
	}

	Object object;
	uint32_t i = 0;
	while (getline(in, line))
	{
		switch(i % 3)
		{
			case 0: object.Origin.X = atoi(line.c_str()); break;
			case 1: object.Origin.Y = atoi(line.c_str()); break;
			case 2: object.Radius = stod(line.c_str()); m_objects.push_back(object); break;
		}
		i++;
	}
	in.close();

	// Check for no rows
	if(map.size() < 1) return FAIL_BAD_FILE;

	// Make sure row lengths are all the same.
	if (map.size() > 1)
	{
		uint32_t rowLength = map[0].size();

		if(rowLength < 1) return FAIL_BAD_FILE;

		for (auto row : map)
		{
			if (row.size() != rowLength) return FAIL_BAD_FILE;
		}
	}

	m_map = map;
	M = map.size();
	N = map[0].size();
	return SUCCESS;
}
