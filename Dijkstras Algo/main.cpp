#include<algorithm>
#include<iostream>
#include<utility>
#include<vector>
#include<queue>
#include<map>
#include<set>

using namespace std;

//structure to hold node data 
struct NodeData
{
	int distanceFromStart;
	pair<int, int> coordinate;
	pair<int, int> parentNode;

	NodeData() : distanceFromStart(0), coordinate({ 0,0 }), parentNode({ 0,0 })
	{
	}
};

struct comp {
	bool operator()(struct NodeData a, struct NodeData b) {
		return (a.distanceFromStart > b.distanceFromStart);
	}
};

int ConvertToVectorIndex(pair<int, int> point, pair<int, int> mapDomensions)
{
	return (mapDomensions.second * point.first) + point.second;
}

/// <summary>
/// distance between two points on the grid using manhattan distance
/// Manhattan distance was recommended for a grid based problem and so i chose it
/// </summary>
/// <param name="start"></param>
/// <param name="end"></param>
/// <returns></returns>
int FindDistanceBetween(pair<int, int> start, pair<int, int> end)
{
	start.first = abs(start.first - end.first);
	start.second = abs(start.second - end.second);

	return start.first + start.second;
}

/// <summary>
/// get the corresponding nodedata for a given point in grid
/// </summary>
/// <param name="point"></param>
/// <returns></returns>
NodeData GetNode(pair<int, int> point, const vector<NodeData>& nodeData)
{
	for (const NodeData& node : nodeData)
	{
		if (node.coordinate == point)
		{
			return node;
		}
	}
	return nodeData[0];
}

/// <summary>
/// updates distance and previous shortest path(a node stores previous node so we can chain the path) for neighbours of a point if 
/// </summary>
/// <param name=""></param>
/// <param name=""></param>
NodeData UpdateDistanceAndParent(pair<int, int> neighbour, pair<int, int> current, const vector<NodeData>& nodeData)
{
	NodeData neighbourNode = GetNode(neighbour, nodeData);
	NodeData currentNode = GetNode(current, nodeData);

	int distanceFromCurrent = FindDistanceBetween(neighbour, current);
	int distanceFromStart = currentNode.distanceFromStart;
	int newDistance = distanceFromCurrent + distanceFromStart;

	if (newDistance < neighbourNode.distanceFromStart)
	{
		neighbourNode.distanceFromStart = newDistance;
		neighbourNode.parentNode = current;
	}
	return neighbourNode;
}

/// <summary>
/// to make the code simpler i used matrix to find path and this will give me
/// the exact index on Map according to the cartesian plane wherever needed
/// </summary>
/// <param name="coordinates"></param>
/// <returns></returns>
int ConvertToCartesianIndex(pair<int, int> coordinates, pair<int,int> mapDimansions)
{
	return (mapDimansions.first * coordinates.second) + coordinates.first;
}

/// <summary>
/// check if a point is in bounds with our map dimensions
/// </summary>
/// <param name="point"></param>
/// <returns></returns>
bool CheckBounds(pair<int, int> point, pair<int, int> mapDimansions)
{
	if (point.first >= 0 && point.second >= 0)
	{
		if (point.first < mapDimansions.first && point.second < mapDimansions.second)
		{
			return true;
		}
	}
	return false;
}

/// <summary>
///	if a point on the grid is explored return false
/// </summary>
bool CheckIfPointIsUnexplored(pair<int, int> point, const vector<pair<int, int>>& exploredPoints)
{
	for (pair<int, int> coordinate : exploredPoints)
	{
		if (point == coordinate)
			return false;
	}
	return true;
}

/// <summary>
///	find neighbours for a point on grid
/// </summary>
void PopulateNeighbours(pair<int, int> currentPoint, set<pair<int, int>>& Neighbours, pair<int, int> mapDimensions, const vector<bool> Map, vector<pair<int, int>> expleoredPoints)
{
	Neighbours.clear();

	pair<int, int> temp;
	bool isTraversible;

	temp = make_pair(currentPoint.first + 1, currentPoint.second);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToCartesianIndex(temp, mapDimensions));
		if (isTraversible) Neighbours.insert(temp);
	}

	temp = make_pair(currentPoint.first - 1, currentPoint.second);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToCartesianIndex(temp, mapDimensions));
		if (isTraversible) Neighbours.insert(temp);
	}

	temp = make_pair(currentPoint.first, currentPoint.second + 1);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToCartesianIndex(temp, mapDimensions));
		if (isTraversible) Neighbours.insert(temp);
	}

	temp = make_pair(currentPoint.first, currentPoint.second - 1);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToCartesianIndex(temp, mapDimensions));
		if (isTraversible) Neighbours.insert(temp);
	}
}

/// <summary>
/// Dijkstra's Pathfinding Algo
/// </summary>
/// <param name="start"></param>
/// <param name="target"></param>
/// <param name="grid"></param>
/// <param name="gridDimensions"></param>
/// <param name="path"></param>
/// <returns></returns>
bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<bool>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath)
{
	if (!CheckBounds(Start, MapDimensions) || !CheckBounds(Target, MapDimensions))
	{
		return false;
	}

	priority_queue<NodeData, vector<NodeData>, comp> PQ;
	NodeData point;
	set<pair<int, int>> Neighbours;
	vector<pair<int, int>> exploredPoints;
	vector<NodeData> vec_NodeData;

	point.distanceFromStart = FindDistanceBetween(make_pair(0, 0), MapDimensions) * 10;

	//distance = max
	//coordinates in given plane
	//parent coordinate = 0,0
	for (int i = 0; i < MapDimensions.first; i++)
	{
		for (int j = 0; j < MapDimensions.second; j++)
		{
			point.coordinate = make_pair(i, j);
			if (point.coordinate == Start)
			{
				point.distanceFromStart = 0;
			}
			vec_NodeData.push_back(point);
			if (point.coordinate == Start)
			{
				point.distanceFromStart = FindDistanceBetween(make_pair(0, 0), MapDimensions) * 10;
			}
		}
	}

	int indexOfStart = ConvertToVectorIndex(Start, MapDimensions);
	PQ.push(vec_NodeData[indexOfStart]);

	while (!PQ.empty())
	{
		//since it's priority queue the least distant node will be at it's top
		pair<int, int> currentNode = PQ.top().coordinate;
		exploredPoints.push_back(currentNode);
		PQ.pop();

		if (currentNode == Target)
		{
			//trace back the shortest path
			pair<int, int> pathTrace = Target;
			while (pathTrace != Start)
			{
				OutPath.push_back(ConvertToCartesianIndex(pathTrace, MapDimensions));
				pathTrace = GetNode(pathTrace, vec_NodeData).parentNode;
			}
			reverse(OutPath.begin(), OutPath.end());
			return true;
		}

		PopulateNeighbours(currentNode, Neighbours, MapDimensions, Map, exploredPoints);

		//update each neighbour distance and parent if necessary
		for (pair<int, int> neighbour : Neighbours)
		{
			NodeData tempNode = UpdateDistanceAndParent(neighbour, currentNode, vec_NodeData);
			
			int i = ConvertToVectorIndex(tempNode.coordinate, MapDimensions);

			if (vec_NodeData[i].distanceFromStart > tempNode.distanceFromStart)
			{
				vec_NodeData[i].distanceFromStart = tempNode.distanceFromStart;
				vec_NodeData[i].parentNode = tempNode.parentNode;
				PQ.push(vec_NodeData[i]);
			}
		}
	}
	return false;
}

//driver code
int main()
{
	//Globals Variables
	pair<int, int> MapDimensions(4,3);
	pair<int, int> Target(1,2);
	pair<int, int> Start(3,2);
	vector<int> Path;
	vector<bool> Map = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };

	FindPath(Start, Target, Map, MapDimensions, Path);

	return 0;
}