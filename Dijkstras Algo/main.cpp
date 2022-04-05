#include<algorithm>
#include<iostream>
#include<utility>
#include<vector>
#include<queue>
#include<map>
#include<set>

using namespace std;

//typeDefs
typedef pair<int, int> intPair;

//structure
struct NodeData
{
	int distanceFromStart;
	intPair coordinate;
	intPair parentNode;

	NodeData() : distanceFromStart(0), coordinate({ 0,0 }), parentNode({ 0,0 })
	{
	}
};

struct comp {
	bool operator()(struct NodeData a, struct NodeData b) {
		return (a.distanceFromStart > b.distanceFromStart);
	}
};

int FindDistanceBetween(intPair start, intPair end);
NodeData GetNode(intPair point, const vector<NodeData>& nodeData);
NodeData UpdateDistanceAndParent(intPair neighbour, intPair current, const vector<NodeData>& nodeData);
int ConvertToIndex(intPair coordinates, pair<int, int> mapDimansions);
bool CheckBounds(intPair point, pair<int, int> mapDimansions);
bool CheckIfPointIsUnexplored(intPair point, const vector<intPair>& exploredPoints);
void PopulateNeighbours(intPair currentPoint, set<intPair>& Neighbours, intPair mapDimensions, const vector<bool>* Map, vector<intPair> expleoredPoints);
bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<bool>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath);

/// <summary>
/// distance between two points on the grid using manhattan distance
/// Manhattan distance was recommended for a grid based problem and so i chose it
/// </summary>
/// <param name="start"></param>
/// <param name="end"></param>
/// <returns></returns>
int FindDistanceBetween(intPair start, intPair end)
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
NodeData GetNode(intPair point, const vector<NodeData>& nodeData) 
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
NodeData UpdateDistanceAndParent(intPair neighbour, intPair current, const vector<NodeData>& nodeData)
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
int ConvertToIndex(intPair coordinates, pair<int,int> mapDimansions)
{
	return (mapDimansions.first * coordinates.second) + coordinates.first;
}

/// <summary>
/// check if a point is in bounds with our map dimensions
/// </summary>
/// <param name="point"></param>
/// <returns></returns>
bool CheckBounds(intPair point, pair<int, int> mapDimansions)
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
bool CheckIfPointIsUnexplored(intPair point, const vector<intPair>& exploredPoints)
{
	for (intPair coordinate : exploredPoints)
	{
		if (point == coordinate)
			return false;
	}
	return true;
}

/// <summary>
///	find neighbours for a point on grid
/// </summary>
void PopulateNeighbours(intPair currentPoint, set<intPair>& Neighbours, intPair mapDimensions, const vector<bool> Map, vector<intPair> expleoredPoints)
{
	Neighbours.clear();

	intPair temp;
	bool isTraversible;

	temp = make_pair(currentPoint.first + 1, currentPoint.second);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToIndex(temp, mapDimensions));
		if (isTraversible) Neighbours.insert(temp);
	}

	temp = make_pair(currentPoint.first - 1, currentPoint.second);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToIndex(temp, mapDimensions));
		if (isTraversible) Neighbours.insert(temp);
	}

	temp = make_pair(currentPoint.first, currentPoint.second + 1);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToIndex(temp, mapDimensions));
		if (isTraversible) Neighbours.insert(temp);
	}

	temp = make_pair(currentPoint.first, currentPoint.second - 1);
	if (CheckBounds(temp, mapDimensions) && CheckIfPointIsUnexplored(temp, expleoredPoints))
	{
		isTraversible = Map.at(ConvertToIndex(temp, mapDimensions));
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
	priority_queue<NodeData, vector<NodeData>, comp> PQ;
	NodeData point;
	set<intPair> Neighbours;
	vector<intPair> exploredPoints;
	vector<NodeData> vec_NodeData;

	point.distanceFromStart = FindDistanceBetween(make_pair(0, 0), MapDimensions) * 10;

	//Fill data in priority queue
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

	for (const NodeData& node : vec_NodeData)
	{
		if (node.distanceFromStart == 0)
		{
			PQ.push(node);
		}
	}

	while (!PQ.empty())
	{
		pair<int, int> currentNode = PQ.top().coordinate;
		exploredPoints.push_back(currentNode);
		PQ.pop();

		if (currentNode == Target)
		{
			intPair pathTrace = Target;
			while (pathTrace != Start)
			{
				OutPath.push_back(ConvertToIndex(pathTrace, MapDimensions));
				pathTrace = GetNode(pathTrace, vec_NodeData).parentNode;
			}
			reverse(OutPath.begin(), OutPath.end());
			return true;
		}

		PopulateNeighbours(currentNode, Neighbours, MapDimensions, Map, exploredPoints);

		for (pair<int, int> neighbour : Neighbours)
		{
			int newDistance = FindDistanceBetween(neighbour, currentNode);
			NodeData tempNode = UpdateDistanceAndParent(neighbour, currentNode, vec_NodeData);
			for(int i=0; i<vec_NodeData.size(); i++)
			{
				if (vec_NodeData[i].coordinate == tempNode.coordinate)
				{
					vec_NodeData[i].distanceFromStart = tempNode.distanceFromStart;
					vec_NodeData[i].parentNode = tempNode.parentNode;
				}
			}
			for (const NodeData& node : vec_NodeData)
			{
				if (node.coordinate == neighbour)
				{
					PQ.push(node);
				}
			}
		}
	}
	return false;
}

//driver code
int main()
{
	//Globals Variables
	pair<int, int> MapDimensions(5,5);
	pair<int, int> Target(2,0);
	pair<int, int> Start(0,0);
	vector<int> Path;
	vector<bool> Map = {1,0,1,1,1 ,1,0,0,0,1 ,1,0,0,0,1 ,1,0,0,0,1, 1,1,1,1,1};

	FindPath(Start, Target, Map, MapDimensions, Path);

	return 0;
}