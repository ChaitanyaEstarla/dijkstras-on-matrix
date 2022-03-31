#include<iostream>
#include<utility>
#include<vector>
#include<queue>
#include<map>

using namespace std;

//typeDefs
typedef pair<int, int> intPair;

//Globals Variables
intPair MapDimensions(4, 4);
intPair Target(3, 3);
intPair Start(1, 2);
vector<int> OutPath;
vector<bool> Map = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 , 1, 0, 0, 1 };

//Identifiers
bool FindPath(intPair, intPair, const vector<bool>*, intPair, vector<int>*);
int FindDistanceBetween(intPair, intPair);

//structure
struct Nodes
{
	int distanceFromStart;
	intPair coordinate;
	intPair parentNode;
};

int main()
{
	FindPath(Start, Target, &Map, MapDimensions, &OutPath);

	return 0;
}

/// <summary>
/// 
/// </summary>
/// <param name="start"></param>
/// <param name="target"></param>
/// <param name="grid"></param>
/// <param name="gridDimensions"></param>
/// <param name="path"></param>
/// <returns></returns>
bool FindPath(intPair start, intPair target, const vector<bool>* grid, intPair gridDimensions, vector<int>* path)
{
	priority_queue<Nodes, vector<Nodes>, greater<Nodes>> nodeData;
	pair<intPair, intPair> parentNodeData;

	Nodes nodes;
	nodes.distanceFromStart = FindDistanceBetween(make_pair(0,0), MapDimensions);
	nodes.parentNode = make_pair(0, 0);

	for (int i=0; i < gridDimensions.first; i++)
	{
		for (int j = 0; j < gridDimensions.second; j++)
		{
			nodes.coordinate = make_pair(i, j);
			nodeData.push(nodes);
		}
	}



	return false;
}

/// <summary>
/// 
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
