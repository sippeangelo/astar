#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <vector>
#include <queue>
#include <set>
#include "DV1419Map.h"
#include <boost/heap/fibonacci_heap.hpp>

class AStar
{
public:
	struct Node;

	// Comparison function for priority queue
	struct LowestFCost
	{
		bool operator()(const Node* l, const Node* r) const
		{
			// Tie breaking
			if (l->F == r->F)
				return l->H > r->H;
			else
				return l->F > r->F;
		}
	};

	struct Node
	{
		Node() : Parent(nullptr) { }
		Node(int X, int Y): X(X), Y(Y), G(0), H(0), F(0), Parent(nullptr) { }
		
		int F;
		int X, Y;

		int G;
		int H;

		Node* Parent;

		boost::heap::fibonacci_heap<Node*, boost::heap::compare<LowestFCost>>::handle_type QueueHandle;
	};

	class Heuristics
	{
	public:
		typedef double (*HeuristicMethod)(Node* start, Node* end);

		static double Manhattan(Node* start, Node* end)
		{
			return abs(end->X - start->X) + abs(end->Y - start->Y);
		}

		static double Euclidean(Node* start, Node* end)
		{
			return sqrt(pow((double)(start->X - end->X), 2) + pow((double)(start->Y - end->Y), 2));
		}

		static double Octile(Node* start, Node* end)
		{
			int xDist = abs(end->X - start->X);
			int yDist = abs(end->Y - start->Y);
			return max(xDist, yDist) + (sqrt((double)2) - 1)*min(xDist, yDist);
		}

		static double Diagonal(Node* start, Node* end)
		{
			int xDist = abs(start->X - end->X);
			int yDist = abs(start->Y - end->Y);

			if (xDist > yDist)
				return 1.4*yDist + (xDist - yDist);
			else
				return 1.4*xDist + (yDist - xDist);
		}

		static double None(Node* start, Node* end)
		{
			return 0;
		}
	};

	AStar(DV1419Map* map) : m_RawMap(map), m_HeuristicMethod(Heuristics::Diagonal) { Initialize(); }
	AStar(DV1419Map* map, Heuristics::HeuristicMethod heuriscitMethod) : m_RawMap(map), m_HeuristicMethod(heuriscitMethod) { Initialize(); }
	void Initialize();
	~AStar();

	void Prepare(Coordinate start, Coordinate goal);
	Node* Update();
	std::vector<Coordinate>* ReconstructPath(Node* finalNode);
	std::vector<Coordinate>* Path(Coordinate start, Coordinate goal);

	Node** m_Nodes;
	boost::heap::fibonacci_heap<Node*, boost::heap::compare<LowestFCost>> m_bpqOpenList;
	Node** m_aOpenList;
	Node** m_aClosedList;
	Node* m_CurrentNode;

private:
	bool IsWalkable(int x, int y);
	Node* GetNode(int x, int y);

	DV1419Map* m_RawMap;
	bool* m_Map;
	int m_MapWidth;
	int m_MapHeight;

	Heuristics::HeuristicMethod m_HeuristicMethod;

	Node* m_StartNode;
	Node* m_GoalNode;
};

void AStar::Initialize()
{
	m_MapWidth = m_RawMap->getWidth();
	m_MapHeight = m_RawMap->getHeight();

	m_Map = new bool[m_MapWidth * m_MapHeight];
	std::string map = m_RawMap->toString();

	for (int i = 0; i < m_MapWidth * m_MapHeight; i++)
	{
		switch (map[i])
		{
		case '.':
		case 'S':
		case 'G':
			m_Map[i] = true;
			break;
		default:
			m_Map[i] = false;
		}
	}

	m_Nodes = new Node*[m_MapWidth * m_MapHeight];
	m_aOpenList = new Node*[m_MapWidth * m_MapHeight];
	m_aClosedList = new Node*[m_MapWidth * m_MapHeight];
}

AStar::~AStar()
{
	delete[] m_Map;
	delete[] m_Nodes;
	delete[] m_aOpenList;
	delete[] m_aClosedList;
}

void AStar::Prepare(Coordinate start, Coordinate goal)
{
	for (int i = 0; i < m_MapWidth * m_MapHeight; i++)
	{
		m_Nodes[i] = nullptr;
		m_aOpenList[i] = nullptr;
		m_aClosedList[i] = nullptr;
	}
	m_bpqOpenList.clear();
	m_CurrentNode = nullptr;

	m_StartNode = GetNode(start.X, start.Y);
	m_GoalNode = GetNode(goal.X, goal.Y);
	m_StartNode->H = (*m_HeuristicMethod)(m_StartNode, m_GoalNode);
	m_StartNode->F = m_StartNode->H;

	auto handle = m_bpqOpenList.push(m_StartNode);
	(*handle)->QueueHandle = handle;
	m_aOpenList[m_StartNode->Y * m_MapWidth + m_StartNode->X] = m_StartNode;
}

AStar::Node* AStar::Update()
{
	// TODO: What happens when a path is not found?
	if (m_bpqOpenList.empty())
		return m_CurrentNode;

	// Look for the lowest F cost node in the open list
	m_CurrentNode = m_bpqOpenList.top();
	m_bpqOpenList.pop();

	m_aOpenList[m_CurrentNode->Y * m_MapWidth + m_CurrentNode->X] = nullptr;
	m_aClosedList[m_CurrentNode->Y * m_MapWidth + m_CurrentNode->X] = m_CurrentNode;

	// Check if we reached the goal yet
	if (m_CurrentNode->X == m_GoalNode->X && m_CurrentNode->Y == m_GoalNode->Y)
		return m_CurrentNode;

	// Add neighboring nodes to the open list
	for (int y = -1; y <= 1; y++)
	{
		for (int x = -1; x <= 1; x++)
		{
			// Ignore the middle node
			if (x == 0 && y == 0)
				continue;

			int neighbourX = m_CurrentNode->X + x;
			int neighbourY = m_CurrentNode->Y + y;

			// Is the coordinate walkable?
			if (!IsWalkable(neighbourX, neighbourY))
				continue;

			// Don't cut corners
			bool isDiagonal = abs(x) == abs(y);
			if (isDiagonal)
			{
				bool walkUp, walkRight, walkDown, walkLeft;
				walkUp = IsWalkable(m_CurrentNode->X, m_CurrentNode->Y - 1);
				walkRight = IsWalkable(m_CurrentNode->X + 1, m_CurrentNode->Y);
				walkDown = IsWalkable(m_CurrentNode->X, m_CurrentNode->Y + 1);
				walkLeft = IsWalkable(m_CurrentNode->X - 1, m_CurrentNode->Y);

				if (x == 1 && y == -1) // North-east
					if (!walkRight || !walkUp)
						continue;
				if (x == 1 && y == 1) // South-east
					if (!walkRight || !walkDown)
						continue;
				if (x == -1 && y == 1) // South-west
					if (!walkLeft || !walkDown)
						continue;
				if (x == -1 && y == -1) // North-west
					if (!walkLeft || !walkUp)
						continue;
			}				

			// Is the node already present in the closed list?
			if (m_aClosedList[neighbourY * m_MapWidth + neighbourX] != nullptr)
				continue;

			// Is the node already present in the open list?
			Node* inOpenList = m_aOpenList[neighbourY * m_MapWidth + neighbourX];

			if (inOpenList == nullptr)
			{
				// Put it in the open list
				Node* node = GetNode(neighbourX, neighbourY);
				int g = m_CurrentNode->G + ((isDiagonal) ? 14 : 10);
				int h = (*m_HeuristicMethod)(node, m_GoalNode) * 10;
				node->G = g;
				node->H = h;
				node->F = g + h;
				node->Parent = m_CurrentNode;
				m_aOpenList[node->Y * m_MapWidth + node->X] = node;
				auto handle = m_bpqOpenList.push(node);
				(*handle)->QueueHandle = handle;
			}
			else
			{
				// Check to see if this path to that node is better
				if (m_CurrentNode->G + ((isDiagonal) ? 14 : 10) < inOpenList->G)
				{
					int g = m_CurrentNode->G + ((isDiagonal) ? 14 : 10);
					int h = (*m_HeuristicMethod)(inOpenList, m_GoalNode) * 10;
					inOpenList->G = g;
					inOpenList->H = h;
					inOpenList->F = g + h;
					inOpenList->Parent = m_CurrentNode;
					// Update the priority queue, since the cost was decreased
					m_bpqOpenList.decrease(inOpenList->QueueHandle);
				}
			}				
		}
	}

	return nullptr;
}

std::vector<Coordinate>* AStar::ReconstructPath(Node* finalNode)
{
	std::vector<Coordinate>* pathCoordinates = new std::vector<Coordinate>;
	while (finalNode != nullptr)
	{
		pathCoordinates->push_back(Coordinate(finalNode->X, finalNode->Y));
		finalNode = finalNode->Parent;
	}
	std::reverse(pathCoordinates->begin(), pathCoordinates->end());

	// Cleanup
	for (int i = 0; i < m_MapWidth * m_MapHeight; i++)
	{
		if (m_Nodes[i] != nullptr)
			delete m_Nodes[i];
	}

	return pathCoordinates;
}

std::vector<Coordinate>* AStar::Path(Coordinate start, Coordinate goal)
{
	Prepare(start, goal);

	Node* goalNode = nullptr;
	while (goalNode == nullptr)
		goalNode = Update();

	return ReconstructPath(goalNode);
}

AStar::Node* AStar::GetNode(int x, int y)
{
	int index = y * m_MapWidth + x;
	Node* node = m_Nodes[index];

	if (node == nullptr)
	{
		node = new Node(x, y);
		m_Nodes[index] = node;
	}

	return node;
}

bool AStar::IsWalkable(int x, int y)
{
	if (x < 0 || x >= m_MapWidth || y < 0 || y >= m_MapHeight)
		return false;

	return m_Map[y * m_MapWidth + x];
}

#endif