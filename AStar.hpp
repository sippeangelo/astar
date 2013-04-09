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

class AStar
{
public:
	struct Node
	{
		Node() : Parent(nullptr) { }
		Node(int X, int Y): X(X), Y(Y), G(0), H(0), F(0), Parent(nullptr) { }

		int X, Y;

		int G;
		int H;
		int F;

		Node* Parent;
	};

	struct LowestFCost
	{
		bool operator()(const Node* l, const Node* r) const
		{
			return l->F > r->F;
		}
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

		static double Diagonal(Node* start, Node* end)
		{
			int xDist = abs(end->X - start->X);
			int yDist = abs(end->Y - start->Y);

			if (xDist > yDist)
				return 1.4 * yDist + (xDist - yDist);
			else
				return 1.4 * xDist + (yDist - xDist);
		}

		static double None(Node* start, Node* end)
		{
			return 0;
		}
	};

	AStar(DV1419Map* map) : m_RawMap(map), m_HeuristicMethod(Heuristics::Diagonal) { Initialize(); }
	AStar(DV1419Map* map, Heuristics::HeuristicMethod heuriscitMethod) : m_RawMap(map), m_HeuristicMethod(heuriscitMethod) { Initialize(); }
	void Initialize();

	void PrintTest();

	void Prepare(Coordinate start, Coordinate goal);
	std::vector<Coordinate>* Update();
	std::vector<Coordinate> Path(Coordinate start, Coordinate goal);

	std::priority_queue<Node*, std::vector<Node*>, LowestFCost> m_qOpenList;
	Node** m_aOpenList;
	Node** m_aClosedList;
	Node* m_CurrentNode;

private:
	bool IsWalkable(int x, int y);
	std::vector<Node*> IdentifySuccessors(Node* current, Node* start, Node* end);
	Node* Jump(int currentX, int currentY, int dX, int Dy, Node* start, Node* end);
	std::vector<Coordinate>* ReconstructPath(Node* finalNode);

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

	m_aOpenList = new Node*[m_MapWidth * m_MapHeight];
	m_aClosedList = new Node*[m_MapWidth * m_MapHeight];
}

void AStar::PrintTest()
{
	std::cout << "PrintTest: " << (*m_HeuristicMethod)(new Node(0, 0), new Node(1, 1)) << std::endl; 
}

void AStar::Prepare(Coordinate start, Coordinate goal)
{
	m_StartNode = new Node(start.X, start.Y);
	m_GoalNode = new Node(goal.X, goal.Y);
	m_StartNode->H = (*m_HeuristicMethod)(m_StartNode, m_GoalNode);
	m_StartNode->F = m_StartNode->H;
	m_CurrentNode = nullptr;

	for (int i = 0; i < m_MapWidth * m_MapHeight; i++)
	{
		m_aOpenList[i] = nullptr;
		m_aClosedList[i] = nullptr;
	}
	while (!m_qOpenList.empty())
		m_qOpenList.pop();

	m_qOpenList.push(m_StartNode);
	m_aOpenList[m_StartNode->Y * m_MapWidth + m_StartNode->X] = m_StartNode;
}

std::vector<Coordinate>* AStar::Update()
{
	if (m_qOpenList.empty())
		return ReconstructPath(m_CurrentNode);

	// Look for the lowest F cost node in the open list
	m_CurrentNode = m_qOpenList.top();
	m_qOpenList.pop();
	m_aOpenList[m_CurrentNode->Y * m_MapWidth + m_CurrentNode->X] = nullptr;
	m_aClosedList[m_CurrentNode->Y * m_MapWidth + m_CurrentNode->X] = m_CurrentNode;

	// Check if we reached the goal yet
	if (m_CurrentNode->X == m_GoalNode->X && m_CurrentNode->Y == m_GoalNode->Y)
		return ReconstructPath(m_CurrentNode);

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
				/*walkUp = m_RawMap->isWalkable(currentNode->X, currentNode->Y - 1);
				walkRight = m_RawMap->isWalkable(currentNode->X + 1, currentNode->Y);
				walkDown = m_RawMap->isWalkable(currentNode->X, currentNode->Y + 1);
				walkLeft = m_RawMap->isWalkable(currentNode->X - 1, currentNode->Y);*/
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
				Node* node = new Node(neighbourX, neighbourY);
				int g = m_CurrentNode->G + ((isDiagonal) ? 14 : 10);
				int h = (*m_HeuristicMethod)(node, m_GoalNode) * 10;
				node->G = g;
				node->H = h;
				node->F = g + h;
				node->Parent = m_CurrentNode;
				m_aOpenList[node->Y * m_MapWidth + node->X] = node;
				m_qOpenList.push(node);
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
					// HACK: Rebuild priority queue or it might crash with an "invalid heap" error
					//std::make_heap(const_cast<Node**>(&m_qOpenList.top()), const_cast<Node**>(&m_qOpenList.top()) + m_qOpenList.size(), LowestFCost());
				}
			}				
		}
	}

	return nullptr;
}

/*std::vector<AStar::Node*> AStar::IdentifySuccessors(Node* current, Node* start, Node* end)
{
	std::vector<Node*> successors;
	
	for (int y = -1; y <= 1; y++)
	{
		for (int x = -1; x <= 1; x++)
		{
			// Ignore the middle node
			if (x == 0 && y == 0)
				continue;

			int neighbourX = current->X + x;
			int neighbourY = current->Y + y;

			// Direction from current node to neighbour
			int dX = neighbourX - current->X;
			dX = (dX > 1) ? 1 : ((dX < -1) ? -1 : dX);
			int dY = neighbourY - current->Y;
			dY = (dY > 1) ? 1 : ((dY < -1) ? -1 : dY);

			// Try to find a node to jump to
			Node* jumpPoint = Jump(current->X, current->Y, dX, dY, start, end);
			if (jumpPoint != nullptr)
				successors.push_back(jumpPoint);
		}
	}
}*/

/*AStar::Node* AStar::Jump(int currentX, int currentY, int dX, int dY, Node* start, Node* end)
{
	int nextX = currentX + dX;
	int nextY = currentY + dY;

	// Is the coordinate walkable?
	if (!IsWalkable(nextX, nextY))
		return nullptr;

	// Check if we reached the goal yet
	if (nextX == end->X && nextY == end->Y)
		return new Node(nextX, nextY);

	// Diagonal case
	if (dX != 0 && dY != 0)
	{

	}
}*/

std::vector<Coordinate>* AStar::ReconstructPath( Node* finalNode )
{
	std::vector<Coordinate>* pathCoordinates = new std::vector<Coordinate>;
	while (finalNode != nullptr)
	{
		pathCoordinates->push_back(Coordinate(finalNode->X, finalNode->Y));
		finalNode = finalNode->Parent;
	}
	std::reverse(pathCoordinates->begin(), pathCoordinates->end());

	return pathCoordinates;
}

std::vector<Coordinate> AStar::Path(Coordinate start, Coordinate goal)
{
	Prepare(start, goal);

	std::vector<Coordinate>* path = nullptr;

	while (path == nullptr)
		path = Update();

	return *path;
}

bool AStar::IsWalkable(int x, int y)
{
	//return m_RawMap->isWalkable(x, y);

	if (x < 0 || x >= m_MapWidth || y < 0 || y >= m_MapHeight)
		return false;

	return m_Map[y * m_MapWidth + x];
}

#endif