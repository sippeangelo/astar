#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <cmath>
#include <set>
#include "DV1419Map.h"

class AStar
{
public:
	struct Node;

	/// <summary>
	/// Comparison function for priority queue
	/// </summary>
	struct LowestFCost
	{
		bool operator()(const Node* l, const Node* r) const
		{
			return l->F < r->F;
		}
	};

	/// <summary>
	/// An internal node structure.
	/// </summary>
	struct Node
	{
		Node() : X(0), Y(0), G(0), H(0), F(0), Parent(nullptr) { }
		Node(int X, int Y): X(X), Y(Y), G(0), H(0), F(0), Parent(nullptr) { }
		
		int X, Y;
		int G, H, F;

		Node* Parent;

		std::multiset<Node*, LowestFCost>::iterator Iterator;
	};

	/// <summary>
	/// Collection of default heuristic functions.
	/// </summary>
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

	std::vector<Coordinate>* Path(Coordinate start, Coordinate goal);
	void Prepare(Coordinate start, Coordinate goal);
	Node* Update();
	std::vector<Coordinate>* ReconstructPath(Node* finalNode);

	Node** m_Nodes;
	std::multiset<Node*, LowestFCost> m_pqOpenList;
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

/// <summary>
/// Initializes this instance.
/// </summary>
void AStar::Initialize()
{
	// Parse the map
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

	// Create a pool of nodes
	m_Nodes = new Node*[m_MapWidth * m_MapHeight];
	for (int x = 0; x < m_MapWidth; x++)
	{
		for (int y = 0; y < m_MapHeight; y++)
			m_Nodes[y * m_MapWidth + x] = new Node(x, y);
	}

	m_aOpenList = new Node*[m_MapWidth * m_MapHeight];
	m_aClosedList = new Node*[m_MapWidth * m_MapHeight];
}

/// <summary>
/// Finalizes an instance of the <see cref="AStar"/> class.
/// </summary>
AStar::~AStar()
{
	delete[] m_Map;
	for (int i = 0; i < m_MapWidth * m_MapHeight; i++)
		delete m_Nodes[i];
	delete[] m_Nodes;
	delete[] m_aOpenList;
	delete[] m_aClosedList;
}

/// <summary>
/// Finds a path.
/// </summary>
/// <param name="start">The start coordinate.</param>
/// <param name="goal">The goal coordinate.</param>
/// <returns>A vector of coordinates that represents the path</returns>
std::vector<Coordinate>* AStar::Path(Coordinate start, Coordinate goal)
{
	if (!IsWalkable(start.X, start.Y) || !IsWalkable(goal.X, goal.Y))
		return new std::vector<Coordinate>;

	Prepare(start, goal);

	Node* goalNode = nullptr;
	while (goalNode == nullptr)
		goalNode = Update();

	return ReconstructPath(goalNode);
}

/// <summary>
/// Prepares the pathfinder.
/// </summary>
/// <param name="start">The start coordinate.</param>
/// <param name="goal">The goal coordinate.</param>
void AStar::Prepare(Coordinate start, Coordinate goal)
{
	for (int i = 0; i < m_MapWidth * m_MapHeight; i++)
	{
		// Reset the pooled nodes
		m_Nodes[i]->G = 0;
		m_Nodes[i]->H = 0;
		m_Nodes[i]->F = 0;
		m_Nodes[i]->Parent = nullptr;
		// Reset the open and closed list
		m_aOpenList[i] = nullptr;
		m_aClosedList[i] = nullptr;
	}
	// Reset the open list priority queue
	m_pqOpenList.clear();
	m_CurrentNode = nullptr;

	// Set up the initial nodes
	m_StartNode = GetNode(start.X, start.Y);
	m_GoalNode = GetNode(goal.X, goal.Y);
	m_StartNode->H = (*m_HeuristicMethod)(m_StartNode, m_GoalNode);
	m_StartNode->F = m_StartNode->H;
	// Insert the first node into the open list and store the iterator
	m_StartNode->Iterator = m_pqOpenList.insert(m_StartNode);
	m_aOpenList[m_StartNode->Y * m_MapWidth + m_StartNode->X] = m_StartNode;
}

/// <summary>
/// Updates the pathfinder
/// </summary>
/// <returns></returns>
AStar::Node* AStar::Update()
{
	if (m_pqOpenList.empty())
		return m_CurrentNode;

	// Look for the lowest F cost node in the open list
	auto it = m_pqOpenList.begin();
	m_CurrentNode = *it;
	m_pqOpenList.erase(it);

	m_aOpenList[m_CurrentNode->Y * m_MapWidth + m_CurrentNode->X] = nullptr;
	m_aClosedList[m_CurrentNode->Y * m_MapWidth + m_CurrentNode->X] = m_CurrentNode;

	// Check if we reached the goal yet
	if (m_CurrentNode == m_GoalNode)
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

			// Is the coordinate within bounds?
			if (neighbourX < 0 || neighbourX >= m_MapWidth || neighbourY < 0 || neighbourY >= m_MapHeight)
				continue;

			// Is the node already present in the closed list?
			if (m_aClosedList[neighbourY * m_MapWidth + neighbourX] != nullptr)
				continue;

			// Is the coordinate walkable?
			if (!IsWalkable(neighbourX, neighbourY))
				continue;

			// Don't cut corners
			bool isDiagonal = abs(x) == abs(y);
			if (isDiagonal 
				&& (!IsWalkable(m_CurrentNode->X + x, m_CurrentNode->Y) 
					|| !IsWalkable(m_CurrentNode->X, m_CurrentNode->Y + y)))
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
				node->Iterator = m_pqOpenList.insert(node);
			}
			else
			{
				// Check to see if this path to that node is better
				if (m_CurrentNode->G + ((isDiagonal) ? 14 : 10) < inOpenList->G)
				{
					// Remove the node from the priority queue
					m_pqOpenList.erase(inOpenList->Iterator);

					// Update it
					int g = m_CurrentNode->G + ((isDiagonal) ? 14 : 10);
					int h = (*m_HeuristicMethod)(inOpenList, m_GoalNode) * 10;
					inOpenList->G = g;
					inOpenList->H = h;
					inOpenList->F = g + h;
					inOpenList->Parent = m_CurrentNode;

					// Insert the node again with an updated F-score
					inOpenList->Iterator = m_pqOpenList.insert(inOpenList);
				}
			}		
		}
	}

	return nullptr;
}

/// <summary>
/// Reconstructs the path by following the parents back up.
/// </summary>
/// <param name="finalNode">The final node.</param>
/// <returns>A vector of coordinates that represents the path</returns>
std::vector<Coordinate>* AStar::ReconstructPath(Node* finalNode)
{
	std::vector<Coordinate>* pathCoordinates = new std::vector<Coordinate>;

	// If a path wasn't found
	if (finalNode != m_GoalNode)
		return pathCoordinates;

	// Reconstruct the path
	while (finalNode != nullptr)
	{
		pathCoordinates->push_back(Coordinate(finalNode->X, finalNode->Y));
		finalNode = finalNode->Parent;
	}
	// Reverse the vector so the start is at the beginning
	std::reverse(pathCoordinates->begin(), pathCoordinates->end());

	return pathCoordinates;
}

/// <summary>
/// Gets the node that corresponds to the X and Y coordinates.
/// </summary>
/// <param name="x">The x-coordinate.</param>
/// <param name="y">The y-coordinate.</param>
/// <returns>A node</returns>
AStar::Node* AStar::GetNode(int x, int y)
{
	int index = y * m_MapWidth + x;
	Node* node = m_Nodes[index];

	return node;
}

/// <summary>
/// Determines whether the specified coordinate is walkable on the map.
/// </summary>
/// <param name="x">The x-coordinate.</param>
/// <param name="y">The y-coordinate.</param>
/// <returns></returns>
bool AStar::IsWalkable(int x, int y)
{
	if (x < 0 || x >= m_MapWidth || y < 0 || y >= m_MapHeight)
		return false;

	return m_Map[y * m_MapWidth + x];
}

#endif