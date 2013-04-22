#include <iostream>
#include <sstream>
#include <fstream>
#include <cassert>
#include <cmath>

#include "DV1419Map.h"

DV1419Map::DV1419Map(const char* filename)
{
	ifstream mapfile;
	mapfile.open(filename, ios_base::in);
	while (!mapfile.eof())
	{
		stringstream linestream;
		string line;
		getline(mapfile, line);
		linestream.str(line);
		string id;
		linestream >> id;
		if (id == "type")
		{
			continue;
		}
		else if (id == "height")
		{
			linestream >> m_height;
		}
		else if (id == "width")
		{
			linestream >> m_width;
		}
		else if (id == "map")
		{
			continue;
		}
		else
			m_map += id;

	}

	m_sqrt2 = sqrt(2.0);
}


DV1419Map::~DV1419Map()
{
}

bool DV1419Map::isWalkable(const Coordinate& c) const
{
	return isWalkable(c.X, c.Y);
}

bool DV1419Map::isWalkable(int x, int y) const
{
	if (x < 0 || x >= m_width || y < 0 || y >= m_height)
		return false;

	bool result = false;
	char element;

	element = this->operator()(x, y);

	switch (element)
	{
	case '.':
	case 'S':
	case 'G':
		result = true;
	}

	return result;
}

double DV1419Map::getPathLength(const vector<Coordinate>& moves) const
{
	assert(moves.size() > 1);
	double length = 0.0;
	if (!isWalkable(moves.front()))
	{
		length = -1.0;
		cerr << "Invalid move (terrain unpassable):\nTerrain unpassable at index 0:\n" << endl;
		return length;
	}
	for (int i = 1; i < moves.size(); ++i)
	{
		if (!isWalkable(moves[i]))
		{
			length = -1.0;
			cerr << "Invalid move (terrain unpassable):\nTerrain unpassable at index " << i << endl;
			break;
		}

		int dx, dy;
		Coordinate beg, end;
		beg = moves[i - 1];
		end = moves[i];
		dx = end.X - beg.X;
		dy = end.Y - beg.Y;

		bool walkUp, walkRight, walkDown, walkLeft;
		walkUp = isWalkable(beg.X, beg.Y - 1);
		walkRight = isWalkable(beg.X + 1, beg.Y);
		walkDown = isWalkable(beg.X, beg.Y + 1);
		walkLeft = isWalkable(beg.X - 1, beg.Y);

		if (abs(dx) > 1 || abs(dy) > 1)
		{
			length = -1.0;
			cerr << "Invalid move (too long):\nMove number " << i << endl;
			break;
		}

		if (dx == 0 && dy == 0)
		{
			length = -1.0;
			cerr << "Invalid move (didn't move):\nMove number " << i << endl;
			break;
		}
		else if (dx == 0 && dy == -1) // STRAIGHT UP
		{
			length += 1.0;
		}
		else if (dx == 1 && dy == -1) // RIGHT UP
		{
			if (!walkRight)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (to the right)):\nMove number " << i << endl;
				break;
			}
			else if (!walkUp)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (upwards)):\nMove number " << i << endl;
				break;
			}
			length += m_sqrt2;
		}
		else if (dx == 1 && dy == 0) // STRAIGHT RIGHT
		{
			length += 1.0;
		}
		else if (dx == 1 && dy == 1) // RIGHT DOWN
		{
			if (!walkRight)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (to the right)):\nMove number " << i << endl;
				break;
			}
			else if (!walkDown)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (downwards)):\nMove number " << i << endl;
				break;
			}
			length += m_sqrt2;
		}
		else if (dx == 0 && dy == 1) // STRAIGHT DOWN
		{
			length += 1.0;
		}
		else if (dx == -1 && dy == 1) // LEFT DOWN
		{
			if (!walkLeft)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (to the left)):\nMove number " << i << endl;
				break;
			}
			else if (!walkDown)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (downwards)):\nMove number " << i << endl;
				break;
			}
			length += m_sqrt2;
		}
		else if (dx == -1 && dy == 0) // STRAIGHT LEFT
		{
			length += 1.0;
		}
		else if (dx == -1 && dy == -1) // LEFT UP
		{
			if (!walkLeft)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (to the left)):\nMove number " << i << endl;
				break;
			}
			else if (!walkUp)
			{
				length = -1.0;
				cerr << "Invalid move (can't cut corners (upwards)):\nMove number " << i << endl;
				break;
			}
			length += m_sqrt2;
		}

		
	}

	return length;
}

void DV1419Map::print() const
{
	for (int i = 0; i < m_height; ++i)
		cout << m_map.substr(i * m_width, m_width) << endl;
}



char DV1419Map::operator()(int x, int y) const
{
	assert(x >= 0 && x < m_width);
	assert(y >= 0 && y < m_height);

	return m_map[y * m_width + x];
}
