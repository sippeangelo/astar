#ifndef DV1419MAP_H
#define DV1419MAP_H

#include <string>
#include <sstream>
#include <vector>

using namespace std;

struct Coordinate
{
	int X, Y;
	Coordinate(){}
	Coordinate(int x, int y): X(x), Y(y){}
	string toString() const { stringstream s; s << X << ", " << Y; return s.str();}
};

class DV1419Map
{
public:
	DV1419Map(const char* filename);
	~DV1419Map();
	bool isWalkable(const Coordinate& c) const;
	bool isWalkable(int x, int y) const;
	double getPathLength(const vector<Coordinate>& moves) const;
	string toString() const {return m_map;}
	void print() const;
	int getWidth() const {return m_width;}
	int getHeight() const {return m_height;}
	char operator()(int x, int y) const;
private:
	int m_height;
	int m_width;
	string m_map;
	double m_sqrt2;
};

#endif