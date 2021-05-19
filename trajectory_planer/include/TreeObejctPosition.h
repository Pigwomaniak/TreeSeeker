#pragma once
#include <math.h>

class Point{
public:
	Point(double p1, double p2) : pos1(p1),pos2(p2){};
	double getPos1()const{return pos1;};
	double getPos2()const{return pos2;};
	void setPos(double p1, double p2) { pos1 = p1;pos2 = p2;};
	double countDistance(const Point& p)const {return pow(p.pos1-pos1,2) + pow(p.pos2-pos2,2); };

private:
	double pos1, pos2;
};


class TreeObejctPosition{
public:
	TreeObejctPosition(unsigned short id, Point sum, double radius);

	bool addIfInclude (const Point& p,unsigned short idp);
	bool ifInclude (const Point& p);
	Point getPoint()const;
	unsigned short getId()const{return id;};
	double getRadius()const {return radius;};
	bool isVisited()const {return visited;};
	void setVisited(){visited = true;};

private:
	unsigned short id;
	Point sum;
	unsigned int num;
	const double radius;
	bool visited;
};


