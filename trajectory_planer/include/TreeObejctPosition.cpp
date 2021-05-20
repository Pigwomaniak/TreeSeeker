#include <TreeObejctPosition.h>

TreeObejctPosition::TreeObejctPosition(unsigned short id, Point sum, double radius, double num) : id(id),sum(sum),  radius(radius), num(num),visited(0)
{
    this->sum.setPos(sum.getPos1()*num,sum.getPos2()*num);
}

bool TreeObejctPosition::addIfInclude (const Point& p, unsigned short idp, double weigth)
{
    if(ifInclude(p) && id == idp)
    {
        sum.setPos(sum.getPos1()+(p.getPos1()*weigth),sum.getPos2()+(p.getPos2()*weigth));
        num+=weigth;
        return true;
    }
    return false;
}

bool TreeObejctPosition::ifInclude (const Point& p)
{
    Point mainP = getPoint();

    return (pow(p.getPos1()-mainP.getPos1(),2) + pow(p.getPos2()-mainP.getPos2(),2)) <= pow(radius,2);
}

Point TreeObejctPosition::getPoint()const
{
    return Point(sum.getPos1()/num, sum.getPos2()/num);
}