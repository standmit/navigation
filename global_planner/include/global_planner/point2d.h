#ifndef POINT2D_H
#define POINT2D_H

/**
 * @brief The Point class provides simple point interface
 */
class Point
{
public:
    Point(uint xv, uint yv): x(xv), y(yv){}
    Point(): Point(0, 0){}

    uint x;
    uint y;
};

inline bool operator<( const Point &p1, const Point &p2 )
{
    return    (p1.x < p2.x)
           || ((p1.x == p2.x) && (p1.y <  p2.y));
}

inline Point operator-( const Point &p1, const Point &p2 )
{
    return Point( p1.x - p2.x, p1.y - p2.y );
}

#endif
