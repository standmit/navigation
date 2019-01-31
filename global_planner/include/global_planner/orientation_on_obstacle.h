#ifndef ORIENTATION_ON_OBSTACLE_H
#define ORIENTATION_ON_OBSTACLE_H

#include "global_planner/orientation_filter.h"
#include <costmap_2d/costmap_2d.h>

#include "global_planner/priority_vector.h"

namespace global_planner
{

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
    return (p1.x < p2.x);
}

inline Point operator-( const Point &p1, const Point &p2 )
{
    return Point( p1.x - p2.x, p1.y - p2.y );
}

/**
 * @brief The ObstacleVector class provides interface for storing and using obstacles vector
 */
class ObstacleVector: public PriorityVector< Point >
{
public:
    ObstacleVector(): PriorityVector< Point >(){}

    /**
     * @brief Returns closest obstacle to selected point
     * @param p - target point
     * @exception runtime_error - throws if vector is empty
     * @return Point of closest obstacle
     */
    Point getClosestObstacleToPoint( const Point &p );
};

/**
 * @brief The OrientationOnObstacle class provide interface for orientation filter,
 * which directs path points on closest obstacles.
 */
class OrientationOnObstacle: public OrientationFilter
{

public:
    OrientationOnObstacle():
        OrientationFilter(),
        _costmap( nullptr ),
        _additionalRotateValue( 0 ){}
    virtual ~OrientationOnObstacle(){}

    virtual void processPath( const geometry_msgs::PoseStamped &start,
                              std::vector< geometry_msgs::PoseStamped > &path );
    void setCostmap( costmap_2d::Costmap2D *costmap );

    void setAdditionalRotateDegree( double value );
protected:
    /**
     * @brief returns pose of the closest obstacle
     * @param pose - checked point
     * @return pose of the closest obstacle
     */
    geometry_msgs::Pose getClosestObstacle( const geometry_msgs::PoseStamped &pose );

    /**
     * @brief get filtered and sorted vector of obstacles
     * @return obstacles vector
     */
    ObstacleVector getObstacles() const;

    costmap_2d::Costmap2D *_costmap;
    double _additionalRotateValue;
};

}

#endif
