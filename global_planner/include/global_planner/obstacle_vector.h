#ifndef OBSTACLE_VECTOR_H
#define OBSTACLE_VECTOR_H

#include <costmap_2d/costmap_2d.h>

#include "global_planner/point2d.h"
#include "global_planner/priority_vector.h"

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

    static ObstacleVector getObstacles( const costmap_2d::Costmap2D &costmap );
};

#endif
