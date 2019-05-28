#ifndef ORIENTATION_ON_OBSTACLE_H
#define ORIENTATION_ON_OBSTACLE_H

#include "global_planner/orientation_filter.h"
#include <costmap_2d/costmap_2d.h>

#include "global_planner/priority_vector.h"

namespace global_planner
{

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

    costmap_2d::Costmap2D *_costmap;
    double _additionalRotateValue;
};

}

#endif
