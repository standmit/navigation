#include "global_planner/orientation_on_obstacle.h"

#include <ros/ros.h>
#include <tf/tf.h>

#include "costmap_2d/cost_values.h"

#include "global_planner/priority_vector.h"
#include "exception"

#define UNUSED(x)(void(x))

#define DEG_TO_RAD(x)((x)*(M_PI/180))
namespace global_planner
{

double calcPath( const Point &p1, const Point &p2 )
{
    Point t = p1 - p2;
    return hypot( int( t.x ), int( t.y ) );
}

void OrientationOnObstacle::processPath(const geometry_msgs::PoseStamped &start, std::vector< geometry_msgs::PoseStamped > &path )
{
    UNUSED( start );
    if( !_costmap )
    {
        ROS_ERROR_NAMED( "obstacle_orientation", "Costmap wasn'\t inited, aborted" );
        return;
    }

    //! Get obstacles list
    ObstacleVector vObs = getObstacles();

    double additionalRotation = DEG_TO_RAD(30);
    //! Process each path point
    for( auto it = path.begin(); it != path.end(); ++it )
    {
        Point checkPoint;

        //! Convert world coords to map coords
        geometry_msgs::Pose &point = (*it).pose;
        _costmap->worldToMap( point.position.x, point.position.y,
                              checkPoint.x,     checkPoint.y );

        try
        {
            Point obstcPoint = vObs.getClosestObstacleToPoint( checkPoint );

            double angle = atan2( int( obstcPoint.y - checkPoint.y ),
                                  int( obstcPoint.x - checkPoint.x ) ) + additionalRotation;

            point.orientation = tf::createQuaternionMsgFromYaw( angle );
            ROS_DEBUG_NAMED( "obstacle_orientation", "Path point yaw orientation: %f\n"
                                                     "\tobst point: %d %d\n"
                                                     "\ttgt point:  %d %d",
                                angle,
                                obstcPoint.x, obstcPoint.y,
                                checkPoint.x, checkPoint.y );
        }
        catch( const std::runtime_error &e )
        {
            UNUSED( e );
            ROS_ERROR_NAMED( "obstacle_orientation", "Trying to get closest obstacle from empty obstacle vector, aborted" );
            return;
        }

    }
}

void OrientationOnObstacle::setCostmap(costmap_2d::Costmap2D *costmap)
{
    _costmap = costmap;
}

ObstacleVector OrientationOnObstacle::getObstacles() const
{
    ObstacleVector result;

    uint sizeY = _costmap->getSizeInCellsY();
    uint sizeX = _costmap->getSizeInCellsX();

    for( uint i = 0; i < sizeY; ++i )
    {
        for( uint j = 0; j < sizeX; ++j )
        {
            unsigned char cell = _costmap->getCost( j, i );
            if( cell == costmap_2d::LETHAL_OBSTACLE )
            {
                result.push_to_heap( Point( j, i ) );
            }
        }
    }

    return result;
}

Point ObstacleVector::getClosestObstacleToPoint(const Point &p)
{
    if( empty() )
    {
        throw std::runtime_error( "Obstacle vector is empty" );
    }

    Point  minPoint;
    double minPath = std::numeric_limits< double >::max();

    for( auto it = begin(); it != end(); ++it )
    {
        double path = calcPath( p, (*it) );
        if( path < minPath )
        {
            minPath  = path;
            minPoint = (*it);
        }
    }

    return minPoint;
}



}
