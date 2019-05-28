#include "global_planner/orientation_on_obstacle.h"

#include <ros/ros.h>
#include <tf/tf.h>

#include "global_planner/obstacle_vector.h"

#include "exception"

#define UNUSED(x)(void(x))

#define DEG_TO_RAD(x)((x)*(M_PI/180))
namespace global_planner
{

void OrientationOnObstacle::processPath(const geometry_msgs::PoseStamped &start, std::vector< geometry_msgs::PoseStamped > &path )
{
    UNUSED( start );
    if( !_costmap )
    {
        ROS_ERROR_NAMED( "obstacle_orientation", "Costmap wasn'\t inited, aborted" );
        return;
    }

    //! Get obstacles list
    ObstacleVector vObs = ObstacleVector::getObstacles(*_costmap);

    //! Process each path point
    for( auto it = path.begin(); it != path.end()-1; ++it )
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
                                  int( obstcPoint.x - checkPoint.x ) ) + _additionalRotateValue;

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

void OrientationOnObstacle::setAdditionalRotateDegree(double value)
{
    _additionalRotateValue = DEG_TO_RAD( value );
}

}
