#include "global_planner/obstacle_vector.h"

#include "costmap_2d/cost_values.h"

#include "exception"

double calcPath( const Point &p1, const Point &p2 )
{
    Point t = p1 - p2;
    return hypot( int( t.x ), int( t.y ) );
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

ObstacleVector ObstacleVector::getObstacles(const costmap_2d::Costmap2D &costmap)
{
    ObstacleVector result;

    uint sizeY = costmap.getSizeInCellsY();
    uint sizeX = costmap.getSizeInCellsX();

    for( uint i = 0; i < sizeY; ++i )
    {
        for( uint j = 0; j < sizeX; ++j )
        {
            unsigned char cell = costmap.getCost( j, i );
            if( cell == costmap_2d::LETHAL_OBSTACLE )
            {
                result.push_to_heap( Point( j, i ) );
            }
        }
    }

    return result;
}
