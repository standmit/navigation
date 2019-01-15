/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_WALL_INFLATION_LAYER_H_
#define COSTMAP_2D_WALL_INFLATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/WallInflationPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

#include <iostream>

#define ALLMOST_LETHAL_OBSTACLE costmap_2d::LETHAL_OBSTACLE - 30

#define SQR(x)((x)*(x))

namespace costmap_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
    /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   */
    CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
        index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
    {
    }
    unsigned int index_;
    unsigned int x_, y_;
    unsigned int src_x_, src_y_;
};

class WallInflationLayer : public Layer
{
    typedef std::vector< std::vector< unsigned char > > CostMatrix;
    typedef std::vector< std::vector< double        > > DistanceMatrix;

public:
    WallInflationLayer();

    virtual ~WallInflationLayer()
    {
        deleteKernels();
        if (dsrv_)
            delete dsrv_;
    }

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual bool isDiscretized()
    {
        return true;
    }
    virtual void matchSize();

    virtual void reset() { onInitialize(); }

    /** @brief  Given a distance, compute a cost.
    * @param  distance The distance from an obstacle in cells
    * @return A cost value for the distance */
    inline unsigned char computeCost(double distance) const
    {
        unsigned char cost = 0;

        //! Calculate distance cost
        double euclidean_distance = distance * resolution_;
        if( distance == 0 )
        {
            cost = LETHAL_OBSTACLE;
        }
        else if( euclidean_distance <= inscribed_radius_ )    //! Near-obstacle area case
        {
            cost = INSCRIBED_INFLATED_OBSTACLE;
        }
        else
        {
            //! Calculate cost using Gaussian
            double factor;

            factor = -1.0 * exp( -1.0 * ( SQR( euclidean_distance - mean_ )
                                          / ( 2 * dispersion_ ) )
                                 ) + 1;

            cost = (unsigned char)( (ALLMOST_LETHAL_OBSTACLE - 1) * ( factor ) );
        }

        return cost;
    }

    /**
   * @brief Change the values of the inflation radius parameters
   * @param inflation_radius The new inflation radius
   * @param cost_scaling_factor The new weight
   */
    void setInflationParameters(double inflation_radius, double cost_scaling_factor);

protected:
    virtual void onFootprintChanged();
    boost::recursive_mutex* inflation_access_;

private:
    /**
   * @brief Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
    inline double distanceLookup(int mx, int my, int src_x, int src_y)
    {
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);

        if(  ( cached_distances_.size() <= dx )
             || ( cached_distances_.front().size() <= dy ) )
        {
            ROS_ERROR_STREAM_NAMED( "WallInflationLayer", "Error: dx or dy value exceed distance array size:\n"
                                    << "\tMax X: " << cached_costs_.size()
                                    << "\tdx: "    << dx << "\n" );
            if( !cached_distances_.empty() )
            {
                ROS_ERROR_STREAM_NAMED( "WallInflationLayer", "\tMax Y: " << cached_distances_.front().size() << "\tdy:" << dy << "\n" );
            }

            return 0.0;
        }

        return cached_distances_[dx][dy];
    }

    /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
    inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
    {
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        if(  ( cached_costs_.size() <= dx )
             || ( cached_costs_.front().size() <= dy ) )
        {
            ROS_ERROR_STREAM_NAMED( "WallInflationLayer", "Error: dx or dy value exceed cost array size:\n"
                                    << "\tMax X: " << cached_costs_.size()
                                    << "\tdx: "    << dx << "\n" );
            if( !cached_costs_.empty() )
            {
                ROS_ERROR_STREAM_NAMED( "WallInflationLayer", "\tMax Y: " << cached_costs_.front().size() << "\tdy:" << dy << "\n" );
            }
            return costmap_2d::LETHAL_OBSTACLE;
        }

        return cached_costs_[dx][dy];
    }

    void computeCaches();
    void deleteKernels();
    void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

    unsigned int cellDistance(double world_dist)
    {
        return layered_costmap_->getCostmap()->cellDistance(world_dist);
    }

    /**
     * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
     * @param  grid The costmap
     * @param  index The index of the cell
     * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
     * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
     * @param  src_x The x index of the obstacle point inflation started at
     * @param  src_y The y index of the obstacle point inflation started at
     */
    inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                        unsigned int src_x, unsigned int src_y);

    void reconfigureCB(costmap_2d::WallInflationPluginConfig &config, uint32_t level);

    double  inflation_radius_;    ///< Radius for generating decreased gradient from wall
    double  inscribed_radius_;    ///< Radius near wall, where drone center hit the wall
    double  weight_;              ///< Coefficent for decreasing gradient
    double  wall_dead_distance_;  ///< Limit distance for moving drone out of the wall
    bool    inflate_unknown_;
    unsigned int cell_inflation_radius_;            ///< Cell-size inflation radius
    unsigned int cached_cell_processing_radius_;    ///< Previous processed size
    unsigned int cell_wall_dead_distance_;          ///< Cell-size wall-dead distance
    std::map<double, std::vector<CellData> > inflation_cells_;

    double resolution_;

    double dispersion_; ///< Dispersion value for using in Gaussian
    double mean_;       ///< Mean value for using in Gaussian

    std::vector< bool > seen_;      ///< Contains vector of all cells, flagged if cell already visited
    unsigned int seen_size_;

    CostMatrix     cached_costs_;       ///< Matrix of cashed costs for each distances.At i and j used X and Y coords
    DistanceMatrix cached_distances_;   ///< Matrix of cashed distances. At i and j used X and Y coords for distance from (0,0) to (X,Y)

    /** min and max values for computing processing bounds */
    double last_min_x_;
    double last_min_y_;
    double last_max_x_;
    double last_max_y_;

    dynamic_reconfigure::Server<costmap_2d::WallInflationPluginConfig> *dsrv_;

    bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_WALL_INFLATION_LAYER_H_
