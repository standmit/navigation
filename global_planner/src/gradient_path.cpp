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
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

#define POS_EPS 0.5f
#define EPS 1e-10f
#define POS_HIGH_F float(POT_HIGH)

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) {
    gradx_ = grady_ = NULL;
}

GradientPath::~GradientPath() {

    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPath::setSize(int xs, int ys) {
    Traceback::setSize(xs, ys);
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

bool GradientPath::getPath(float* potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path)
{
    std::pair<float, float> current;
    int stc = getIndex( int( goal_x ), int( goal_y ) );

    // set up offset
    float dx = float( goal_x - int( goal_x ) );
    float dy = float( goal_y - int( goal_y ) );

    // set up total size
    int ns = xs_ * ys_;

    memset( gradx_, 0, ns * sizeof(float) );
    memset( grady_, 0, ns * sizeof(float) );

    for( int c = 0; c < ns*4; ++c )
    {
        float nx = stc % xs_ + dx;
        float ny = stc / xs_ + dy;

        // check if near goal
        if(  ( std::abs( nx - float( start_x ) ) < POS_EPS )
          && ( std::abs( ny - float( start_y ) ) < POS_EPS ) )
        {
            current.first  = float( start_x );
            current.second = float( start_y );
            path.push_back( current );
            return true;
        }

        // check if out of bounds
        if(  ( stc < xs_ )
          || ( stc > xs_ * ys_ - xs_ ) )
        {
            ROS_ERROR_NAMED( "PathCalc", "Out of bounds" );
            return false;
        }

        current.first  = nx;
        current.second = ny;

        //ROS_INFO("%d %d | %f %f ", stc%xs_, stc/xs_, dx, dy);

        path.push_back( current );

        bool oscillation_detected = false;

        size_t npath = path.size();
        if(  ( npath > 2 )
          && ( std::abs( path[npath - 1].first  - path[npath - 3].first  ) < EPS )
          && ( std::abs( path[npath - 1].second - path[npath - 3].second ) < EPS ) )
        {
            ROS_DEBUG_NAMED("PathCalc", "Oscillation detected, attempting fix.");
            oscillation_detected = true;
        }

        int stcnx = stc + xs_;
        int stcpx = stc - xs_;

        // check for potentials at eight positions near cell
        if(  ( potential[ stc       ] >= POS_HIGH_F )
          || ( potential[ stc + 1   ] >= POS_HIGH_F )
          || ( potential[ stc - 1   ] >= POS_HIGH_F )
          || ( potential[ stcnx     ] >= POS_HIGH_F )
          || ( potential[ stcnx + 1 ] >= POS_HIGH_F )
          || ( potential[ stcnx - 1 ] >= POS_HIGH_F )
          || ( potential[ stcpx     ] >= POS_HIGH_F )
          || ( potential[ stcpx + 1 ] >= POS_HIGH_F )
          || ( potential[ stcpx - 1 ] >= POS_HIGH_F )
          || oscillation_detected )
        {
            ROS_DEBUG_NAMED( "PathCalc", "Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());

            // check eight neighbors to find the lowest
            int   minc = stc;
            float minp = potential[stc];

            int st = stcpx - 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            st = stc - 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            st = stc + 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            st = stcnx - 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }

            stc = minc;
            dx = 0;
            dy = 0;

            //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
            //    potential[stc], path[npath-1].first, path[npath-1].second);

            if( potential[stc] >= POS_HIGH_F )
            {
                ROS_DEBUG_NAMED( "PathCalc", "No path found, high potential" );
                //savemap("navfn_highpot");
                return false;
            }
        }
        // have a good gradient here
        else
        {
            // get grad at four positions near cell
            gradCell( potential, stc     );
            gradCell( potential, stc + 1 );
            gradCell( potential, stcnx     );
            gradCell( potential, stcnx + 1 );

            // get interpolated gradient
            float x1 = ( 1.0f - dx ) * gradx_[ stc   ] + dx * gradx_[ stc   + 1 ];
            float x2 = ( 1.0f - dx ) * gradx_[ stcnx ] + dx * gradx_[ stcnx + 1 ];
            float x  = ( 1.0f - dy ) * x1 + dy * x2; // interpolated x

            float y1 = ( 1.0f - dx ) * grady_[ stc   ] + dx * grady_[ stc   + 1 ];
            float y2 = ( 1.0f - dx ) * grady_[ stcnx ] + dx * grady_[ stcnx + 1 ];
            float y  = ( 1.0f - dy ) * y1 + dy * y2; // interpolated y

            // show gradients
            /*
            ROS_DEBUG_NAMED( "PathCalc", "%0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                             gradx_[ stc     ], grady_[ stc     ],
                             gradx_[ stc+1   ], grady_[ stc+1   ],
                             gradx_[ stcnx   ], grady_[ stcnx   ],
                             gradx_[ stcnx+1 ], grady_[ stcnx+1 ],
                             x, y );
            */

            // check for zero gradient, failed
            if(  ( std::abs( x ) < EPS )
              && ( std::abs( y ) < EPS ) )
            {
                ROS_DEBUG_NAMED( "PathCalc", "Zero gradient");
                return false;
            }

            // move in the right direction
            float ss = pathStep_ / hypotf( x, y );
            dx += x * ss;
            dy += y * ss;

            // check for overflow
            if( dx > 1.0f )
            {
                stc++;
                dx -= 1.0f;
            }

            if( dx < -1.0f )
            {
                stc--;
                dx += 1.0f;
            }

            if( dy > 1.0f )
            {
                stc += xs_;
                dy -= 1.0f;
            }

            if( dy < -1.0f )
            {
                stc -= xs_;
                dy += 1.0f;
            }

        }

        //printf("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //         potential[stc], dx, dy, path[npath-1].first, path[npath-1].second);
    }

    ROS_DEBUG_NAMED( "PathCalc", "Out of cells, goal not found" );
    return false;
}

/*
 int
 NavFn::calcPath(int n, int *st)
 {
 // set up start position at cell
 // st is always upper left corner for 4-point bilinear interpolation
 if (st == NULL) st = start;
 int stc = st[1]*nx + st[0];

 // go for <n> cycles at most
 for (int i=0; i<n; i++)
 {



 }

 //  return npath;            // out of cycles, return failure
 ROS_DEBUG("[PathCalc] No path found, path too long");
 //savemap("navfn_pathlong");
 return 0;            // out of cycles, return failure
 }
 */

//
// gradient calculations
//
// calculate gradient at a cell
// positive value are to the right and down
float GradientPath::gradCell(float* potential, int n)
{
    // check this cell content
    if( gradx_[n] + grady_[n] > 0.0f )
    {
        return 1.0;
    }

    // would be out of bounds
    if(  ( n < xs_ )
      || ( n > xs_ * ys_ - xs_ ) )
    {
        return 0.0;
    }

    float cv = potential[n];
    float dx = 0.0;
    float dy = 0.0;

    // check for in an obstacle
    if( cv >= POS_HIGH_F )
    {
        if( potential[n - 1] < POS_HIGH_F )
        {
            dx = -lethal_cost_;
        }
        else if( potential[n + 1] < POS_HIGH_F )
        {
            dx = lethal_cost_;
        }

        if( potential[n - xs_] < POS_HIGH_F )
        {
            dy = -lethal_cost_;
        }
        else if( potential[n + xs_] < POS_HIGH_F )
        {
            dy = lethal_cost_;
        }
    }
    // not in an obstacle
    else
    {
        // dx calc, average to sides
        if( potential[n - 1] < POS_HIGH_F )
        {
            dx += potential[n - 1] - cv;
        }

        if( potential[n + 1] < POS_HIGH_F )
        {
            dx += cv - potential[n + 1];
        }

        // dy calc, average to sides
        if( potential[n - xs_] < POS_HIGH_F )
        {
            dy += potential[n - xs_] - cv;
        }

        if( potential[n + xs_] < POS_HIGH_F )
        {
            dy += cv - potential[n + xs_];
        }
    }

    // normalize
    float norm = hypotf( dx, dy );
    if( norm > 0 )
    {
        norm = 1.0f / norm;
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }

    return norm;
}

} //end namespace global_planner

