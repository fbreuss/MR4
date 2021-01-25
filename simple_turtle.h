/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Hermann Steffan
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
 *   * All advertising materials mentioningfeatures or use of this
 *     software must display the following acknowledgement: “This product
 *     includes software developed by Graz University of Technology and
 *     its contributors.”
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
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
 *********************************************************************/

#ifndef simple_turtle_h___
#define simple_turtle_h___

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/SensorState.h>

#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/DockInfraRed.h>
#include <kobuki_msgs/SensorState.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <tug_2d_occupancy_grid_map/tug_2d_occupancy_grid_map.hpp>

#include <sstream>
#include <fstream>
#include <map>

namespace SimpleTurtle
{

class SimpleTurtle
{
protected:
    ros::NodeHandle nh_;

    ros::Subscriber bumper_sub_;
    ros::Subscriber button_sub_;
    ros::Subscriber infrared_sub_;
    ros::Subscriber laser_scan_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber sensor_state_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher vel_pub_;
    
    tf::TransformBroadcaster br;

    double linear_;
    double angular_;
    bool button_0_pressed_;

    tug_2d_occupancy_grid_map::Tug2dOccupancyGridMap occupancy_grid_map_;


    std::fstream  odomfile;
    std::fstream  posefile;
    
    int poses;
    // ADD YOUR VARIABLES HERE

    bool keep_logging_ = true;

    double get_probability_of_cell_occupied(float dist_pixel, float dist_laser);

    double laser_precision_ = 1.0f;

    double p_occupied_ = 0.8f;
    double p_free_ = 0.2f;
    double p_unknown_ = 0.5f;

    double p_ratio_ = p_occupied_ / p_free_;
    double p_ratio_inv_ = 1.0f / p_ratio_;

    int number_of_measurements_ = 0;

    geometry_msgs::Pose pose_;
    std::map<int, std::map<int, int> > hits_;
    std::map<int, std::map<int, int> > interceptions_;
    

public:
    SimpleTurtle();

    virtual ~SimpleTurtle();

    void init();

    void step();


protected:
    void pubCmdVel(double linear, double angular);

    void bumberCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

    void buttonCallback(const kobuki_msgs::ButtonEvent::ConstPtr& msg);

    void dockingCallback(const kobuki_msgs::DockInfraRed::ConstPtr& msg);

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

    void sensorStateCallback(const kobuki_msgs::SensorState::ConstPtr& msg);
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
   
    void publishTransform(tf::Vector3 translation, tf::Quaternion rotation);
    
    void dump_pose();
    void dump_odom();

};

} // end namespace SimpleTurtle

#endif // simple_turtle_h___
