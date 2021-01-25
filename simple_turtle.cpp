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

#include <tug_simple_turtle/simple_turtle.h>
#include <stdio.h>
#include <math.h>

namespace SimpleTurtle
{

SimpleTurtle::SimpleTurtle() :
    nh_(),
    linear_(0.0),
    angular_(0.0),
    button_0_pressed_(false),
    occupancy_grid_map_(nh_)
{

	poses = 0;
	
	posefile.open("pose.dat", std::ios::out|std::ios::trunc);
	odomfile.open("odom.dat", std::ios::out|std::ios::trunc);
		
}

SimpleTurtle::~SimpleTurtle()
{
	posefile.close();
	odomfile.close();
}

void SimpleTurtle::init()
{
    occupancy_grid_map_.init(2048, 2048, 0.05, -51.2, -51.2);

    bumper_sub_ = nh_.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 10, &SimpleTurtle::bumberCallback, this);
    button_sub_ = nh_.subscribe<kobuki_msgs::ButtonEvent>("/mobile_base/events/button", 10, &SimpleTurtle::buttonCallback, this);
    infrared_sub_ = nh_.subscribe<kobuki_msgs::DockInfraRed>("/mobile_base/events/", 10, &SimpleTurtle::dockingCallback, this);

    laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &SimpleTurtle::laserScanCallback, this);

    pose_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("/pose2D", 10, &SimpleTurtle::poseCallback, this);
    
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 10, &SimpleTurtle::odomCallback, this);

    sensor_state_sub_ = nh_.subscribe("/sensor_state", 10, &SimpleTurtle::sensorStateCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

// publish cmd_vel
void SimpleTurtle::pubCmdVel(double linear, double angular)
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    vel_pub_.publish(vel);
}

// Bumper
void SimpleTurtle::bumberCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    ROS_INFO("Bumper");

    switch(msg->state)
    {
    case kobuki_msgs::BumperEvent::CENTER:
        ROS_INFO("Center");
        break;
    case kobuki_msgs::BumperEvent::LEFT:
        ROS_INFO("Left");
        break;
    case kobuki_msgs::BumperEvent::RIGHT:
        ROS_INFO("Right");
        break;

    }
}


// Buttons
void SimpleTurtle::buttonCallback(const kobuki_msgs::ButtonEvent::ConstPtr& msg)
{
    ROS_INFO("Buttons");
    switch(msg->button)
    {
    case kobuki_msgs::ButtonEvent::Button0:
        ROS_INFO("Button 0");
        button_0_pressed_ = true;
        break;
    case kobuki_msgs::ButtonEvent::Button1:
        ROS_INFO("Button 1");
        break;
    case kobuki_msgs::ButtonEvent::Button2:
        ROS_INFO("Button 2");
        break;
    }
}


// Docking sensor
void SimpleTurtle::dockingCallback(const kobuki_msgs::DockInfraRed::ConstPtr& msg)
{
    ROS_INFO("Docking");
}

// Sensor State
void SimpleTurtle::sensorStateCallback(const kobuki_msgs::SensorState::ConstPtr& msg)
{
    ROS_INFO_STREAM("left encoder= " << msg->left_encoder);
    ROS_INFO_STREAM("right encoder= " << msg->right_encoder);
    
    // MAKE YOUR CHANGES HERE
    
    // don't forget to call publishTransform at the end
    
    // LOG YOUR ODOMETRY DATA
}

double SimpleTurtle::get_probability_of_cell_occupied(float dist_pixel, float dist_laser)
{
  double dist = dist_laser - dist_pixel;

  // in front of the obstacle
  if (dist > laser_precision_)
    return p_free_;

  // inside the obstacle
  if (abs(dist) <= laser_precision_)
    return p_occupied_;

  // behind the obstacle
  return p_unknown_;
}

// PointCloud
void SimpleTurtle::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("laser scan");

  if (keep_logging_ == false)
  {
    std::cout << "     - SKIP" << std::endl;
    return;
  }

  double l0 = 0.0f;

  // map parameters
  int mapX = occupancy_grid_map_.getSizeX();
  int mapY = occupancy_grid_map_.getSizeY();
  double map_radius = sqrt(mapX * mapX + mapY * mapY);

  // quaternion to YAW
  geometry_msgs::Pose robot_pose = pose_;
  geometry_msgs::Quaternion q = robot_pose.orientation;
  double robot_yaw = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

  // get ranges in [cellsize]
  float range_min = msg->range_min / occupancy_grid_map_.getResolution();
  float range_max = msg->range_max / occupancy_grid_map_.getResolution();

  // for all laser measurements
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    // laser range in [cellsize] - valid
    float range = msg->ranges[i] / occupancy_grid_map_.getResolution();
    if (range < range_min || range > range_max)
      continue; // invalid laser data
    
    double angle_laser = msg->angle_min + msg->angle_increment * i;
    double angle = robot_yaw + angle_laser;

    // Bresenham algorithm copied from https://de.wikipedia.org/wiki/Bresenham-Algorithmus (21.01.2021)
    int x0, y0;
    occupancy_grid_map_.worldToMap(robot_pose.position.x, robot_pose.position.y, x0, y0);
    int x_init = x0, y_init = y0;

    int x1 = x0 + map_radius * cos(angle);
    int y1 = x0 + map_radius * sin(angle);
    
    int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = dx+dy, e2; /* error value e_xy */
    
    float d = 0.0f;
    while (1)
    {
      // break if point is outside of map
      if (x0 < 0 || y0 < 0 || x0 >= mapX || y0 >= mapY)
        break;
      
      // valid sensor data
      d = sqrt(pow(x_init - x0, 2) + pow(y_init - y0, 2));
      //std::cout << " " << d << " " << range << " " << (d / range) << " " << (inverse_sensor_model(d / range) - l0) << std::endl;
      double probability_of_cell_occupied = get_probability_of_cell_occupied(d, range);
      double inverse_sensor_model = log(probability_of_cell_occupied / (1 - probability_of_cell_occupied));
      double new_value = occupancy_grid_map_.getCell(x0, y0) + inverse_sensor_model - l0;
      occupancy_grid_map_.updateCell(x0, y0, new_value);

      // update hits / interceptions
      if (keep_logging_)
      {
        if (probability_of_cell_occupied >= 0.6f) // hit
          hits_[y0][x0]++;
        else if (probability_of_cell_occupied <= 0.4f) // interception
          interceptions_[y0][x0]++;
      }

      if (x0==x1 && y0==y1) break;
      e2 = 2*err;
      if (e2 > dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
      if (e2 < dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */
    }
  }

  number_of_measurements_++;

  // Task 2
  if (button_0_pressed_ && keep_logging_)
  {
    keep_logging_ = false;
    std::cout << "OPTIMIZZZZZE" << std::endl;
    for (int y = 0; y < mapY; y++)
    {
      for (int x = 0; x < mapX; x++)
      {
        int sum = hits_[y][x] + interceptions_[y][x];
        if (sum == 0)
          continue;
        
        double P_hit = (double)hits_[y][x] / sum;

        double P_interception = 1 - P_hit;

        double p_cell = pow(p_ratio_, P_hit * number_of_measurements_) * pow(p_ratio_inv_, P_interception * number_of_measurements_);

        // flip cells
        if (p_cell >= p_occupied_)
          occupancy_grid_map_.updateCell(x, y, 10);
        else
          occupancy_grid_map_.updateCell(x, y, -10);
      }
    }
    std::cout << "DOOOOOOOOOONE" << std::endl;
  }
}

// Robot Pose in World
void SimpleTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("odom");
    pose_ = msg->pose.pose;
}

// Scan Matcher Pose
void SimpleTurtle::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_INFO("pose");
    // LOG LASER SCAN MATCHER POSE DATA
}


void SimpleTurtle::dump_pose()
{
	posefile << "happy logging\n";
}

void SimpleTurtle::dump_odom()
{
	odomfile << "happy logging\n";
}

void SimpleTurtle::publishTransform(tf::Vector3 translation, tf::Quaternion rotation)
{
	tf::Transform t;
	t.setOrigin(translation);
	t.setRotation(rotation);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "odom", "my_base"));
}

// step function
// called with 10Hz
void SimpleTurtle::step()
{
    if(button_0_pressed_)
    {
        // MAKE YOUR CHANGES HERE
        
        linear_ = 0.0;
        angular_ = 0.0;

        pubCmdVel(linear_,angular_);
        pubCmdVel(linear_,angular_);
        
    }
}

} // end namespace SimpleTurtle

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_turtle");

    SimpleTurtle::SimpleTurtle simple_turtle;

    simple_turtle.init();

    // main loop at 10Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        simple_turtle.step();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}