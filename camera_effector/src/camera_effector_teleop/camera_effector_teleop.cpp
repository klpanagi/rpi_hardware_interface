/*********************************************************************
 * *
 * * Software License Agreement (BSD License)
 * *
 * *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
 * *  All rights reserved.
 * *
 * *  Redistribution and use in source and binary forms, with or without
 * *  modification, are permitted provided that the following conditions
 * *  are met:
 * *
 * *   * Redistributions of source code must retain the above copyright
 * *     notice, this list of conditions and the following disclaimer.
 * *   * Redistributions in binary form must reproduce the above
 * *     copyright notice, this list of conditions and the following
 * *     disclaimer in the documentation and/or other materials provided
 * *     with the distribution.
 * *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 * *     contributors may be used to endorse or promote products derived
 * *     from this software without specific prior written permission.
 * *
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * *  POSSIBILITY OF SUCH DAMAGE.
 * *
 * * Author: Konstantinos Panayiotou,   
 * * Maintainer: Konstantinos Panayiotou
 * * Email: klpanagi@gmail.com
 * *
 * *********************************************************************/

#include "ros/ros.h"

#include "camera_effector_teleop/camera_effector_teleop.h"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Twist.h>
#include <signal.h>
//#include <stdio.h>
#include <iostream>
#include <curses.h> 


class Teleoperation
{
  public:
    Teleoperation(double max_linear = 0.5, double max_angular = 0.8);
    ~Teleoperation(void);
    double getLinearScale(void);
    double getAngularScale(void);
    void publishTwist(void);
    void captureArrowInput(void);

  private:
    ros ::NodeHandle nh_;
    double linear_, linear_scale_;
    double angular_, angular_scale_;
    ros ::Publisher twist_pub_;
    boost ::thread pub_thread_;
    boost ::mutex lock_;
};

Teleoperation::Teleoperation(
  double max_linear, double max_angular) :
  linear_(0),
  angular_(0),
  linear_scale_(max_linear),
  angular_scale_(max_angular)
{
  nh_.param("scale_linear", linear_scale_, linear_scale_);
  nh_.param("scale_angular", angular_scale_, angular_scale_);
  twist_pub_ = nh_.advertise < geometry_msgs::Twist > ("cmd_pos", 1);
  pub_thread_ = boost::thread(&Teleoperation::publishTwist, this);
}

Teleoperation::~Teleoperation(void)
{
  pub_thread_.join();
}

double Teleoperation::getLinearScale(void)
{
  return linear_scale_;
}

double Teleoperation::getAngularScale(void)
{
  return angular_scale_;
}

void Teleoperation::publishTwist(void)
{
  geometry_msgs::Twist twist;
  ros::Rate rate(100);
  while (ros::ok)
  {
    twist.linear .x = linear_ * linear_scale_;
    twist.angular .z = angular_ * angular_scale_;
    {
      boost::mutex::scoped_lock lock(lock_);
      twist_pub_.publish(twist);
    }
    rate.sleep();
  }
}

void Teleoperation::captureArrowInput(void)
{
  initscr();
  noecho();

  keypad ( stdscr, TRUE );

  while ( 1 )
  {
    switch ( getch() )
    {
      case KEY_UP:
        printw ( "UP\n" );
        break;
      case KEY_DOWN:
        printw ( "DOWN\n" );
        break;
      case KEY_LEFT:
        printw ( "LEFT\n" );
        break;
      case KEY_RIGHT:
        printw ( "RIGHT\n" );
        break;
    }
    refresh();
  }

  endwin();
}


void quit(int sig)
{
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleoperation");
  Teleoperation* teleoperation;

  if ( argc == 3 )
  {
    teleoperation = new Teleoperation(atof(argv[1]), atof(argv[2]));
  }
  else
  {
    teleoperation = new Teleoperation();

    std ::cout << "---------------------\n"
      << "Maximum linear velocity was not specified, defaults to "
      << teleoperation->getLinearScale()
      << " m/s.\n";
    std ::cout << "---------------------\n"
      << "Maximum angular velocity was not specified, defaults to "
      << teleoperation->getAngularScale()
      << " r/s.\n";
  }

  // Catch program interrupt signal --> [C-c]
  signal(SIGINT, quit);
  //teleoperation->keyLoop();
  return(0);
}
