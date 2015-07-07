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

#include "camera_effector_teleop/camera_effector_teleop.h"

#include <signal.h>
#include <iostream>
#include <curses.h> 
#include "std_msgs/Float64.h"
#include <sstream>

namespace camera_effector
{

  Teleoperation::Teleoperation(const std::string& _nh_namespace):
    nh_(_nh_namespace),
    step_(0.02),
    pub_rate_(10)
  {
    nh_.param<double>("pan_joint/limits/min", pan_limits_[0], 1.3962);
    nh_.param<double>("pan_joint/limits/max", pan_limits_[1], -1.3962);

    nh_.param<double>("tilt_joint/limits/min", tilt_limits_[0], 0.6);
    nh_.param<double>("tilt_joint/limits/max", tilt_limits_[1], -0.6);

    nh_.param<std::string>("subscribed_topics/pan_command", topic_pan_cmd_,
      "/camera_effector/pan_command");
    nh_.param<std::string>("subscribed_topics/tilt_command", topic_tilt_cmd_,
      "/camera_effector/tilt_command");

    pan_pub_ = nh_.advertise < std_msgs::Float64 > (topic_pan_cmd_, 1);
    tilt_pub_ = nh_.advertise < std_msgs::Float64 > (topic_tilt_cmd_, 1);

    pan_current_pos_ = 0;
    tilt_current_pos_ = 0;
    key_thread_ = boost::thread(&Teleoperation::captureArrowInput, this);
    pub_thread_ = boost::thread(&Teleoperation::publish, this);
  }


  /*!
   *  Default Destructor
   */
  Teleoperation::~Teleoperation(void)
  {
    key_thread_.join();
    pub_thread_.join();
    endwin();
  }


  void Teleoperation::cmdPan(double cmd)
  {
    std_msgs::Float64 msg;
    msg.data = cmd;
    pan_pub_.publish(msg);
  }


  void Teleoperation::cmdTilt(double cmd)
  {
    std_msgs::Float64 msg;
    msg.data = cmd;
    tilt_pub_.publish(msg);
  }


  void Teleoperation::clear_osstr(std::ostringstream* sstr)
  {
    sstr->str("");
    sstr->flush();
  }


  std::string Teleoperation::craftHeader(void)
  {
    std::string header;
    header = "=========================================================\n";
    header += "             Camera_Effector_Teleoperation               \n";
    header += "=========================================================\n";
    header += "Usage: \r\n\r\n";
    header += "Navigate camera effector by using the following keys:\n";
    header += "[ARROW_UP]    --->  Possitive Tilt command\n";
    header += "[ARROW_DOWN]  --->  Negative Tilt command\n";
    header += "[ARROW_LEFT]  --->  Possitive Pan command\n";
    header += "[ARROW_RIGHT] --->  Negative Pan command\n";
    header += "\r\n\r\n";
    
    return header;
  }


  void Teleoperation::publish(void)
  {
    std_msgs::Float64 msg;
    // Keep ros::ok in order to join thread on interrupt signal received
    while ( ros::ok )
    {
      /* ----< Publish tilt pos >---- */
      msg.data = pan_current_pos_;
      pan_pub_.publish(msg);
      /* ---------------------------- */

      /* ----< Publish tilt pos >---- */
      msg.data = tilt_current_pos_;
      tilt_pub_.publish(msg);
      /* ---------------------------- */
      pub_rate_.sleep();
    }
  }


  void Teleoperation::captureArrowInput(void)
  {
    double cmd = 0;
    int key_pressed;
    std::string header = craftHeader();
    std::string msg;
    std::ostringstream sstr; 

    /* ---< Create new terminal window >--- */
    //initscr();
    newterm(getenv("TERM"), stdout, stdin);
    noecho();
    start_color();
    keypad ( stdscr, TRUE );
    attron(A_BLINK);
    /* ------------------------------------ */

    printw(header.c_str());

    // Keep ros::ok in order to join thread on interrupt signal received
    while ( ros::ok )
    {
      key_pressed = getch();
      switch ( key_pressed )
      {
        case KEY_UP:
          //printw ( "UP\n" );
          tilt_current_pos_ += step_;
          //cmd = tilt_current_pos_; 
          //cmdTilt(cmd);
          break;
        case KEY_DOWN:
          //printw ( "DOWN\n" );
          tilt_current_pos_ -= step_;
          //cmd = tilt_current_pos_; 
          //cmdTilt(cmd);
          break;
        case KEY_LEFT:
          //printw ( "LEFT\n" );
          pan_current_pos_ += step_;
          //cmd = pan_current_pos_; 
          //cmdPan(cmd);
          break;
        case KEY_RIGHT:
          //printw ( "RIGHT\n" );
          pan_current_pos_ -= step_;
          //cmd = pan_current_pos_; 
          //cmdPan(cmd);
          break;
        case KEY_R:
          pan_current_pos_ = 0;
          tilt_current_pos_ = 0;
          cmdPan(0);
          cmdTilt(0);
          break;
        default:
          break;
      }

      clear_osstr(&sstr);
      sstr << pan_current_pos_;
      msg = "Pan Position: " + sstr.str() + ", Tilt Position: ";
      clear_osstr(&sstr);
      sstr << tilt_current_pos_;
      msg += sstr.str() + "\r";

      clrtoeol();
      printw(msg.c_str());
      refresh();
    }

    endwin();
  }

} //namespace 


using camera_effector::Teleoperation;

Teleoperation* teleop;


void int_signal_handler(int sig)
{
  // Send ros::shutdown signal then free teleoperation instance pointer
  ros::shutdown();
  endwin();
  delete teleop;
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_effector_teleop");

  teleop = new Teleoperation("/camera_effector");

  // Catch program interrupt signal --> [C-c]
  signal(SIGINT, int_signal_handler);
  //teleop->captureArrowInput();
  ros::spin();

  return(0);
}
