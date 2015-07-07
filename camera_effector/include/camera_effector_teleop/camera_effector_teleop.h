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

#ifndef CAMERA_EFFECTOR_TELEOP_CAMERA_EFFECTOR_TELEOP_H
#define CAMERA_EFFECTOR_TELEOP_CAMERA_EFFECTOR_TELEOP_H

#include <cstring>
#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace camera_effector
{

  #define KEY_SPACEBAR 32
  #define KEY_ENTER 10
  #define KEY_ESCAPE 27
  #define KEY_A 97
  #define KEY_B 98
  #define KEY_C 99
  #define KEY_D 100
  #define KEY_E 101
  #define KEY_F 102
  #define KEY_G 103
  #define KEY_H 104
  #define KEY_I 105
  #define KEY_J 106
  #define KEY_K 107
  #define KEY_L 108
  #define KEY_M 109
  #define KEY_N 110
  #define KEY_O 111
  #define KEY_P 112
  #define KEY_Q 113
  #define KEY_R 114
  #define KEY_S 115
  #define KEY_T 116
  #define KEY_U 117
  #define KEY_V 118
  #define KEY_W 119
  #define KEY_X 120
  #define KEY_Y 121
  #define KEY_Z 122


  class Teleoperation
  {
    public:

      /*!
       *  Default Constructor
       */
      Teleoperation(const std::string& _nh_namespace);

      /*!
       *  Default Destructor
       */
      virtual ~Teleoperation(void);

      void cmdTilt(double cmd);

      void cmdPan(double cmd);


      /*!
       * @brief Publish current command values.
       * @return void.
       */
      virtual void publish(void);

      virtual void captureArrowInput(void);


      /*!
       * @brief Crear/Flush ostringstream contents.
       * @param sstr std::ostringstream pointer to clear.
       * @return void.
       */
      void clear_osstr(std::ostringstream* sstr);

      virtual std::string craftHeader(void);

    private:
      ros::NodeHandle nh_;
      ros::Rate pub_rate_;

      double pan_limits_[2];
      double tilt_limits_[2];
      int step_;

      double pan_current_pos_;
      double tilt_current_pos_;

      std::string topic_pan_cmd_;
      std::string topic_tilt_cmd_;
      ros::Publisher pan_pub_;
      ros::Publisher tilt_pub_;

      boost::thread key_thread_;
      boost::thread pub_thread_;
      //boost ::mutex lock_;
  }; 

}

#endif
