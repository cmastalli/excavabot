/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Simon Bolivar University.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/
/*
 * Author: Carlos Mastalli
 */

#include <excavaROB_teleop/excavaROB_teleop.h>


TeleopExcavaROBKeyboard::TeleopExcavaROBKeyboard() {
}

TeleopExcavaROBKeyboard::~TeleopExcavaROBKeyboard() {
}

void TeleopExcavaROBKeyboard::keyboardLoop()
{
  char c;
  bool pub = false;

  // Get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  // Setting a new line, the end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOL] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("Use W or S to translate");
  puts("Use A or D to rotate");
  puts("Use Q or E to translate and rotate");
  puts("Press 'CAPS' to run");

  for(;;)
  {
    // Get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    cmd_.linear = cmd_.angular = 0;

    switch(c)
    {
      case KEYCODE_W:
        cmd_.linear = low_vel_linear_;
        pub = true;
	break;
      case KEYCODE_S:
        cmd_.linear = - low_vel_linear_;
        pub = true;
	break;
      case KEYCODE_A:
        cmd_.angular = low_vel_angular_;
        pub = true;
	break;
      case KEYCODE_D:
        cmd_.angular = - low_vel_angular_;
        pub = true;
	break;
      case KEYCODE_Q:
        cmd_.linear = low_vel_linear_;
        cmd_.angular = 0.25 * low_vel_angular_;
        pub = true;
	break;
      case KEYCODE_E:
        cmd_.linear = low_vel_linear_;
        cmd_.angular = - 0.25 * low_vel_angular_;
        pub = true;
	break;

      case KEYCODE_W_CAP:
        cmd_.linear = high_vel_linear_;
        pub = true;
	break;
      case KEYCODE_S_CAP:
        cmd_.linear = - high_vel_linear_;
        pub = true;
	break;
      case KEYCODE_A_CAP:
        cmd_.angular = high_vel_angular_;
        pub = true;
	break;
      case KEYCODE_D_CAP:
        cmd_.angular = - high_vel_angular_;
        pub = true;
	break;
      case KEYCODE_Q_CAP:
        cmd_.linear = high_vel_linear_;
        cmd_.angular = 0.25 * high_vel_angular_;
        pub = true;
	break;
      case KEYCODE_E_CAP:
        cmd_.linear = high_vel_linear_;
        cmd_.angular = - 0.25 * high_vel_angular_;
        pub = true;
	break;
    }

    if(pub == true)
      vel_pub_.publish(cmd_);
  }
}

void TeleopExcavaROBKeyboard::init()
{
  cmd_.linear = cmd_.angular = 0;

  vel_pub_ = node_.advertise<excavaROB_msgs::VelocityBase>("cmd_vel", 1);

  ros::NodeHandle n_private("~");
  n_private.param("low_vel_linear", low_vel_linear_, 0.75);
  n_private.param("high_vel_linear", high_vel_linear_, 1.25);
  n_private.param("low_vel_angular", low_vel_angular_, 1.0);
  n_private.param("high_vel_angular", high_vel_angular_, 1.5);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "excavaROB_base_teleop_keyboard");

  TeleopExcavaROBKeyboard teleop_;
  teleop_.init();

  signal(SIGINT, quit);

  teleop_.keyboardLoop();

  return(0);
}
