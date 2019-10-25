/**
 *
 * Copyright (c) 2018 Carroll Vance.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef PROJECT_ROBOCLAW_ROSCORE_H
#define PROJECT_ROBOCLAW_ROSCORE_H

#include "ros/ros.h"
#include "ros/package.h"

#include "roboclaw_driver.h"

#include "roboclaw/RoboclawEncoderSteps.h"
#include "roboclaw/RoboclawMotorVelocity.h"

namespace roboclaw {

    class roboclaw_roscore {
    public:
        roboclaw_roscore(ros::NodeHandle nh, ros::NodeHandle nh_private);
        ~roboclaw_roscore();

        void run();

    private:
        driver *roboclaw;

        std::map<int, unsigned char> roboclaw_mapping;

        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        ros::Publisher encoder_pub;
        ros::Subscriber velocity_sub;

        ros::Time last_message;

        void velocity_callback(const roboclaw::RoboclawMotorVelocity &msg);
    };


}

#endif //PROJECT_ROBOCLAW_ROSCORE_H
