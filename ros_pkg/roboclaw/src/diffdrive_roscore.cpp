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

#include <cmath>
#include <iostream>
#include <string>

#include "diffdrive_roscore.h"
#include "roboclaw/RoboclawMotorVelocity.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"

namespace roboclaw {

    diffdrive_roscore::diffdrive_roscore(ros::NodeHandle nh, ros::NodeHandle nh_private) {

        this->nh = nh;
        this->nh_private = nh_private;

        odom_pub = nh.advertise<nav_msgs::Odometry>(std::string("odom"), 10);
        motor_pub = nh.advertise<roboclaw::RoboclawMotorVelocity>(std::string("motor_cmd_vel"), 10);

        encoder_sub = nh.subscribe(std::string("motor_enc"), 10, &diffdrive_roscore::encoder_callback, this);
        twist_sub = nh.subscribe(std::string("cmd_vel"), 10, &diffdrive_roscore::twist_callback, this);

        last_theta = 0.0;
        last_steps_1 = 0;
        last_steps_2 = 0;

        if(!nh_private.getParam("base_width", base_width)){
            throw std::runtime_error("Must specify base_width!");
        }
        if(!nh_private.getParam("steps_per_meter", steps_per_meter)) {
            throw std::runtime_error("Must specify steps_per_meter!");
        }

        if(!nh_private.getParam("swap_motors", swap_motors))
            swap_motors = true;
        if(!nh_private.getParam("invert_motor_1", invert_motor_1))
            invert_motor_1 = false;
        if(!nh_private.getParam("invert_motor_2", invert_motor_2))
            invert_motor_2 = false;

        if(!nh_private.getParam("var_pos_x", var_pos_x)){
            var_pos_x = 0.01;
        }
        if(!nh_private.getParam("var_pos_y", var_pos_y)){
            var_pos_y = 0.01;
        }
        if(!nh_private.getParam("var_theta_z", var_theta_z)){
            var_theta_z = 0.01;
        }

    }

    void diffdrive_roscore::twist_callback(const geometry_msgs::Twist &msg) {

        roboclaw::RoboclawMotorVelocity motor_vel;
        motor_vel.index = 0;
        motor_vel.mot1_vel_sps = 0;
        motor_vel.mot2_vel_sps = 0;

        // Linear
        motor_vel.mot1_vel_sps += (int) (steps_per_meter * msg.linear.x);
        motor_vel.mot2_vel_sps += (int) (steps_per_meter * msg.linear.x);

        if(msg.linear.y > 0){
            motor_vel.mot2_vel_sps += (int) (steps_per_meter * msg.linear.y);
        }else if(msg.linear.y < 0){
            motor_vel.mot1_vel_sps += (int) (steps_per_meter * msg.linear.y);
        }

        // Angular
        motor_vel.mot1_vel_sps += (int) -(steps_per_meter * msg.angular.z * base_width/2);
        motor_vel.mot2_vel_sps += (int) (steps_per_meter * msg.angular.z * base_width/2);

        if (invert_motor_1)
            motor_vel.mot1_vel_sps = -motor_vel.mot1_vel_sps;

        if (invert_motor_2)
            motor_vel.mot2_vel_sps = -motor_vel.mot2_vel_sps;

        if (swap_motors){
            int tmp = motor_vel.mot1_vel_sps;
            motor_vel.mot1_vel_sps = motor_vel.mot2_vel_sps;
            motor_vel.mot2_vel_sps = tmp;
        }

        motor_pub.publish(motor_vel);
    }

    void diffdrive_roscore::encoder_callback(const roboclaw::RoboclawEncoderSteps &msg) {

        static tf::TransformBroadcaster br;

        int delta_1 = msg.mot1_enc_steps - last_steps_1;
        int delta_2 = msg.mot2_enc_steps - last_steps_2;

        last_steps_1 = msg.mot1_enc_steps;
        last_steps_2 = msg.mot2_enc_steps;

        if (invert_motor_1)
            delta_1 = -delta_1;

        if (invert_motor_2)
            delta_1 = -delta_2;

        if (swap_motors){
            int tmp = delta_1;
            delta_1 = delta_2;
            delta_2 = tmp;
        }

        double u_w = ((delta_1 + delta_2) / steps_per_meter) / 2.0;
        double u_p = ((delta_2 - delta_1) / steps_per_meter);

        double delta_x = u_w * cos(last_theta);
        double delta_y = u_w * sin(last_theta);
        double delta_theta = u_p / base_width;

        double cur_x = last_x + delta_x;
        double cur_y = last_y + delta_y;
        double cur_theta = last_theta + delta_theta;

        nav_msgs::Odometry odom;

        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // Time
        odom.header.stamp = ros::Time::now();

        // Position
        odom.pose.pose.position.x = cur_x;
        odom.pose.pose.position.y = cur_y;

        // Velocity
        odom.twist.twist.linear.x = cur_x - last_x;
        odom.twist.twist.linear.y = cur_y - last_y;
        odom.twist.twist.angular.z = cur_theta - last_theta;

        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, cur_theta);
        odom.pose.pose.orientation.w = quaternion.w();
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();

        // Pos_x Variance
        odom.pose.covariance[0] = var_pos_x;

        // Pos_y Variance
        odom.pose.covariance[7] = var_pos_y;

        // Theta_z Variance
        odom.pose.covariance[35] = var_theta_z;

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(last_x, last_y, 0.0));
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, cur_theta));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));

        odom_pub.publish(odom);

        last_x = cur_x;
        last_y = cur_y;
        last_theta = cur_theta;

    }


}
