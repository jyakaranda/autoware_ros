/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <iostream>

#include "autoware_msgs/ConfigTwistFilter.h"

namespace {

//Publisher
ros::Publisher g_twist_pub;
double g_lateral_accel_limit = 5.0;//设置机器人的最大侧向加速度
double g_lowpass_gain_linear_x = 0.0;//设置x方向线速度低通滤波器的增益
double g_lowpass_gain_angular_z = 0.0;//设置z方向角速度低通滤波器的增益
constexpr double RADIUS_MAX = 9e10;//最大转弯半径
constexpr double ERROR = 1e-8;

void configCallback(const autoware_msgs::ConfigTwistFilterConstPtr &config)
{
  g_lateral_accel_limit = config->lateral_accel_limit;
  ROS_INFO("g_lateral_accel_limit = %lf",g_lateral_accel_limit);
  g_lowpass_gain_linear_x = config->lowpass_gain_linear_x;
  ROS_INFO("lowpass_gain_linear_x = %lf",g_lowpass_gain_linear_x);
  g_lowpass_gain_angular_z = config->lowpass_gain_angular_z;
  ROS_INFO("lowpass_gain_angular_z = %lf",g_lowpass_gain_angular_z);
}

void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{

  double v = msg->twist.linear.x;
  double omega = msg->twist.angular.z;

  if(fabs(omega) < ERROR){
    //若角速度接近于0则直接放送消息
    g_twist_pub.publish(*msg);
    return;
  }

  double max_v = g_lateral_accel_limit / omega;//计算当前角速度下的侧向最大加速度

  geometry_msgs::TwistStamped tp;
  tp.header = msg->header;

  double a = v * omega;
  ROS_INFO("lateral accel = %lf", a);

  //如果当前侧向加速度大于最大侧向加速度，就将x方向线速度设置为max_v
  tp.twist.linear.x = fabs(a) > g_lateral_accel_limit ? max_v
                    : v;
  tp.twist.angular.z = omega;

  static double lowpass_linear_x = 0;
  static double lowpass_angular_z = 0;
  //对线速度和角速度进行低通滤波
  lowpass_linear_x = g_lowpass_gain_linear_x * lowpass_linear_x + (1 - g_lowpass_gain_linear_x) * tp.twist.linear.x;
  lowpass_angular_z = g_lowpass_gain_angular_z * lowpass_angular_z + (1 - g_lowpass_gain_angular_z) * tp.twist.angular.z;

  tp.twist.linear.x = lowpass_linear_x;
  tp.twist.angular.z = lowpass_angular_z;

  ROS_INFO("v: %f -> %f",v,tp.twist.linear.x);
  g_twist_pub.publish(tp);

}
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_filter");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber twist_sub = nh.subscribe("twist_raw", 1, TwistCmdCallback);//订阅pure_pursuit节点的消息
    ros::Subscriber config_sub = nh.subscribe("config/twist_filter", 10, configCallback);//订阅runtime_manager节点的消息，进行参数设置
    g_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);

    ros::spin();
    return 0;
}
