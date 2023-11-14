/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/**
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include "ros/ros.h"
#include "netft_rdt_driver.h"
#include "geometry_msgs/WrenchStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "netft_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double pub_rate_hz;
  string address;

  bool publish_wrench = false;

  pnh.param("address", address, string("172.16.0.32"));
  pnh.param("pub_rate_hz", pub_rate_hz, 976.0);

  ros::Publisher ready_pub;
  std_msgs::Bool is_ready;
  ready_pub = pnh.advertise<std_msgs::Bool>("netft_ready", 1);
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> netft;
  try
  {
    netft = std::shared_ptr<netft_rdt_driver::NetFTRDTDriver>(new netft_rdt_driver::NetFTRDTDriver(address));
    is_ready.data = true;
    ready_pub.publish(is_ready);
  }
  catch(std::runtime_error)
  {
    is_ready.data = false;
    ready_pub.publish(is_ready);
  }

  ros::Publisher pub;
  if (publish_wrench)
  {
    pub = pnh.advertise<geometry_msgs::Wrench>("netft_data", 3);
  }
  else
  {
    pub = pnh.advertise<geometry_msgs::WrenchStamped>("netft_data", 3);
  }
  ros::Rate pub_rate(pub_rate_hz);
  geometry_msgs::WrenchStamped data;
  geometry_msgs::WrenchStamped data_rot;

  ros::Duration diag_pub_duration(0.01);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time last_diag_pub_time(ros::Time::now());

  while (ros::ok())
  {
    if (netft->waitForNewData())
    {
      netft->getData(data_rot);

      data = data_rot;
      data.wrench.force.x = data_rot.wrench.force.x;
      data.wrench.force.y = data_rot.wrench.force.y;
      data.wrench.force.z = data_rot.wrench.force.z;
      data.wrench.torque.x = data_rot.wrench.torque.x;
      data.wrench.torque.y = data_rot.wrench.torque.y;
      data.wrench.torque.z = data_rot.wrench.torque.z;

      if (publish_wrench)
      {
        //geometry_msgs::Wrench(data.wrench);
        pub.publish(data.wrench);
      }
      else
      {
        pub.publish(data);
      }
    }

    ros::Time current_time(ros::Time::now());
    if ( (current_time - last_diag_pub_time) > diag_pub_duration )
    {
      diag_array.status.clear();
      netft->diagnostics(diag_status);
      diag_array.status.push_back(diag_status);
      diag_array.header.stamp = ros::Time::now();
      diag_pub.publish(diag_array);
      ready_pub.publish(is_ready);
      last_diag_pub_time = current_time;
    }

    ros::spinOnce();
    pub_rate.sleep();
  }

  return 0;
}