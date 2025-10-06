/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Jason Miller & Mingxi Zhou
    Email: jason_miller@uri.edu, mzhou@uri.edu
    Year: 2023

    Copyright (C) 2023 Smart Ocean Systems Laboratory
*/
#ifndef ACOMM_GEOPOINT_HPP_
#define ACOMM_GEOPOINT_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "acomms_msgs/msg/usbl_data.hpp"
#include "robot_localization/srv/to_ll.hpp"

class AcommGeoPoint : public rclcpp::Node
{
public:
  AcommGeoPoint();

private:
  // Callbacks
  void f_usbl_callback(const acomms_msgs::msg::UsblData::SharedPtr msg);

  // Params / frames
  std::string m_tf_prefix;
  std::string m_usbl_frame;
  std::string m_modem_frame;
  std::string m_world_frame;

  // Pub/Sub
  rclcpp::Subscription<acomms_msgs::msg::UsblData>::SharedPtr m_usbl_fix_sub;

  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr m_modem_geopoint_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_modem_point_pub;

  // TF
  tf2_ros::Buffer m_transform_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  // Services
  rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr m_to_ll_client;

};


#endif  // ACOMM_GEOPOINT_HPP_