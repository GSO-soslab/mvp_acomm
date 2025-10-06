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

    Author: Jason Miller
    Email: jason_miller@uri.edu
    Year: 2023

    Copyright (C) 2023 Smart Ocean Systems Laboratory
*/

#include "acomm_geopoint.hpp"

using std::placeholders::_1;

AcommGeoPoint::AcommGeoPoint()
: Node("acomm_geopoint"),
  m_transform_buffer(this->get_clock())
{
  // Declare & get parameters
  this->declare_parameter<std::string>("tf_prefix", "wamv_rise");
  this->declare_parameter<std::string>("usbl_frame_id", "usbl");
  this->declare_parameter<std::string>("modem_frame_id", "modem");
  this->declare_parameter<std::string>("world_frame_id", "world");

  this->get_parameter("tf_prefix", m_tf_prefix);
  this->get_parameter("usbl_frame_id", m_usbl_frame);
  this->get_parameter("modem_frame_id", m_modem_frame);
  this->get_parameter("world_frame_id", m_world_frame);

  m_usbl_frame  = m_tf_prefix + "/" + m_usbl_frame;
  m_modem_frame = m_tf_prefix + "/" + m_modem_frame;
  m_world_frame = m_tf_prefix + "/" + m_world_frame;

  // ROS Subscribers
  m_usbl_fix_sub = this->create_subscription<acomms_msgs::msg::UsblData>("usbl/fix", 10, std::bind(&AcommGeoPoint::f_usbl_callback, this, _1));

  // ROS Publishers
  m_modem_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("usbl/modem_point", 10);
  m_modem_geopoint_pub = this->create_publisher<geographic_msgs::msg::GeoPointStamped>("usbl/modem_geopoint", 10);

  // TF
  m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Service client to robot_localization ToLL
  m_to_ll_client = this->create_client<robot_localization::srv::ToLL>("toLL");
}

void AcommGeoPoint::f_usbl_callback(const acomms_msgs::msg::UsblData::SharedPtr msg)
{
  //broadcast a tf between usbl and modem based on the usbl xyz
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = m_usbl_frame;   // parent (USBL)
  transform.child_frame_id  = m_modem_frame;  // child  (modem)

  transform.transform.translation.x = msg->xyz.x;
  transform.transform.translation.y = msg->xyz.y;
  transform.transform.translation.z = msg->xyz.z;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  m_tf_broadcaster->sendTransform(transform);

  //Lookup world -> modem transform to get modem ENU in world
  try
  {
    auto tf_refenu2acomm = m_transform_buffer.lookupTransform(
        m_world_frame,
        m_modem_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.1));

    //Publish PointStamped in world
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.frame_id = m_world_frame;
    point_msg.header.stamp = this->now();
    point_msg.point.x = tf_refenu2acomm.transform.translation.x;
    point_msg.point.y = tf_refenu2acomm.transform.translation.y;
    point_msg.point.z = tf_refenu2acomm.transform.translation.z;
    m_modem_point_pub->publish(point_msg);

    //Convert ENU->LLA via robot_localization ToLL service
    if (!m_to_ll_client->wait_for_service(std::chrono::milliseconds(50))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Service 'toLL' not available yet.");
      return;
    }

    auto req = std::make_shared<robot_localization::srv::ToLL::Request>();
    req->map_point.x = tf_refenu2acomm.transform.translation.x;
    req->map_point.y = tf_refenu2acomm.transform.translation.y;
    req->map_point.z = tf_refenu2acomm.transform.translation.z;

    auto future = m_to_ll_client->async_send_request(req);
    auto ret = rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::milliseconds(200));

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "toLL call failed or timed out.");
      return;
    }

    const auto resp = future.get();

    //Publish GeoPointStamped
    geographic_msgs::msg::GeoPointStamped geopoint_msg;
    geopoint_msg.header.frame_id = m_world_frame;
    geopoint_msg.header.stamp = this->now();
    geopoint_msg.position.latitude  = resp->ll_point.latitude;
    geopoint_msg.position.longitude = resp->ll_point.longitude;
    geopoint_msg.position.altitude  = resp->ll_point.altitude;
    m_modem_geopoint_pub->publish(geopoint_msg);
  }
  catch (const tf2::TransformException & e)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "TF lookup modem->world failed: %s. Need valid odometry/geopose.",
                         e.what());
  }
}
