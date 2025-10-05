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
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <GeographicLib/Geodesic.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

/**
 * @brief Construct a Modem object
 *
 */
AcommGeoPoint::AcommGeoPoint() : Node("acomm_geopoint_node")
{

    this->declare_parameter("use_reference_geopose_orientation", false);
    this->get_parameter("use_reference_geopose_orientation", m_use_ref_geopose_orientation);

    this->declare_parameter("tf_prefix", "");
    this->get_parameter("tf_prefix", m_tf_prefix);

    this->declare_parameter("geopose_frame_id","");
    this->get_parameter("geopose_frame_id", m_ref_frame);

    this->declare_parameter("reference_enu_frame_id", "world");
    this->get_parameter("reference_enu_frame_id", m_refenu_frame);

    this->declare_parameter("acomm_frame_id", "acomm");
    this->get_parameter("acomm_frame_id", m_acomm_frame);

    m_ref_frame = m_tf_prefix + "/" + m_ref_frame;
    m_refenu_frame = m_tf_prefix + "/" + m_refenu_frame;
    m_acomm_frame = m_tf_prefix + "/" + m_acomm_frame;

    // //subscriber
    m_ref_geopose_sub = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>("reference_geopose", 10, 
                                                                std::bind(&AcommGeoPoint::f_geopose_callback, 
                                                                this, _1));

    evologics_usbl_sub = this->create_subscription<acomms_msgs::msg::UsblData>("usbl_data", 10,
                                                                std::bind(&AcommGeoPoint::f_usbl_callback2,
                                                                this, _1));

    // //publisher
    m_acomm_geopoint_pub = this->create_publisher<geographic_msgs::msg::GeoPointStamped>("acomm_geopoint", 10);
    m_usbl_geopose_pub = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("usbl_geopose", 10);

    //Cliet 
    toLL_client_ = this->create_client<robot_localization::srv::ToLL>("toLL");

    //tf stuff
    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

}

 void AcommGeoPoint::f_geopose_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
 {
    // // printf("got geopose\r\n");
    // // //get reference geoposem_acomm_geopoint_pub
    // m_refenu_pose = *msg;
    // m_refenu_pose.header.frame_id = m_refenu_frame;
    // m_refenu_pose.pose.orientation.x = 0.0;
    // m_refenu_pose.pose.orientation.y = 0.0;
    // m_refenu_pose.pose.orientation.z = 0.0;
    // m_refenu_pose.pose.orientation.w = 1.0;

    // // // //publish the tf between an reference frame and a virtual frame with enu.
    // geometry_msgs::msg::TransformStamped transform;
    // transform.header.stamp = this->now(); // Set the current time
    // // transform.header.frame_id = m_refenu_frame; 
    // // transform.child_frame_id = msg->header.frame_id; 

    // transform.header.frame_id = m_ref_frame;  //geopose frame
    // transform.child_frame_id = m_refenu_frame; // The new frame ID
    // // printf("%s->%s\r\n", m_ref_frame.c_str(), m_refenu_frame.c_str());

    // transform.transform.translation.x = 0;
    // transform.transform.translation.y = 0;
    // transform.transform.translation.z = 0;

    // geometry_msgs::msg::Quaternion inverse_orientation;
    // inverse_orientation.x = -msg->pose.orientation.x;
    // inverse_orientation.y = -msg->pose.orientation.y;
    // inverse_orientation.z = -msg->pose.orientation.z;
    // inverse_orientation.w = msg->pose.orientation.w;

    // // Assign the inverse orientation to the transform
    // transform.transform.rotation = inverse_orientation;

    // static_broadcaster_->sendTransform(transform);

 }



void AcommGeoPoint::f_usbl_callback2(const acomms_msgs::msg::UsblData::SharedPtr msg)
{
    //make a pointstamped message for acomm in usbl frame
    geometry_msgs::msg::PointStamped acomm_point_in_usbl;

    acomm_point_in_usbl.header = msg->header;
    acomm_point_in_usbl.point.x = msg->xyz.x;
    acomm_point_in_usbl.point.y = msg->xyz.y;
    acomm_point_in_usbl.point.z = msg->xyz.z;


    //convert the point from usbl frame into enu frame using transform
    geometry_msgs::msg::PointStamped acomm_point_in_enu;
    acomm_point_in_usbl.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(); // Set to current time
    
    try{
            acomm_point_in_enu = m_transform_buffer->transform(acomm_point_in_usbl, m_refenu_frame, tf2::durationFromSec(1.0));

    }
    catch (tf2::TransformException &ex) {
            auto steady_clock = rclcpp::Clock();
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Could NOT transform the acomm point"));
            return;
    }
    f_call_toLL(acomm_point_in_enu);

    
}

void AcommGeoPoint::f_call_toLL(const geometry_msgs::msg::PointStamped &point_stamped)
{
    //call the servce to make the point in to latitude and longitude and publish in geopoint
    auto request = std::make_shared<robot_localization::srv::ToLL::Request>();
    request->map_point = point_stamped.point;
    m_acomm_geopoint.header = point_stamped.header;

    // Wait for the service to be available
    if (!toLL_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service /toLL not available");
      return;
    }

    printf("calling the service \r\n");
    // auto future = toLL_client_->async_send_request(request);
    
    auto future = toLL_client_->async_send_request(
    request,
    [this](rclcpp::Client<robot_localization::srv::ToLL>::SharedFuture result_future)
    {
        auto response = result_future.get();
        m_acomm_geopoint.header.frame_id = m_acomm_frame;
        m_acomm_geopoint.position.latitude = response->ll_point.latitude;
        m_acomm_geopoint.position.longitude = response->ll_point.longitude;
        m_acomm_geopoint.position.altitude = response->ll_point.altitude;

        m_acomm_geopoint_pub->publish(m_acomm_geopoint);
    });

}
