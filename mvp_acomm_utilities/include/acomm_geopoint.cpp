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

    this->declare_parameter("reference_enu_frame_id", "reference");
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

    evologics_usbl_sub = this->create_subscription<mvp_acomm_interfaces::msg::UsblData>("usbl_data", 10,
                                                                std::bind(&AcommGeoPoint::f_usbl_callback,
                                                                this, _1));

    // //publisher
    m_acomm_geopoint_pub = this->create_publisher<geographic_msgs::msg::GeoPointStamped>("acomm_geopose", 10);
    m_usbl_geopose_pub = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("usbl_geopose", 10);

    //tf stuff
    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

}

 void AcommGeoPoint::f_geopose_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
 {
    // printf("got geopose\r\n");
    // //get reference geopose
    m_refenu_pose = *msg;
    m_refenu_pose.header.frame_id = m_refenu_frame;
    m_refenu_pose.pose.orientation.x = 0.0;
    m_refenu_pose.pose.orientation.y = 0.0;
    m_refenu_pose.pose.orientation.z = 0.0;
    m_refenu_pose.pose.orientation.w = 1.0;

    // // //publish the tf between an reference frame and a virtual frame with enu.
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now(); // Set the current time
    // transform.header.frame_id = m_refenu_frame; 
    // transform.child_frame_id = msg->header.frame_id; 

    transform.header.frame_id = m_ref_frame;  //geopose frame
    transform.child_frame_id = m_refenu_frame; // The new frame ID
    // printf("%s->%s\r\n", m_ref_frame.c_str(), m_refenu_frame.c_str());

    transform.transform.translation.x = 0;
    transform.transform.translation.y = 0;
    transform.transform.translation.z = 0;

    geometry_msgs::msg::Quaternion inverse_orientation;
    inverse_orientation.x = -msg->pose.orientation.x;
    inverse_orientation.y = -msg->pose.orientation.y;
    inverse_orientation.z = -msg->pose.orientation.z;
    inverse_orientation.w = msg->pose.orientation.w;

    // Assign the inverse orientation to the transform
    transform.transform.rotation = inverse_orientation;

    static_broadcaster_->sendTransform(transform);

 }


void AcommGeoPoint::f_usbl_callback(const mvp_acomm_interfaces::msg::UsblData::SharedPtr msg)
{
    m_usbl_frame = msg->header.frame_id;
    m_usbl_geopose.header = msg->header;
    ////////////////////////////////get usbl geopose////////////////////
    //calculate usbl geopose using tf
    try 
    {        
        geometry_msgs::msg::TransformStamped tf_refenu2usbl = m_transform_buffer->lookupTransform(
            m_refenu_frame,
            m_usbl_frame,
            tf2::TimePointZero,
            10ms
            );

        geometry_msgs::msg::Point usbl_point;
        usbl_point.x = tf_refenu2usbl.transform.translation.x;
        usbl_point.y = tf_refenu2usbl.transform.translation.y;
        usbl_point.z = tf_refenu2usbl.transform.translation.z;

        // printf("usbl_point, x=%lf, and y%lf\r\n", usbl_point.x, usbl_point.y);
        //get orientation for the usbl based on tf
        if(m_use_ref_geopose_orientation){
            tf2::Quaternion q_refenu_to_usbl(tf_refenu2usbl.transform.rotation.x, 
                                            tf_refenu2usbl.transform.rotation.y, 
                                            tf_refenu2usbl.transform.rotation.z, 
                                            tf_refenu2usbl.transform.rotation.w);

            // Normalize the result to ensure it's a valid unit quaternion
            q_refenu_to_usbl.normalize();
            m_usbl_geopose.pose.orientation.x = q_refenu_to_usbl.x();
            m_usbl_geopose.pose.orientation.y = q_refenu_to_usbl.y();
            m_usbl_geopose.pose.orientation.z = q_refenu_to_usbl.z();
            m_usbl_geopose.pose.orientation.w = q_refenu_to_usbl.w();
        }
        else{
            //update usbl orientation
            m_usbl_geopose.pose.orientation = msg->orientation;
        }

        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

        double lat, lon;
        // Calculate latitude and longitude of the usbl based on latitude and longitude of the reference enu
        geod.Direct(m_refenu_pose.pose.position.latitude, m_refenu_pose.pose.position.longitude, 
                    0.0, usbl_point.y, 
                    lat, lon); // Move north
        geod.Direct(lat, lon, 90.0, usbl_point.x, 
                    m_usbl_geopose.pose.position.latitude, m_usbl_geopose.pose.position.longitude); // Move east

        //publish m_usbl_geopose
        m_usbl_geopose_pub->publish(m_usbl_geopose);

    } 
    catch(tf2::TransformException &e) 
    {
        RCLCPP_WARN(get_logger(), "Can't get the tf from refenu to usbl callback");
    }
    ////////////////////////////////

    // //////////////////////////get acomm geopose////////////////////
    //broadcast a tf between USBL and the acomm
    //publish a tf between usbl and the acomm //assuming acomm has the same orientation as usbl
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now(); // Set the current time
    transform.header.frame_id = m_usbl_frame; // usbl
    transform.child_frame_id = m_acomm_frame; // acomm

    transform.transform.translation.x = msg->xyz.x;
    transform.transform.translation.y = msg->xyz.y;
    transform.transform.translation.z = msg->xyz.z;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(transform);

    ///Compute the geo pose of the acomm.
    try
    {
        geometry_msgs::msg::TransformStamped tf_refenu2acomm = m_transform_buffer->lookupTransform(
            m_refenu_frame,
            m_acomm_frame,
            tf2::TimePointZero,
            10ms
            );

        geometry_msgs::msg::Point acomm_point;
        acomm_point.x = tf_refenu2acomm.transform.translation.x;
        acomm_point.y = tf_refenu2acomm.transform.translation.y;
        acomm_point.z = tf_refenu2acomm.transform.translation.z;
        
        // printf("acomm, x=%lf, and y%lf\r\n", acomm_point.x, acomm_point.y);

        //calculate lat lon
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

        double lat, lon;
        // Calculate latitude and longitude of the usbl based on latitude and longitude of the reference enu
        geod.Direct(m_refenu_pose.pose.position.latitude, m_refenu_pose.pose.position.longitude, 
                    0.0, acomm_point.y, 
                    lat, lon); // Move north
        geod.Direct(lat, lon, 90.0, acomm_point.x, 
                    m_acomm_geopoint.position.latitude, m_acomm_geopoint.position.longitude); // Move east

        m_acomm_geopoint.header = msg->header;
        m_acomm_geopoint.header.frame_id = m_acomm_frame;
        m_acomm_geopoint.position.altitude = acomm_point.z;
        //publish m_usbl_geopose
        m_acomm_geopoint_pub->publish(m_acomm_geopoint);
    }
    catch(tf2::TransformException &e)
    {
        RCLCPP_WARN(get_logger(), "Can't get the tf from in acomm to ref_enu callback");
    }


}