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

#include <chrono>
#include <functional>

#include <ros/ros.h>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <acomms_msgs/UsblData.h>

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include <robot_localization/ToLL.h>

class AcommGeoPoint
{

public:
    AcommGeoPoint();

    
private:
    void f_usbl_callback(const acomms_msgs::UsblDataConstPtr msg);
    void f_geopose_callback(const geographic_msgs::GeoPoseStampedConstPtr msg);

    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr pnh_;

    //  //subscriber
    ros::Subscriber m_odom_geopose_sub;
    ros::Subscriber m_usbl_fix_sub;
    
    // //publisher
    ros::Publisher m_modem_geopose_pub;
    ros::Publisher m_modem_point_pub;

    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::StaticTransformBroadcaster br;

    tf2_ros::Buffer m_transform_buffer;

    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;

    std::string m_tf_prefix;
    std::string m_usbl_frame;
    std::string m_modem_frame;
    std::string m_world_frame;

    geographic_msgs::GeoPoseStamped m_odom_geopose;
};

#endif