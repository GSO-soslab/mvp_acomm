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
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

/**
 * @brief Construct a Modem object
 *
 */
AcommGeoPoint::AcommGeoPoint()
{

    nh_.reset(new ros::NodeHandle(""));
    pnh_.reset(new ros::NodeHandle("~"));

    pnh_->param<std::string>("tf_prefix", m_tf_prefix, "wamv_rise");
    pnh_->param<std::string>("usbl_frame_id", m_usbl_frame, "usbl");
    pnh_->param<std::string>("modem_frame_id", m_modem_frame, "modem");
    pnh_->param<std::string>("geopose_frame_id", m_world_frame, "world");

    m_usbl_frame = m_tf_prefix + "/" + m_usbl_frame;
    m_modem_frame = m_tf_prefix + "/" + m_modem_frame;
    m_world_frame = m_tf_prefix + "/" + m_world_frame;

    // //subscriber
    m_odom_geopose_sub = nh_->subscribe("odometry/geopose", 10, &AcommGeoPoint::f_geopose_callback, this);
    m_usbl_fix_sub = nh_->subscribe("usbl/fix", 10, &AcommGeoPoint::f_usbl_callback, this);

    // //publisher
    m_modem_geopose_pub = nh_->advertise<geographic_msgs::GeoPoseStamped>("usbl/modem_geopose", 10);
    m_modem_point_pub = nh_->advertise<geometry_msgs::PointStamped>("usbl/modem_point", 10);
    m_modem_navsatfix_pub = nh_->advertise<sensor_msgs::NavSatFix>("usbl/modem_navsatfix", 10);

    //tf stuff
    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );
}

 void AcommGeoPoint::f_geopose_callback(const geographic_msgs::GeoPoseStampedConstPtr msg)
 {
    m_odom_geopose = *msg;
 }


void AcommGeoPoint::f_usbl_callback(const acomms_msgs::UsblDataConstPtr msg)
{
    //broadcast a tf between USBL and the modem based on UsblData XYZ measurement
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now(); // Set the current time
    transform.header.frame_id = m_usbl_frame; // usbl
    transform.child_frame_id = m_modem_frame; // acomm

    transform.transform.translation.x = msg->xyz.x;
    transform.transform.translation.y = msg->xyz.y;
    transform.transform.translation.z = msg->xyz.z;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    br.sendTransform(transform);

    ///Compute the geopose of the modem.
    try
    {
        //transform to get the ENU from the modem to the world frame.
        geometry_msgs::TransformStamped tf_refenu2acomm = m_transform_buffer.lookupTransform(
            m_world_frame,
            m_modem_frame,
            ros::Time(0),
            ros::Duration(0.1)
            );

        //publish the point
        geometry_msgs::PointStamped point_msg;
        point_msg.header.frame_id = m_world_frame;
        point_msg.header.stamp = ros::Time::now();
        point_msg.point.x = tf_refenu2acomm.transform.translation.x;
        point_msg.point.y = tf_refenu2acomm.transform.translation.y;
        point_msg.point.z = tf_refenu2acomm.transform.translation.z;

        m_modem_point_pub.publish(point_msg);


        //convert relative ENU from world frame to lat long using toLL service (via datum)
        robot_localization::ToLL toll;
        toll.request.map_point.x = tf_refenu2acomm.transform.translation.x;
        toll.request.map_point.y = tf_refenu2acomm.transform.translation.y;
        toll.request.map_point.z = tf_refenu2acomm.transform.translation.z;

        ros::service::call("toLL",toll);

        geographic_msgs::GeoPoseStamped geopose_msg;
        geopose_msg.header.frame_id = m_world_frame;
        geopose_msg.header.stamp = ros::Time::now();
        geopose_msg.pose.position.latitude = toll.response.ll_point.latitude;
        geopose_msg.pose.position.longitude = toll.response.ll_point.longitude;
        geopose_msg.pose.position.altitude = toll.response.ll_point.altitude;

        m_modem_geopose_pub.publish(geopose_msg);

        sensor_msgs::NavSatFix navsatfix_msg;
        navsatfix_msg.header.frame_id = m_world_frame;
        navsatfix_msg.header.stamp = ros::Time::now();
        navsatfix_msg.latitude = toll.response.ll_point.latitude;
        navsatfix_msg.longitude = toll.response.ll_point.longitude;
        navsatfix_msg.altitude = toll.response.ll_point.altitude;
        navsatfix_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
        navsatfix_msg.position_covariance[0] = msg->accuracy;
        navsatfix_msg.position_covariance[4] = msg->accuracy;
        navsatfix_msg.position_covariance[8] = msg->accuracy;
            
        m_modem_navsatfix_pub.publish(navsatfix_msg);

    }
    catch(tf2::TransformException &e)
    {
        ROS_WARN("Can't get the tf from modem to world. Need valid odometry/geopose messages.");
    }


}