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

#include "acomms_localization.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <GeographicLib/Geodesic.hpp>


/**
 * @brief Construct a Modem object
 *
 */
AcommsLocalization::AcommsLocalization()
{
    nh_.reset(new ros::NodeHandle(""));
    pnh_.reset(new ros::NodeHandle("~"));

    pnh_->param<bool>("external_heading", external_heading_enabled_, false);

    ship_geopose_sub_ = nh_->subscribe("ship_geopose", 10, &AcommsLocalization::shipGeoposeCallback, this);
    evologics_usbl_sub_ = nh_->subscribe("evologics/usbllong", 10, &AcommsLocalization::usblCallback, this);

    usbl_orientation_pub_ = nh_->advertise<geometry_msgs::QuaternionStamped>("evologics/orientation", 10);
    track_latlong_pub_ = nh_->advertise<geographic_msgs::GeoPointStamped>("evologics/track/latlong", 10);

}

/**
 * @brief Destroy the AcommsLocalization:: AcommsLocalization object
 *
 */
AcommsLocalization::~AcommsLocalization()
{
}

/**
 * @brief Subscriber callback to the vessels geopose
 * 
 * @param msg 
 */
void AcommsLocalization::shipGeoposeCallback(const geographic_msgs::GeoPoseConstPtr &msg)
{
    std::unique_lock<std::mutex> lock(ship_geopose_mutex_);

    ship_geopose_ = *msg;
}

/**
 * @brief Subscriber callback to the evologics usbllong msg from the modem node
 * 
 * @param msg 
 */
void AcommsLocalization::usblCallback(const mvp_acomms::EvologicsUsbllongConstPtr &msg)
{
    std::unique_lock<std::mutex> lock(ship_geopose_mutex_);

    // First we will publish the usbl's orientation 
    geometry_msgs::QuaternionStamped usbl_orientation;
    tf2::Quaternion quat_tf;
    
    //if external_heading_enabled then fuse heading from ship external heading source with pitch and roll from usbl
    if(external_heading_enabled_)
    {
        //exclude heading from evologics msg
        quat_tf.setRPY(msg->rpy.x, msg->rpy.y, 0.0);

        tf2::Quaternion ship_quat;
        tf2::fromMsg(ship_geopose_.orientation, ship_quat);

        quat_tf = ship_quat * quat_tf;
    }
    // else we just use roll pitch and yaw from the usbl
    else
    {
        quat_tf.setRPY(msg->rpy.x, msg->rpy.y, msg->rpy.z);

    }

    quat_tf.normalize();
    usbl_orientation.quaternion = tf2::toMsg(quat_tf);
    usbl_orientation.header.frame_id = "usbl";
    usbl_orientation.header.stamp = ros::Time::now();

    usbl_orientation_pub_.publish(usbl_orientation);

    // Next we will broadcast the transformation from the tracking data
    static tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "usbl";
    transform_stamped.child_frame_id = "modem";

    transform_stamped.transform.translation.x = msg->xyz.x;
    transform_stamped.transform.translation.y = msg->xyz.y;
    transform_stamped.transform.translation.z = msg->xyz.z;

    // we don't have any information about the orientation of the asset being tracked. this could change with fusing acoustic messages...???
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();;

    broadcaster.sendTransform(transform_stamped);

    // Finally, transform from modem frame to gps frame and convert to latlong
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tf_buffer.lookupTransform("gps", "modem",ros::Time(0)); 
    
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        double lat, lon;
        // Calculate latitude and longitude of target point
        geod.Direct(ship_geopose_.position.latitude, ship_geopose_.position.longitude, 0.0, transform_stamped.transform.translation.y, lat, lon); // Move north
        geod.Direct(lat, lon, 90.0, transform_stamped.transform.translation.x, lat, lon); // Move east
        
        geographic_msgs::GeoPointStamped geopoint;
        geopoint.header.frame_id = "modem";
        geopoint.header.stamp = ros::Time::now();
        geopoint.position.latitude = lat;
        geopoint.position.longitude = lon;
        geopoint.position.altitude = transform_stamped.transform.translation.z;

        track_latlong_pub_.publish(geopoint);
    
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "acomms_localization");

    AcommsLocalization d;

    ros::spin();

    return 0;
}
