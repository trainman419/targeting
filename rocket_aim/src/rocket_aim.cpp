/*
 * rocket_aim: a node for aiming a USB nerf turret based on targets
 * detected by a vision algorithm and a kinect
 *
 * Author: Austin Hendrix
 */

#include <math.h>

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <target_detector/PointArray.h>

double dx, dy, dz;

double dist(geometry_msgs::Point p) {
   return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

void pointsCb( const target_detector::PointArrayConstPtr & msg ) {
   // coordinates are in optical frame, and I'm too lazy to translate them
   // right now.
   // z: distance to target
   // x: right
   // y: down
   if( msg->points.size() < 1 ) return;
   geometry_msgs::Point target = msg->points.front();
   double d = dist(target);

   // choose the closest target (to the camera)
   BOOST_FOREACH( geometry_msgs::Point p, msg->points ) {
      if( dist(p) < d ) {
         d = dist(p);
         target = p;
      }
   }

   ROS_INFO("Chosen target: (%lf, %lf, %lf)", target.x, target.y, target.z);

   double x = target.x - dx;
   double y = target.y - dy;
   double z = target.z - dz;
   double yaw = atan2( x, z );

   double pitch = atan2( -y, sqrt( x*x + z*z ) );

   ROS_INFO("Target yaw, pitch: %lf, %lf", yaw*180.0/M_PI, pitch*180.0/M_PI);
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "rocket_aim");

   ros::NodeHandle nh;

   nh.param<double>("x_offset", dx, 0.0);
   nh.param<double>("y_offset", dy, 0.0);
   nh.param<double>("z_offset", dz, 0.0);

   ros::Subscriber s = nh.subscribe<target_detector::PointArray>("targets", 1,
         &pointsCb);

   ROS_INFO("Rocket aiming ready");

   ros::spin();
   return 0;
}
